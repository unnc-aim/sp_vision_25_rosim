#include "io/sim_board.hpp"

#include <thread>

#include <yaml-cpp/yaml.h>

#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/shoot_cmd.hpp"
#include "vision_interfaces/msg/auto_aim.hpp"  // 【新增】用于发布自瞄状态
#include "tools/math_tools.hpp"  // 【新增】用于 tools::eulers()

using namespace std::chrono_literals;

// ============================================================================
// 仿真板卡实现 - 从 ROS2 IMU 话题获取云台姿态，发布控制指令
// ============================================================================
// 设计说明：
//   1. 通过 ROS2 订阅仿真 IMU 话题（Gazebo 发布）
//   2. 保持与原 CBoard::imu_at() 相同的四元数插值逻辑
//   3. 时间戳从 ROS 消息头转换为 steady_clock（与其他模块统一）
//   4. 发布云台控制指令到 Gazebo（yaw/pitch 角度和射击指令）
// 坐标系转换：
//   Gazebo 默认使用 ENU 坐标系，可能与实车 IMU 不同
//   通过 config 中的 R_sim2real 矩阵进行转换
// 线程模型：
//   使用 SingleThreadedExecutor + spin_some() 处理消息，与 SimCamera 一致
// ROS2 生命周期：
//   不调用 rclcpp::init/shutdown，由外部 io::ROS2 统一管理
// ============================================================================

namespace io
{

SimBoard::SimBoard(const std::string & config_path, const std::string & imu_topic)
: queue_(5000),  // 队列大小与原 CBoard 保持一致
  R_sim2real_(Eigen::Matrix3d::Identity())  // 默认无坐标系转换
{
  // ----------------------------------------------------
  // 【1】读取配置文件
  // ----------------------------------------------------
  try {
    auto yaml = YAML::LoadFile(config_path);

    // 读取坐标系转换矩阵
    if (yaml["R_sim2real"]) {
      auto R_data = yaml["R_sim2real"].as<std::vector<double>>();
      if (R_data.size() == 9) {
        R_sim2real_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_data.data());
        tools::logger()->info(
          "[SimBoard] 加载坐标系转换矩阵 R_sim2real:\n"
          "  [{:.4f}, {:.4f}, {:.4f}]\n"
          "  [{:.4f}, {:.4f}, {:.4f}]\n"
          "  [{:.4f}, {:.4f}, {:.4f}]",
          R_sim2real_(0, 0), R_sim2real_(0, 1), R_sim2real_(0, 2),
          R_sim2real_(1, 0), R_sim2real_(1, 1), R_sim2real_(1, 2),
          R_sim2real_(2, 0), R_sim2real_(2, 1), R_sim2real_(2, 2));
      }
    } else {
      tools::logger()->info("[SimBoard] 未配置 R_sim2real，使用单位矩阵");
    }

    // 读取控制话题名称
    if (yaml["gimbal_cmd_topic"]) {
      gimbal_cmd_topic_ = yaml["gimbal_cmd_topic"].as<std::string>();
      tools::logger()->info("[SimBoard] 云台控制话题: {}", gimbal_cmd_topic_);
    }
    if (yaml["shoot_cmd_topic"]) {
      shoot_cmd_topic_ = yaml["shoot_cmd_topic"].as<std::string>();
      tools::logger()->info("[SimBoard] 射击控制话题: {}", shoot_cmd_topic_);
    }
    
    // 【新增】读取云台状态话题（优先使用，比 IMU 更准确）
    if (yaml["gimbal_state_topic"]) {
      gimbal_state_topic_ = yaml["gimbal_state_topic"].as<std::string>();
      use_gimbal_state_ = true;
      tools::logger()->info("[SimBoard] 云台状态话题: {} (优先使用)", gimbal_state_topic_);
    }

    // 【新增】读取自瞄激活话题（用于步兵右键激活）
    if (yaml["auto_aim_activate_topic"]) {
      auto_aim_activate_topic_ = yaml["auto_aim_activate_topic"].as<std::string>();
      tools::logger()->info("[SimBoard] 自瞄激活话题: {}", auto_aim_activate_topic_);
    }

    // 【新增】读取自瞄状态发布话题（供 player_web 使用）
    if (yaml["auto_aim_cmd_topic"]) {
      auto_aim_cmd_topic_ = yaml["auto_aim_cmd_topic"].as<std::string>();
      tools::logger()->info("[SimBoard] 自瞄状态发布话题: {}", auto_aim_cmd_topic_);
    }

    // 【新增】读取仿真弹速
    if (yaml["sim_bullet_speed"]) {
      bullet_speed = yaml["sim_bullet_speed"].as<double>();
      tools::logger()->info("[SimBoard] 仿真弹速: {:.2f} m/s", bullet_speed);
    } else {
      tools::logger()->info("[SimBoard] 使用默认弹速: {:.2f} m/s", bullet_speed);
    }

    // 【新增】读取自瞄激活状态
    if (yaml["auto_aim_enabled"]) {
      auto_aim_enabled = yaml["auto_aim_enabled"].as<bool>();
    }
    tools::logger()->info("[SimBoard] 自瞄初始状态: {}", auto_aim_enabled.load() ? "激活" : "未激活");

  } catch (const std::exception & e) {
    tools::logger()->warn("[SimBoard] 读取配置文件失败: {}，使用默认值", e.what());
  }

  // ----------------------------------------------------
  // 【2】检查 ROS2 是否已初始化
  // ----------------------------------------------------
  if (!rclcpp::ok()) {
    tools::logger()->info("[SimBoard] ROS2 未初始化，正在初始化...");
    rclcpp::init(0, nullptr);
    tools::logger()->info("[SimBoard] ROS2 初始化完成");
  }

  // ----------------------------------------------------
  // 【3】创建 ROS2 节点
  // ----------------------------------------------------
  auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  std::string node_name = "sim_board_" + std::to_string(now_ns % 1000000);

  node_ = std::make_shared<rclcpp::Node>(node_name);
  tools::logger()->info("[SimBoard] 创建节点: {}", node_name);

  // 初始化插值用的前后帧数据
  // 【重要】设置 initialized_ = false，等待第一帧有效数据
  data_ahead_.timestamp = std::chrono::steady_clock::now();
  data_ahead_.q = Eigen::Quaterniond::Identity();
  data_behind_ = data_ahead_;
  initialized_ = false;  // 标记未初始化

  // ----------------------------------------------------
  // 【4】创建订阅器 - 云台状态/IMU 话题
  // ----------------------------------------------------
  if (use_gimbal_state_ && !gimbal_state_topic_.empty()) {
    // 优先使用 gimbal_state 话题（更准确的 yaw/pitch）
    gimbal_state_subscription_ = node_->create_subscription<rmoss_interfaces::msg::Gimbal>(
      gimbal_state_topic_, 10,
      std::bind(&SimBoard::gimbal_state_callback, this, std::placeholders::_1));
    tools::logger()->info("[SimBoard] 订阅云台状态话题: {} (推荐)", gimbal_state_topic_);
  } else {
    // 回退到 IMU 话题
    imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 10,
      std::bind(&SimBoard::imu_callback, this, std::placeholders::_1));
    tools::logger()->info("[SimBoard] 订阅 IMU 话题: {}", imu_topic);
  }

  // ----------------------------------------------------
  // 【5】创建发布器 - 控制话题（如果配置了）
  // 使用 rmoss_interfaces 消息类型，与 Gazebo 仿真器兼容
  // ----------------------------------------------------
  if (!gimbal_cmd_topic_.empty()) {
    gimbal_publisher_ = node_->create_publisher<rmoss_interfaces::msg::GimbalCmd>(
      gimbal_cmd_topic_, 10);
    tools::logger()->info("[SimBoard] 创建云台控制发布器: {} (GimbalCmd)", gimbal_cmd_topic_);
    publish_enabled_ = true;
  }

  if (!shoot_cmd_topic_.empty()) {
    shoot_publisher_ = node_->create_publisher<rmoss_interfaces::msg::ShootCmd>(
      shoot_cmd_topic_, 10);
    tools::logger()->info("[SimBoard] 创建射击控制发布器: {} (ShootCmd)", shoot_cmd_topic_);
    publish_enabled_ = true;
  }

  // 【新增 - 5.1】创建自瞄状态发布器（供 player_web 使用）
  if (!auto_aim_cmd_topic_.empty()) {
    auto_aim_cmd_publisher_ = node_->create_publisher<vision_interfaces::msg::AutoAim>(
      auto_aim_cmd_topic_, 10);
    tools::logger()->info("[SimBoard] 创建自瞄状态发布器: {} (AutoAim)", auto_aim_cmd_topic_);
  }

  if (!publish_enabled_) {
    tools::logger()->info("[SimBoard] 未配置控制话题，send() 将仅输出日志");
  }

  // ----------------------------------------------------
  // 【新增 - 5.1】创建自瞄激活订阅器
  // 用于步兵等机器人的动态自瞄激活（右键按下时激活）
  // ----------------------------------------------------
  if (!auto_aim_activate_topic_.empty()) {
    auto_aim_activate_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      auto_aim_activate_topic_, 10,
      std::bind(&SimBoard::auto_aim_activate_callback, this, std::placeholders::_1));
    tools::logger()->info("[SimBoard] 订阅自瞄激活话题: {}", auto_aim_activate_topic_);
    tools::logger()->info("[SimBoard] 自瞄将由操作手终端控制（默认状态: {}）",
      auto_aim_enabled.load() ? "激活" : "未激活");
  }

  // ----------------------------------------------------
  // 【6】启动 ROS2 消息处理线程
  // ----------------------------------------------------
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  running_ = true;
  spin_thread_ = std::thread([this]() {
    while (running_ && rclcpp::ok()) {
      executor_->spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  tools::logger()->info("[SimBoard] SimBoard 初始化完成");
}

SimBoard::~SimBoard()
{
  tools::logger()->info("[SimBoard] 关闭 SimBoard...");

  running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  if (executor_) {
    executor_->remove_node(node_);
  }

  imu_subscription_.reset();
  gimbal_publisher_.reset();
  shoot_publisher_.reset();
  node_.reset();
  executor_.reset();

  tools::logger()->info("[SimBoard] SimBoard 已关闭");
}

// ============================================================================
// IMU 消息回调 - 从 ROS 消息提取四元数和时间戳
// ============================================================================
void SimBoard::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // 【时间戳转换】
  int64_t nanoseconds =
    static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(msg->header.stamp.nanosec);

  auto timestamp = tools::TimeConverter::from_nanoseconds(nanoseconds);

  // 【四元数提取】
  Eigen::Quaterniond q_sim(
    msg->orientation.w,
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z);

  q_sim.normalize();

  // 【坐标系转换】
  Eigen::Matrix3d R_sim = q_sim.toRotationMatrix();
  Eigen::Matrix3d R_real = R_sim2real_ * R_sim * R_sim2real_.transpose();
  Eigen::Quaterniond q_real(R_real);
  q_real.normalize();

  // 推入队列供 imu_at() 使用
  queue_.push({q_real, timestamp});
}

// ============================================================================
// 云台状态消息回调 - 从 yaw/pitch 转换为四元数
// ============================================================================
void SimBoard::gimbal_state_callback(const rmoss_interfaces::msg::Gimbal::SharedPtr msg)
{
  // 【时间戳】使用当前时间（gimbal_state 消息没有时间戳）
  auto timestamp = std::chrono::steady_clock::now();

  // 【从 yaw/pitch 构建四元数】
  // Gazebo 云台状态给出的是云台相对于底盘的 yaw 和 pitch
  // 使用 ZYX 内旋顺序（与 dm_imu.cpp 和 tools::eulers 一致）
  // 
  // 正确顺序：yaw * pitch * roll（roll=0）
  // 这样 tools::eulers(q, 2, 1, 0) 可以正确还原出 yaw 和 pitch
  double yaw = msg->yaw;
  double pitch = msg->pitch;
  
  // 【新增】缓存云台相对于底盘的角度（用于 send() 中的坐标系转换）
  last_gimbal_state_yaw_.store(yaw, std::memory_order_relaxed);
  last_gimbal_state_pitch_.store(pitch, std::memory_order_relaxed);
  
  // 【调试】原始云台状态
  static int gimbal_debug_count = 0;
  if (gimbal_debug_count < 20 || gimbal_debug_count % 100 == 0) {
    tools::logger()->info("[云台状态] 原始: yaw={:.2f}° pitch={:.2f}°", yaw * 57.3, pitch * 57.3);
  }
  
  // 【修复】使用 ZYX 内旋顺序：yaw * pitch（与 dm_imu.cpp 一致）
  // 四元数乘法：q = Rz(yaw) * Ry(pitch) * Rx(roll)，roll=0 时省略
  Eigen::AngleAxisd yaw_rotation(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch_rotation(pitch, Eigen::Vector3d::UnitY());
  
  // ZYX 顺序：先 yaw 再 pitch（内旋 = 从左到右乘）
  Eigen::Quaterniond q = yaw_rotation * pitch_rotation;
  q.normalize();
  
  // 【调试】转换后的四元数对应的欧拉角（使用统一的 tools::eulers）
  if (gimbal_debug_count < 20 || gimbal_debug_count % 100 == 0) {
    Eigen::Vector3d euler = tools::eulers(q, 2, 1, 0);  // ZYX 顺序
    tools::logger()->info("[云台状态] 构建四元数: w={:.4f} x={:.4f} y={:.4f} z={:.4f}",
      q.w(), q.x(), q.y(), q.z());
    tools::logger()->info("[云台状态] 验证 (tools::eulers ZYX): yaw={:.2f}° pitch={:.2f}° roll={:.2f}°",
      euler[0] * 57.3, euler[1] * 57.3, euler[2] * 57.3);
  }
  gimbal_debug_count++;

  // 推入队列供 imu_at() 使用
  queue_.push({q, timestamp});
}

// ============================================================================
// 自瞄激活回调 - 用于步兵右键激活
// ============================================================================
// 说明：
//   操作手终端发布 Bool 消息，控制自瞄是否激活
//   用于步兵等机器人的"右键按下时激活"功能
// ============================================================================
void SimBoard::auto_aim_activate_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool new_state = msg->data;
  bool old_state = auto_aim_enabled.exchange(new_state);
  
  if (old_state != new_state) {
    tools::logger()->info("[SimBoard] 自瞄激活状态变更: {} -> {}",
      old_state ? "激活" : "未激活",
      new_state ? "激活" : "未激活");
  }
}

// ============================================================================
// 按时间戳查询姿态 - 仿真模式简化版
// ============================================================================
// 【重要】仿真模式下不做时间插值，直接返回最新的云台状态
// 原因：
//   1. 真实 IMU 数据有硬件时间戳，可以精确插值
//   2. 仿真的 gimbal_state 用的是接收时间，与图像时间戳不同步
//   3. 尝试插值会导致 k 系数异常（如 -20000），结果完全错误
// ============================================================================
Eigen::Quaterniond SimBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  (void)timestamp;  // 仿真模式下忽略时间戳参数
  
  // 【初始化】第一次调用时，阻塞等待第一帧云台数据
  if (!initialized_) {
    tools::logger()->info("[imu_at] 等待第一帧云台状态数据...");
    queue_.pop(data_behind_);  // 阻塞等待
    initialized_ = true;
    
    Eigen::Vector3d euler = tools::eulers(data_behind_.q, 2, 1, 0);
    tools::logger()->info("[imu_at] 初始化完成: yaw={:.2f}° pitch={:.2f}°",
      euler[0] * 57.3, euler[1] * 57.3);
  }
  
  // 【简化处理】非阻塞读取队列中所有数据，保留最新的一个
  IMUData latest;
  while (queue_.try_pop_nowait(latest)) {
    data_behind_ = latest;  // 更新缓存
  }
  
  // 使用缓存的最新数据
  Eigen::Quaterniond q = data_behind_.q.normalized();
  
  // 【调试】输出返回的姿态
  static int imu_at_debug_count = 0;
  if (imu_at_debug_count < 20 || imu_at_debug_count % 100 == 0) {
    Eigen::Vector3d euler = tools::eulers(q, 2, 1, 0);
    tools::logger()->info("[imu_at] 返回最新姿态: yaw={:.2f}° pitch={:.2f}° roll={:.2f}°",
      euler[0] * 57.3, euler[1] * 57.3, euler[2] * 57.3);
  }
  imu_at_debug_count++;

  return q;
}

// ============================================================================
// 发送控制指令 - 发布到 ROS2 话题
// ============================================================================
// 消息格式（rmoss_interfaces）：
//   云台控制 (GimbalCmd): yaw/pitch 绝对角度 (单位: rad)
//   射击控制 (ShootCmd): 射击数量和弹速
// 
// 【重要】坐标系转换：
//   sp_vision 输出的 command.yaw/pitch 是目标在**世界坐标系**中的方位角
//   Gazebo 云台控制器期望的是云台**相对于底盘**的目标角度
//   
//   转换公式：
//     目标_相对底盘 = 目标_世界系 - 底盘_世界系朝向
//   
//   由于仿真中我们不直接获取底盘世界系朝向，使用如下方法：
//     底盘_世界系朝向 = 云台_世界系 - 云台_相对底盘
//     其中：
//       - 云台_世界系 = tools::eulers(data_behind_.q, 2, 1, 0)[0] (从四元数计算)
//       - 云台_相对底盘 = last_gimbal_state_yaw_ (从 gimbal_state 回调缓存)
//
// 【新增 - 自瞄激活检查】：
//   如果 auto_aim_enabled 为 false，跳过云台控制指令发布
//   射击指令不受影响（即使自瞄未激活，射击仍然可用）
// ============================================================================
void SimBoard::send(Command command) const
{
  // 【调试日志】限制频率
  static int send_debug_count = 0;
  
  // 如果未配置发布话题，仅输出日志
  if (!publish_enabled_) {
    if (send_debug_count < 10) {
      tools::logger()->warn("[SimBoard::send] 发布未启用! publish_enabled_=false");
    }
    send_debug_count++;
    return;
  }

  // 【新增 - 自瞄激活检查】
  // 如果自瞄未激活，跳过云台控制（但仍处理射击指令）
  bool aim_enabled = auto_aim_enabled.load();
  
  // 【新增】发布自瞄状态到 player_web（无论自瞄是否激活都发布）
  // 这样 player_web 可以知道 sp_vision 是否有检测到目标
  if (auto_aim_cmd_publisher_) {
    vision_interfaces::msg::AutoAim auto_aim_msg;
    auto_aim_msg.tracking = command.control;  // control 为 true 表示有跟踪目标
    auto_aim_msg.aim_yaw = static_cast<float>(command.yaw);
    auto_aim_msg.aim_pitch = static_cast<float>(command.pitch);
    auto_aim_msg.fire = command.shoot;
    auto_aim_cmd_publisher_->publish(auto_aim_msg);
    
    // 调试日志
    if (send_debug_count < 20 || send_debug_count % 100 == 0) {
      tools::logger()->info("[SimBoard::send] 发布 AutoAim: tracking={}, yaw={:.2f}°, pitch={:.2f}°, fire={}",
        auto_aim_msg.tracking, command.yaw * 57.3, command.pitch * 57.3, auto_aim_msg.fire);
    }
  }
  
  if (!aim_enabled) {
    // 自瞄未激活时，仅处理射击指令（如果有）
    if (shoot_publisher_ && command.shoot) {
      rmoss_interfaces::msg::ShootCmd shoot_msg;
      shoot_msg.projectile_num = 1;
      shoot_msg.projectile_velocity = static_cast<float>(bullet_speed);
      shoot_publisher_->publish(shoot_msg);
    }
    
    // 限制日志频率
    if (send_debug_count < 5 || send_debug_count % 200 == 0) {
      tools::logger()->info("[SimBoard::send] 自瞄未激活，跳过云台控制");
    }
    send_debug_count++;
    return;
  }

  // 【发布云台控制指令】
  // 使用 rmoss_interfaces/msg/GimbalCmd 消息类型
  if (gimbal_publisher_ && command.control) {
    // 【坐标系转换】将世界系目标角度转换为相对底盘的目标角度
    // 
    // 当前云台姿态（从 data_behind_.q 获取，这是世界系中的云台朝向）
    Eigen::Vector3d gimbal_world_euler = tools::eulers(data_behind_.q, 2, 1, 0);
    double gimbal_world_yaw = gimbal_world_euler[0];    // 云台在世界系中的 yaw
    double gimbal_world_pitch = gimbal_world_euler[1];  // 云台在世界系中的 pitch
    
    // 从 gimbal_state 获取云台相对于底盘的角度（缓存在回调中）
    double gimbal_chassis_yaw = last_gimbal_state_yaw_;
    double gimbal_chassis_pitch = last_gimbal_state_pitch_;
    
    // 计算底盘在世界系中的朝向
    double chassis_world_yaw = gimbal_world_yaw - gimbal_chassis_yaw;
    
    // 将目标世界系角度转换为相对底盘的角度
    double target_chassis_yaw = command.yaw - chassis_world_yaw;
    double target_chassis_pitch = command.pitch;  // pitch 不受底盘旋转影响
    
    // 角度归一化到 [-π, π]
    while (target_chassis_yaw > M_PI) target_chassis_yaw -= 2 * M_PI;
    while (target_chassis_yaw < -M_PI) target_chassis_yaw += 2 * M_PI;
    
    // 【调试】坐标系转换过程
    if (send_debug_count < 50 || send_debug_count % 100 == 0) {
      tools::logger()->info("[SimBoard::send] ===== 坐标系转换 =====");
      tools::logger()->info("[SimBoard::send] 输入(世界系): yaw={:.2f}° pitch={:.2f}°",
        command.yaw * 57.3, command.pitch * 57.3);
      tools::logger()->info("[SimBoard::send] 云台世界系: yaw={:.2f}° pitch={:.2f}°",
        gimbal_world_yaw * 57.3, gimbal_world_pitch * 57.3);
      tools::logger()->info("[SimBoard::send] 云台相对底盘: yaw={:.2f}° pitch={:.2f}°",
        gimbal_chassis_yaw * 57.3, gimbal_chassis_pitch * 57.3);
      tools::logger()->info("[SimBoard::send] 底盘世界系朝向: yaw={:.2f}°",
        chassis_world_yaw * 57.3);
      tools::logger()->info("[SimBoard::send] 输出(相对底盘): yaw={:.2f}° pitch={:.2f}°",
        target_chassis_yaw * 57.3, target_chassis_pitch * 57.3);
    }
    
    rmoss_interfaces::msg::GimbalCmd gimbal_msg;
    gimbal_msg.yaw_type = rmoss_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
    gimbal_msg.pitch_type = rmoss_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
    gimbal_msg.position.yaw = static_cast<float>(target_chassis_yaw);    // 相对底盘的 yaw
    gimbal_msg.position.pitch = static_cast<float>(target_chassis_pitch);  // pitch
    
    gimbal_publisher_->publish(gimbal_msg);
  }
  
  send_debug_count++;

  // 【发布射击控制指令】
  // 使用 rmoss_interfaces/msg/ShootCmd 消息类型
  if (shoot_publisher_ && command.shoot) {
    rmoss_interfaces::msg::ShootCmd shoot_msg;
    shoot_msg.projectile_num = 1;  // 每次射击 1 发
    shoot_msg.projectile_velocity = static_cast<float>(bullet_speed);  // 使用配置的弹速
    shoot_publisher_->publish(shoot_msg);
  }
}

}  // namespace io
