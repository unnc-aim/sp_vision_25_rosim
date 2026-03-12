#ifndef IO__SIM_BOARD_HPP
#define IO__SIM_BOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"  // 【新增】用于自瞄激活话题
#include "rmoss_interfaces/msg/gimbal.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/shoot_cmd.hpp"
#include "vision_interfaces/msg/auto_aim.hpp"  // 【新增】用于发布自瞄状态到 player_web

#include "io/command.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/time_converter.hpp"

// ============================================================================
// 仿真板卡 - 通过 ROS2 话题获取云台姿态和发布控制指令
// ============================================================================
// 用途：
//   替代真实 CBoard（下位机控制板），接收仿真云台姿态话题
//   提供与原 CBoard 接口兼容的 imu_at() 和 send() 方法
// 输入话题：
//   - IMU 话题：获取云台姿态四元数（如 /red_standard_robot1/livox/imu）
// 输出话题：
//   - 云台控制话题：发布 yaw/pitch 角度（如 /red_standard_robot1/cmd_gimbal_joint）
//   - 射击控制话题：发布射击指令（如 /red_standard_robot1/cmd_shoot）
// 时间戳处理：
//   从 ROS IMU 消息头提取 header.stamp，转换为 steady_clock::time_point
// 坐标系处理：
//   Gazebo 使用 ENU 坐标系（X-前, Y-左, Z-上）
//   需要通过 R_sim2real_ 转换到实车 IMU 坐标系
// 自瞄激活：
//   仿真模式下通过配置文件设置 auto_aim_enabled，默认为 true（始终激活）
// 注意：
//   ROS2 初始化由外部 io::ROS2 统一管理，本类不调用 rclcpp::init/shutdown
// ============================================================================

namespace io
{

class SimBoard
{
public:
  // config_path: 配置文件路径（读取 R_sim2real 坐标系转换矩阵和话题名称）
  // imu_topic: 仿真 IMU 话题，如 "/red_standard_robot1/livox/imu"
  SimBoard(const std::string & config_path, const std::string & imu_topic);

  ~SimBoard();

  // 【接口】按时间戳查询云台姿态四元数
  // 实现四元数插值对齐，与原 CBoard::imu_at() 逻辑相同
  // 返回的四元数已经过坐标系转换，可直接用于 solver
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  // 【接口】发送控制指令到仿真器
  // 发布云台角度和射击指令到 ROS2 话题
  void send(Command command) const;

  // 【公开状态】模拟 CBoard 的公开成员变量
  // 这些值可通过配置文件设置，或在仿真中保持默认
  double bullet_speed = 15.0;  // 弹速 (m/s)，可在配置文件中设置 sim_bullet_speed
  std::atomic<bool> auto_aim_enabled{true};  // 【修改】仿真模式下自瞄是否激活（支持动态更新）

  // 【接口】查询 imu_at() 返回的四元数是否是直接的 gimbal2world 格式
  // 如果是，solver 不需要做额外的坐标系转换
  bool is_direct_gimbal2world() const { return use_gimbal_state_; }

private:
  // 【数据结构】与原 CBoard 保持一致
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  // ROS2 节点和订阅器/发布器
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_aim_activate_subscription_;  // 【新增】自瞄激活订阅器
  rclcpp::Publisher<rmoss_interfaces::msg::GimbalCmd>::SharedPtr gimbal_publisher_;
  rclcpp::Publisher<rmoss_interfaces::msg::ShootCmd>::SharedPtr shoot_publisher_;
  rclcpp::Publisher<vision_interfaces::msg::AutoAim>::SharedPtr auto_aim_cmd_publisher_;  // 【新增】发布自瞄状态
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{true};

  // IMU 数据队列（从回调线程推入，从 imu_at() 取出并插值）
  tools::ThreadSafeQueue<IMUData> queue_;

  // 用于插值的前后两帧数据（与原 CBoard 一致）
  IMUData data_ahead_;
  IMUData data_behind_;

  // 【坐标系转换】
  // R_sim2real_: 仿真 IMU 坐标系 → 实车 IMU 坐标系的旋转矩阵
  Eigen::Matrix3d R_sim2real_;

  // 【控制话题名称】
  std::string gimbal_cmd_topic_;
  std::string shoot_cmd_topic_;
  std::string gimbal_state_topic_;  // 云台状态话题（yaw/pitch 格式）
  std::string auto_aim_activate_topic_;  // 【新增】自瞄激活话题
  std::string auto_aim_cmd_topic_;  // 【新增】自瞄状态发布话题（供 player_web 使用）
  bool publish_enabled_ = false;  // 是否启用发布
  bool use_gimbal_state_ = false;  // 是否使用 gimbal_state 话题（而非 IMU 话题）
  bool initialized_ = false;  // 是否收到第一帧有效云台数据

  // 【新增】缓存最新的 gimbal_state（云台相对于底盘的角度）
  // 用于 send() 中的坐标系转换
  mutable std::atomic<double> last_gimbal_state_yaw_{0.0};
  mutable std::atomic<double> last_gimbal_state_pitch_{0.0};

  // IMU 回调函数
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  // 云台状态回调函数（yaw/pitch 格式）
  void gimbal_state_callback(const rmoss_interfaces::msg::Gimbal::SharedPtr msg);
  // 【新增】自瞄激活回调函数
  void auto_aim_activate_callback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace io

#endif  // IO__SIM_BOARD_HPP
