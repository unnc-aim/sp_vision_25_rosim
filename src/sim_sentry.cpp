/**
 * @file sim_sentry.cpp
 * @brief 仿真环境下的哨兵视觉程序
 * 
 * 【说明】
 * 这是 src/sentry.cpp 在仿真环境中的对应版本
 * 仅修改了 Camera 和 CBoard 的初始化方式，其余逻辑完全相同
 * 
 * 【编译】
 *   cd ~/ros_ws && colcon build --packages-select sp_vision
 * 
 * 【运行】
 *   # 方式1：使用默认配置
 *   ros2 run sp_vision sim_sentry
 *
 *   # 方式2：指定配置文件
 *   ros2 run sp_vision sim_sentry configs/sim_sentry.yaml
 *
 *   # 方式3：命令行指定话题（覆盖配置文件）
 *   ros2 run sp_vision sim_sentry --image-topic=/robot/camera/image --imu-topic=/robot/imu
 * 
 * 【调试图像】
 *   在 rqt 中查看: rqt_image_view /sp_vision/debug_image
 *   在 rviz 中添加 Image Display，话题选择 /sp_vision/debug_image
 * 
 * 【自瞄激活】
 *   仿真模式下自瞄始终激活（由 auto_aim_enabled 配置控制）
 *   不需要像实车一样通过操作手右键激活
 */

#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// ament_index_cpp for resolving package share directory
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                              | 输出命令行参数说明}"
  "{@config-path   | configs/sim_sentry.yaml   | 位置参数，yaml配置文件路径 }"
  "{image-topic    | /red_standard_robot1/front_industrial_camera/image | 【仿真】图像话题}"
  "{imu-topic      | /red_standard_robot1/livox/imu | 【仿真】IMU话题}"
  "{debug          | false                        | 是否发布调试图像到 ROS 话题}";

// ============================================================================
// 调试图像发布器 - 在 rqt/rviz 中查看检测结果
// ============================================================================
class DebugImagePublisher
{
public:
  DebugImagePublisher()
  {
    node_ = std::make_shared<rclcpp::Node>("sp_vision_debug");
    publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/sp_vision/debug_image", 10);
    
    // 启动 spin 线程（使用 spin_some 避免阻塞）
    running_ = true;
    spin_thread_ = std::thread([this]() {
      while (running_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
    
    tools::logger()->info("[DebugImagePublisher] 发布调试图像到 /sp_vision/debug_image");
  }
  
  ~DebugImagePublisher()
  {
    // 注意：不在这里调用 rclcpp::shutdown()
    // ROS2 生命周期由 io::ROS2 或主程序统一管理
    running_ = false;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
  }
  
  void publish(const cv::Mat & img)
  {
    if (img.empty()) return;
    
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
    msg->header.stamp = node_->now();
    publisher_->publish(*msg);
  }
  
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
};

// ============================================================================
// 绘制检测结果
// ============================================================================
void draw_debug_info(
  cv::Mat & img,
  const std::list<auto_aim::Armor> & armors,
  const std::list<auto_aim::Target> & targets,
  const std::string & tracker_state,
  const io::Command & command)
{
  // 绘制检测到的装甲板
  for (const auto & armor : armors) {
    // 绘制四个角点
    tools::draw_points(img, armor.points, {0, 255, 0}, 2);
    
    // 绘制信息文字
    auto info = fmt::format("{:.2f} {}", 
      armor.confidence, 
      auto_aim::ARMOR_NAMES[armor.name]);
    tools::draw_text(img, info, cv::Point(armor.center.x, armor.center.y - 20), {0, 255, 0});
  }
  
  // 绘制跟踪目标信息（用红色标记）
  for (const auto & target : targets) {
    // 显示目标名称
    tools::draw_text(img, fmt::format("Target: {}", auto_aim::ARMOR_NAMES[target.name]), 
      {10, 130}, {0, 0, 255});
  }
  
  // 绘制状态信息
  int y = 30;
  tools::draw_text(img, fmt::format("State: {}", tracker_state), {10, y}, {255, 255, 0});
  y += 25;
  tools::draw_text(img, fmt::format("Armors: {}", armors.size()), {10, y}, {255, 255, 0});
  y += 25;
  tools::draw_text(img, fmt::format("Targets: {}", targets.size()), {10, y}, {255, 255, 0});
  y += 25;
  if (command.control) {
    tools::draw_text(img, 
      fmt::format("Cmd: yaw={:.2f} pitch={:.2f}", command.yaw * 57.3, command.pitch * 57.3), 
      {10, y}, {0, 255, 255});
  }
}

int main(int argc, char * argv[])
{
  tools::Exiter exiter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto debug_mode = cli.get<bool>("debug");

  // Resolve config_path: if relative and not found, try package share directory
  std::string pkg_share;
  if (!std::filesystem::exists(config_path)) {
    try {
      pkg_share = ament_index_cpp::get_package_share_directory("sp_vision");
      auto resolved = pkg_share + "/" + config_path;
      if (std::filesystem::exists(resolved)) {
        config_path = resolved;
      }
    } catch (...) {
      // ament_index not available, keep original path
    }
  }

  // Change working directory to package share so that relative asset paths work
  if (!pkg_share.empty()) {
    std::filesystem::current_path(pkg_share);
    tools::logger()->info("[SimSentry] 工作目录切换到: {}", pkg_share);
  }

  // 【仿真模式】读取仿真话题名称
  auto image_topic = cli.get<std::string>("image-topic");
  auto imu_topic = cli.get<std::string>("imu-topic");

  tools::logger()->info("[SimSentry] ========================================");
  tools::logger()->info("[SimSentry] 仿真哨兵视觉程序启动");
  tools::logger()->info("[SimSentry] ========================================");
  tools::logger()->info("[SimSentry] 配置文件: {}", config_path);
  tools::logger()->info("[SimSentry] 图像话题: {}", image_topic);
  tools::logger()->info("[SimSentry] IMU话题: {}", imu_topic);
  tools::logger()->info("[SimSentry] 调试模式: {}", debug_mode ? "开启" : "关闭");

  // 【仿真初始化】传入话题名称，自动使用仿真驱动
  io::Camera camera(config_path, image_topic);      // 【仿真】订阅图像话题
  io::CBoard cboard(config_path, imu_topic);        // 【仿真】订阅 IMU 话题

  // 调试图像发布器（可选）
  std::unique_ptr<DebugImagePublisher> debug_publisher;
  if (debug_mode) {
    debug_publisher = std::make_unique<DebugImagePublisher>();
  }

  auto_aim::YOLO yolo(config_path, false);  // debug=false，不使用 cv::imshow
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  tools::logger()->info("[SimSentry] 初始化完成，等待图像数据...");

  while (!exiter.exit() && rclcpp::ok()) {
    // 【核心逻辑】
    camera.read(img, timestamp);
    
    // 检查是否收到空图像（可能是退出信号）
    if (img.empty()) {
      if (exiter.exit() || !rclcpp::ok()) {
        break;  // 正常退出
      }
      continue;  // 还没收到图像，继续等待
    }
    
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q, cboard.is_direct_gimbal2world());

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 【调试】当前云台姿态
    static int gimbal_debug_counter = 0;
    if (gimbal_debug_counter < 20 || gimbal_debug_counter % 100 == 0) {
      tools::logger()->info("[主循环] ========== 帧 {} ==========", gimbal_debug_counter);
      tools::logger()->info("[主循环] 云台当前姿态: yaw={:.2f}° pitch={:.2f}° roll={:.2f}°",
        gimbal_pos[0] * 57.3, gimbal_pos[1] * 57.3, gimbal_pos[2] * 57.3);
    }

    auto armors = yolo.detect(img);

    // 【调试】检测结果
    if ((gimbal_debug_counter < 20 || gimbal_debug_counter % 100 == 0) && !armors.empty()) {
      tools::logger()->info("[主循环] 检测到 {} 个装甲板", armors.size());
    }
    gimbal_debug_counter++;

    auto targets = tracker.track(armors, timestamp);

    // 【仿真简化】去掉全向感知和敌方状态订阅（仿真中可能没有这些话题）
    // 【重要】仿真模式下必须设置 to_now = false，因为：
    //   - timestamp 来自仿真时间戳（通过 TimeConverter::from_nanoseconds）
    //   - 如果 to_now = true，会用 steady_clock::now()（系统时间）减去仿真时间
    //   - 两者时间基准不同，导致 dt 异常大，EKF predict 发散
    io::Command command = aimer.aim(targets, timestamp, cboard.bullet_speed, false);

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    // 发送控制指令到仿真器
    cboard.send(command);

    // 【调试图像发布】
    if (debug_publisher) {
      cv::Mat debug_img = img.clone();
      draw_debug_info(debug_img, armors, targets, tracker.state(), command);
      debug_publisher->publish(debug_img);
    }

    // 定时输出状态日志
    static auto last_log = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 2) {
      tools::logger()->info(
        "[SimSentry] 检测: {} 装甲板, {} 目标, 状态: {}, 弹速: {:.1f} m/s",
        armors.size(), targets.size(), tracker.state(), cboard.bullet_speed);
      if (command.control) {
        tools::logger()->info(
          "[SimSentry] 控制: yaw={:.2f}° pitch={:.2f}° shoot={}",
          command.yaw * 57.3, command.pitch * 57.3, command.shoot);
      }
      last_log = now;
    }
  }

  tools::logger()->info("[SimSentry] 程序退出");
  return 0;
}
