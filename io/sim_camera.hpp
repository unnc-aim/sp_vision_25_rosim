#ifndef IO__SIM_CAMERA_HPP
#define IO__SIM_CAMERA_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/time_converter.hpp"

// ============================================================================
// 仿真相机 - 通过 ROS2 话题获取图像和时间戳
// ============================================================================
// 用途：
//   替代真实硬件（MindVision / HikRobot），接收仿真图像话题
//   提供与原 Camera 接口兼容的 read() 方法
// 时间戳处理：
//   从 ROS 消息头提取 header.stamp，转换为 steady_clock::time_point
// 注意：
//   ROS2 初始化由外部 io::ROS2 统一管理，本类不调用 rclcpp::init/shutdown
// ============================================================================

namespace io
{

class SimCamera : public CameraBase
{
public:
  // config_path: 配置文件路径（用于读取图像参数，保持接口兼容）
  // image_topic: 仿真图像话题名称，如 "/red_standard_robot1/front_industrial_camera/image"
  SimCamera(const std::string & config_path, const std::string & image_topic);

  ~SimCamera() override;

  // 【接口】读取图像和时间戳，与原 Camera 接口一致
  // 阻塞直到有图像数据可用
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  // 【数据结构】与 MindVision/HikRobot 保持一致
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  // ROS2 节点和订阅器
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{true};

  // 数据队列（从回调线程推入，从 read() 线程取出）
  // 队列大小为 1，与硬件相机行为一致（只保留最新帧）
  tools::ThreadSafeQueue<CameraData> queue_;

  // 图像回调函数
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  // namespace io

#endif  // IO__SIM_CAMERA_HPP
