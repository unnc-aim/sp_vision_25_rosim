#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

#if defined(SP_VISION_HAS_ROS2) && (SP_VISION_HAS_ROS2 == 1)
#include "sim_camera.hpp"  // 【新增 - 仿真支持】
#endif

namespace io
{

// 【原有构造函数】用于实车硬件驱动
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

// 【新增构造函数】用于仿真模式
// 当传入 image_topic 非空时，使用仿真相机
// image_topic 示例: "/red_standard_robot1/front_industrial_camera/image"
Camera::Camera(const std::string & config_path, const std::string & image_topic)
{
  if (image_topic.empty()) {
    // 如果话题为空，回退到原有的硬件驱动逻辑
    auto yaml = tools::load(config_path);
    auto camera_name = tools::read<std::string>(yaml, "camera_name");
    auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

    if (camera_name == "mindvision") {
      auto gamma = tools::read<double>(yaml, "gamma");
      auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
      camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
    } else if (camera_name == "hikrobot") {
      auto gain = tools::read<double>(yaml, "gain");
      auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
      camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
    } else {
      throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
    }
  } else {
    // 【新增 - 仿真模式】使用 ROS2 话题获取图像
#if defined(SP_VISION_HAS_ROS2) && (SP_VISION_HAS_ROS2 == 1)
    camera_ = std::make_unique<SimCamera>(config_path, image_topic);
#else
    throw std::runtime_error(
      "[Camera] Simulation mode requires ROS2 (rclcpp/sensor_msgs/cv_bridge). "
      "Please install ROS2 dependencies and source the ROS2 environment, or pass empty image_topic.");
#endif
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io