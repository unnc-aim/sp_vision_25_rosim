#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

class Camera
{
public:
  Camera(const std::string & config_path);

  // 【新增 - 仿真支持】带仿真话题的构造函数
  // image_topic: 仿真图像话题，如 "/red_standard_robot1/front_industrial_camera/image"
  // 当 image_topic 非空时，自动使用仿真相机而不是硬件驱动
  Camera(const std::string & config_path, const std::string & image_topic);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP