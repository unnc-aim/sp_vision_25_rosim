#ifndef TOOLS__TIME_CONVERTER_HPP
#define TOOLS__TIME_CONVERTER_HPP

#include <chrono>
#include <cstdint>

// ============================================================================
// 时间戳转换工具 - 用于仿真和实车的时间基准统一
// ============================================================================
// 说明：
//   仿真环境中，所有时间戳来自 ROS2 消息头的 header.stamp（仿真时间）
//   需要转换为 std::chrono::steady_clock::time_point 格式，以兼容现有代码
//   关键：确保 Camera 和 CBoard 使用的时间基准一致
// ============================================================================

namespace tools
{

class TimeConverter
{
public:
  // 【新增 - 仿真模式】将 ROS2 时间戳转换为 steady_clock time_point
  // 输入：纳秒计数（从 rclcpp::Time 获取）
  // 输出：steady_clock::time_point
  // 注意：使用 int64_t 以支持 ROS2 时间戳（可能为负数，虽然通常不会）
  static std::chrono::steady_clock::time_point
  from_nanoseconds(int64_t nanoseconds)
  {
    // 直接将纳秒映射到 steady_clock 的内部表示
    // 注意：这不是真实物理时间，而是仿真虚拟时间
    // 只要整个系统（Camera、CBoard、IMU）都用这个转换，就能保持时间一致性
    return std::chrono::steady_clock::time_point(
      std::chrono::nanoseconds(nanoseconds));
  }

  // 【新增 - 仿真模式】从 rclcpp::Time 直接转换
  // 需要在包含 rclcpp 的地方调用
  // 这里只提供声明，实现在需要用到 rclcpp 的文件中
  // static std::chrono::steady_clock::time_point from_rclcpp_time(
  //   const rclcpp::Time & time);

  // 【原有】获取当前时间（真实模式下使用）
  static std::chrono::steady_clock::time_point now()
  {
    return std::chrono::steady_clock::now();
  }
};

}  // namespace tools

#endif  // TOOLS__TIME_CONVERTER_HPP
