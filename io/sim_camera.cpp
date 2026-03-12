#include "io/sim_camera.hpp"

#include <thread>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{

// ============================================================================
// 构造函数 - 初始化仿真相机
// ============================================================================
// 与硬件相机（MindVision/HikRobot）的初始化逻辑对应：
//   硬件相机：打开设备 -> 启动取图线程 -> 等待首帧
//   仿真相机：创建节点 -> 订阅话题 -> 启动 spin 线程 -> 等待首帧
// ============================================================================
SimCamera::SimCamera(const std::string & config_path, const std::string & image_topic)
: queue_(1)  // 队列大小为 1，只保留最新帧（与硬件相机行为一致）
{
  // 【步骤 1】检查 ROS2 是否已初始化
  // 注意：ROS2 初始化由 io::ROS2 统一管理，这里只检查状态
  if (!rclcpp::ok()) {
    // 如果 ROS2 未初始化，则由本类负责初始化
    // 这种情况发生在不使用 io::ROS2 而直接使用 SimCamera 时
    rclcpp::init(0, nullptr);
    tools::logger()->info("[SimCamera] ROS2 initialized by SimCamera");
  }

  // 【步骤 2】创建 ROS2 节点
  // 节点名称添加时间戳后缀，避免多实例冲突
  auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  std::string node_name = "sim_camera_" + std::to_string(now % 10000);
  node_ = rclcpp::Node::make_shared(node_name);

  // 【步骤 3】创建图像订阅器
  // QoS 设置：
  //   - 队列深度 10：允许短暂的处理延迟
  //   - 默认可靠性：确保图像不丢失
  image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
    image_topic,
    10,
    std::bind(&SimCamera::image_callback, this, std::placeholders::_1));

  // 【步骤 4】创建 executor 并启动 spin 线程
  // 使用独立的 executor，避免与其他节点的 spin 冲突
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  spin_thread_ = std::thread([this]() {
    tools::logger()->debug("[SimCamera] Spin thread started");
    while (running_ && rclcpp::ok()) {
      executor_->spin_some(10ms);
    }
    tools::logger()->debug("[SimCamera] Spin thread stopped");
  });

  tools::logger()->info("[SimCamera] Subscribed to topic: {}", image_topic);
  tools::logger()->info("[SimCamera] Waiting for first image frame...");
}

// ============================================================================
// 析构函数 - 清理资源
// ============================================================================
SimCamera::~SimCamera()
{
  // 【步骤 1】停止 spin 线程
  running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  // 【步骤 2】清理 executor 和节点
  if (executor_) {
    executor_->remove_node(node_);
  }
  node_.reset();
  executor_.reset();

  // 注意：不在这里调用 rclcpp::shutdown()
  // ROS2 的生命周期由 io::ROS2 或主程序统一管理

  tools::logger()->info("[SimCamera] Destructed");
}

// ============================================================================
// read() - 读取图像和时间戳
// ============================================================================
// 接口实现：与 MindVision/HikRobot 的 read() 完全相同
// 使用带超时的 try_pop，每 100ms 检查一次 rclcpp::ok()
// 确保在 Ctrl+C 时能够及时退出
// ============================================================================
void SimCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  // 使用带超时的 try_pop，每 100ms 检查一次 rclcpp::ok()
  // 这样在 Ctrl+C 时能够及时退出
  while (running_ && rclcpp::ok()) {
    if (queue_.try_pop(data, std::chrono::milliseconds(100))) {
      img = data.img;
      timestamp = data.timestamp;
      return;
    }
  }
  // 如果退出循环但没有数据，返回空图像
  img = cv::Mat();
  timestamp = std::chrono::steady_clock::now();
}

// ============================================================================
// image_callback() - ROS2 图像回调函数
// ============================================================================
// 处理流程：
//   1. 从消息头提取仿真时间戳
//   2. 将 ROS 图像消息转换为 OpenCV Mat
//   3. 处理图像格式（RGB -> BGR）
//   4. 推入队列
// ============================================================================
void SimCamera::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // 【步骤 1】提取仿真时间戳
  // ROS2 时间戳格式：sec（秒）+ nanosec（纳秒）
  // 转换为统一的 steady_clock::time_point 格式
  // 注意：使用 int64_t 与 SimBoard 保持一致（ROS2 时间戳可能为负数）
  int64_t nanoseconds =
    static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(msg->header.stamp.nanosec);
  auto timestamp = tools::TimeConverter::from_nanoseconds(nanoseconds);

  // 【步骤 2】转换图像格式
  // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV Mat
  cv::Mat img;
  try {
    // 尝试直接转换为 BGR8 格式（与真实相机输出一致）
    // cv_bridge 会自动处理 RGB -> BGR 的转换
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    img = cv_ptr->image;
  } catch (const cv_bridge::Exception & e) {
    // 如果转换失败，尝试使用原始编码
    // 常见原因：图像编码不是 RGB/BGR 格式
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg);
      img = cv_ptr->image;

      // 如果是单通道图像，转换为 BGR
      if (img.channels() == 1) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
      }
      // 如果是 RGBA，转换为 BGR
      else if (img.channels() == 4) {
        cv::cvtColor(img, img, cv::COLOR_RGBA2BGR);
      }

      // 首次转换时输出警告日志
      static bool warned = false;
      if (!warned) {
        tools::logger()->warn(
          "[SimCamera] Image encoding '{}' converted to BGR", msg->encoding);
        warned = true;
      }
    } catch (const cv_bridge::Exception & e2) {
      tools::logger()->error("[SimCamera] cv_bridge exception: {}", e2.what());
      return;
    }
  }

  // 【步骤 3】验证图像有效性
  if (img.empty()) {
    tools::logger()->warn("[SimCamera] Received empty image frame");
    return;
  }

  // 【步骤 4】推入队列
  // 与硬件相机的逻辑一致：将 {图像, 时间戳} 打包推入队列
  queue_.push({img, timestamp});

  // 【调试日志】限制输出频率，每秒最多 1 次
  static auto last_log_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
    tools::logger()->debug(
      "[SimCamera] Image received: {}x{}, encoding: {}",
      msg->width, msg->height, msg->encoding);
    last_log_time = now;
  }
}

}  // namespace io
