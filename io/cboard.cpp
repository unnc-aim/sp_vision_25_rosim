#include "cboard.hpp"

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#if defined(SP_VISION_HAS_ROS2) && (SP_VISION_HAS_ROS2 == 1)
#include "io/sim_board.hpp"  // 【新增 - 仿真支持】
#endif

namespace io
{

// 【原有构造函数】用于实车 CAN 接口
CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000),
  is_simulation_(false)
// 注意: callback的运行会早于Cboard构造函数的完成
{
  // 【修改 - 使用指针】创建 SocketCAN 对象
  can_ = std::make_unique<SocketCAN>(
    read_yaml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1));

  tools::logger()->info("[Cboard] Waiting for q...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[Cboard] Opened.");
}

// 【新增构造函数】用于仿真模式
// 当传入 imu_topic 非空时，使用仿真板卡（SimBoard）
// imu_topic 示例: "/red_standard_robot1/livox/imu"
CBoard::CBoard(const std::string & config_path, const std::string & imu_topic)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000),
  is_simulation_(true)
{
#if !defined(SP_VISION_HAS_ROS2) || (SP_VISION_HAS_ROS2 != 1)
  throw std::runtime_error(
    "[CBoard] Simulation mode requires ROS2 (rclcpp/sensor_msgs). "
    "Please install ROS2 dependencies and source the ROS2 environment, or use the CAN-mode constructor.");
#else
  if (imu_topic.empty()) {
    // 如果话题为空，回退到原有的 CAN 驱动逻辑
    // 使用原始初始化器列表重新初始化 can_（但这可能不太好）
    // 更好的方案：通过配置文件判断
    throw std::runtime_error(
      "[CBoard] Simulation mode requires non-empty imu_topic. "
      "Or use the original constructor for CAN mode.");
  }

  // 【新增 - 仿真模式】创建 SimBoard 对象
  sim_board_ = new SimBoard(config_path, imu_topic);

  // 【重要】从 SimBoard 同步弹速和自瞄状态到 CBoard 公开成员
  // 这样上层代码（如 sim_sentry.cpp）可以直接使用 cboard.bullet_speed
  bullet_speed = sim_board_->bullet_speed;

  // mode 在仿真模式下可以设为 auto_aim（如果 auto_aim_enabled）
  // 注意：auto_aim_enabled 是 atomic<bool>，需要使用 .load() 读取
  if (sim_board_->auto_aim_enabled.load()) {
    mode = Mode::auto_aim;
  }

  tools::logger()->info("[CBoard] Simulation mode enabled with IMU topic: {}", imu_topic);
  tools::logger()->info("[CBoard] Synced bullet_speed={:.2f} m/s, mode={}", 
    bullet_speed, MODES[mode]);
#endif
}

// 【新增 - 析构函数】清理仿真模式下的资源
CBoard::~CBoard()
{
#if defined(SP_VISION_HAS_ROS2) && (SP_VISION_HAS_ROS2 == 1)
  if (sim_board_ != nullptr) {
    delete sim_board_;
    sim_board_ = nullptr;
  }
#else
  // 无 ROS2 构建下不会创建 sim_board_（仿真构造函数会直接抛异常）
  sim_board_ = nullptr;
#endif
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  // 【新增 - 仿真模式分支】
#if defined(SP_VISION_HAS_ROS2) && (SP_VISION_HAS_ROS2 == 1)
  if (is_simulation_ && sim_board_ != nullptr) {
    return sim_board_->imu_at(timestamp);
  }
#endif

  // 【原有 - 实车模式】
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command) const
{
  // 【新增 - 仿真模式分支】
#if defined(SP_VISION_HAS_ROS2) && (SP_VISION_HAS_ROS2 == 1)
  if (is_simulation_ && sim_board_ != nullptr) {
    sim_board_->send(command);
    return;
  }
#endif

  // 【原有 - 实车模式】
  can_frame frame;
  frame.can_id = send_canid_;
  frame.can_dlc = 8;
  frame.data[0] = (command.control) ? 1 : 0;
  frame.data[1] = (command.shoot) ? 1 : 0;
  frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;
  frame.data[3] = (int16_t)(command.yaw * 1e4);
  frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;
  frame.data[5] = (int16_t)(command.pitch * 1e4);
  frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;
  frame.data[7] = (int16_t)(command.horizon_distance * 1e4);

  try {
    can_->write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

// 【新增 - 仿真支持】查询四元数格式
bool CBoard::is_direct_gimbal2world() const
{
#if defined(SP_VISION_HAS_ROS2) && (SP_VISION_HAS_ROS2 == 1)
  if (is_simulation_ && sim_board_ != nullptr) {
    return sim_board_->is_direct_gimbal2world();
  }
#endif
  // 实车模式：四元数需要通过 R_gimbal2imubody 变换
  return false;
}

void CBoard::callback(const can_frame & frame)
{
  auto timestamp = std::chrono::steady_clock::now();

  if (frame.can_id == quaternion_canid_) {
    auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({{w, x, y, z}, timestamp});
  }

  else if (frame.can_id == bullet_speed_canid_) {
    bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    mode = Mode(frame.data[2]);
    shoot_mode = ShootMode(frame.data[3]);
    ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }

  return yaml["can_interface"].as<std::string>();
}

}  // namespace io