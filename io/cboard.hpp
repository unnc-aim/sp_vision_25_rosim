#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有

  CBoard(const std::string & config_path);

  // 【新增 - 仿真支持】带仿真话题的构造函数
  // imu_topic: 仿真 IMU 话题，如 "/red_standard_robot1/livox/imu"
  CBoard(const std::string & config_path, const std::string & imu_topic);

  // 【新增 - 析构函数】清理仿真模式下的 SimBoard 对象
  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

  // 【新增 - 仿真支持】查询 imu_at() 返回的四元数是否是直接的 gimbal2world 格式
  // 如果是（使用 gimbal_state 话题），solver 不需要做额外的坐标系转换
  bool is_direct_gimbal2world() const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能

  // 【修改 - 仿真支持】使用指针以支持延迟初始化（仿真模式下不创建 CAN）
  std::unique_ptr<SocketCAN> can_;

  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;

  // 【新增 - 仿真支持】指向 SimBoard 对象的指针（仅在仿真模式下使用）
  class SimBoard * sim_board_ = nullptr;
  bool is_simulation_ = false;

  void callback(const can_frame & frame);

  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP