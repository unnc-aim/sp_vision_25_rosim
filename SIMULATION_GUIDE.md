## 仿真环境集成指南 - 时间戳与 ROS2 话题对接

### 概述

本项目现已支持在 ROS2 Humble + Gazebo 仿真环境中运行。核心修改包括：

1. **时间戳转换** (`tools/time_converter.hpp`)
2. **仿真相机驱动** (`io/sim_camera.hpp/cpp`)
3. **仿真控制板** (`io/sim_board.hpp/cpp`)
4. **最小化的现有代码改动** (注释完好，保留原有逻辑)

### 关键概念：时间戳统一

#### 问题
- **实车模式**：使用 `std::chrono::steady_clock::now()` 获取当前物理时间
- **仿真模式**：使用 ROS2 仿真时间 (`/clock` 话题，ROS 消息头中的 `stamp`)

#### 解决方案
- 新建 `tools/TimeConverter::from_nanoseconds()` 方法
- 将 ROS 消息的纳秒时间戳直接映射到 `steady_clock::time_point`
- **关键**：确保图像 (Image)、IMU、和所有其他传感器使用同一套时间基准

---

### 新增文件说明

#### 1. `tools/time_converter.hpp`
**作用**：时间戳转换工具

```cpp
// 将 ROS 纳秒时间戳转换为 steady_clock::time_point
auto sim_time = tools::TimeConverter::from_nanoseconds(nanoseconds);

// 获取当前时间（真实模式使用）
auto now = tools::TimeConverter::now();
```

#### 2. `io/sim_camera.hpp/cpp`
**作用**：仿真图像获取

```cpp
// 订阅仿真图像话题，从 ROS 消息头获取时间戳
// 完全兼容原 Camera 接口
io::SimCamera sim_cam("config_path", "/red_standard_robot1/front_industrial_camera/image");
sim_cam.read(img, timestamp);  // 返回仿真图像和对应的仿真时间戳
```

**核心逻辑**：
- ROS 消息回调函数自动被触发
- 从消息头提取 `header.stamp`（仿真时间）
- 转换为 `steady_clock::time_point`
- 推入内部队列

#### 3. `io/sim_board.hpp/cpp`
**作用**：仿真控制板（云台姿态 + 控制）

```cpp
// 订阅仿真 IMU 话题，获取云台姿态四元数
io::SimBoard sim_board("config_path", "/red_standard_robot1/livox/imu");

// 按时间戳查询姿态（与实车逻辑完全相同，包括插值）
auto q = sim_board.imu_at(image_timestamp - 1ms);

// 发送控制指令（仿真模式下暂为日志输出）
sim_board.send(command);
```

**核心逻辑**：
- 订阅仿真 IMU 话题，获取高频四元数
- 支持与图像时间戳的插值对齐（与 CBoard 逻辑完全相同）

---

### 使用方式

#### 方式 A：最小改动 - 仅修改启动代码

在你的主程序（如 `src/sentry.cpp`）中，使用新的构造函数：

```cpp
// 【实车模式 - 原有代码】
io::Camera camera("configs/sentry.yaml");
io::CBoard cboard("configs/sentry.yaml");

// 【仿真模式 - 新增】只需传入仿真话题名称
io::Camera camera("configs/sentry.yaml", "/red_standard_robot1/front_industrial_camera/image");
io::CBoard cboard("configs/sentry.yaml", "/red_standard_robot1/livox/imu");

// 下面的逻辑完全不变！
while (!exiter.exit()) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    // ... 核心算法逻辑，完全相同 ...
}
```

#### 方式 B：通过配置文件切换（推荐）

创建一个新的配置文件 `configs/sim_sentry.yaml`：

```yaml
# 仿真模式配置
camera_name: "simulation"  # 特殊标记
image_topic: "/red_standard_robot1/front_industrial_camera/image"
imu_topic: "/red_standard_robot1/livox/imu"

# 其他配置与原 sentry.yaml 相同
# ...
```

然后在代码中：

```cpp
std::string image_topic;
std::string imu_topic;

auto yaml = tools::load(config_path);
if (tools::read<std::string>(yaml, "camera_name") == "simulation") {
    image_topic = tools::read<std::string>(yaml, "image_topic");
    imu_topic = tools::read<std::string>(yaml, "imu_topic");
}

io::Camera camera(config_path, image_topic);
io::CBoard cboard(config_path, imu_topic);
```

---

### 关键架构设计

```
原有代码结构（实车）：
Camera (factory)
  ├─ MindVision -> capture_thread -> steady_clock::now()
  ├─ HikRobot -> capture_thread -> steady_clock::now()
  └─ USBCamera -> capture_thread -> steady_clock::now()

新增代码结构（仿真）：
Camera (factory)
  └─ SimCamera -> ROS callback -> TimeConverter::from_nanoseconds()

CBoard (factory)
  └─ SimBoard -> ROS callback -> TimeConverter::from_nanoseconds()
```

**优势**：
- 上层业务逻辑（Tasks、Src）完全不需改动
- 只修改 IO 层的实现细节
- 支持同一份二进制切换实车/仿真（通过配置）

---

### 时间戳流程图

```
仿真场景：
┌─────────────────────────────────────────────┐
│ Gazebo 仿真世界 (/clock)                    │
└──────────┬──────────────────────────────────┘
           │ 仿真时间
           ├─────────────────┬─────────────────┐
           │                 │                 │
      [图像话题]        [IMU话题]         [其他话题]
      image_raw      sensor_msgs/Imu       ...
      header.stamp   header.stamp
           │                 │
           ▼                 ▼
      SimCamera        SimBoard
      (callback)       (callback)
           │                 │
      TimeConverter::   TimeConverter::
      from_nano...()    from_nano...()
           │                 │
      steady_clock::   steady_clock::
      time_point       time_point
           │                 │
      queue_.push()     queue_.push()
           │                 │
    ┌──────▼──────────────────▼───────┐
    │ 核心算法（不知道时间来源）      │
    │ - Kalman 滤波                   │
    │ - 目标跟踪                       │
    │ - 轨迹预测                       │
    │ - 火力控制                       │
    └────────────────────────────────┘
```

---

### 仿真话题映射表

**图像**：
- Gazebo 发布：`/red_standard_robot1/front_industrial_camera/image`
- 类型：`sensor_msgs/msg/Image`
- 时间戳：`header.stamp`
- 映射到：`io::SimCamera`

**IMU/姿态**：
- Gazebo 发布：`/red_standard_robot1/livox/imu`
- 类型：`sensor_msgs/msg/Imu`
- 时间戳：`header.stamp`
- 四元数：`orientation`
- 映射到：`io::SimBoard`

**控制输出**（暂未实现，可扩展）：
- 目标发送：`/red_standard_robot1/cmd_gimbal_joint` (云台角度)
- 目标发送：`/red_standard_robot1/cmd_shoot` (射击控制)

---

### 编译和运行

#### 编译
```bash
cd ~/ros2_ws
colcon build --packages-select sp_vision_25

# 或指定仿真模式编译
colcon build --packages-select sp_vision_25 --cmake-args -DENABLE_SIMULATION=ON
```

#### 运行（仿真模式）
```bash
# 启动 Gazebo 仿真环境
ros2 launch rm_simulation sim.launch.py

# 在另一个终端运行视觉程序（使用仿真配置）
ros2 run sp_vision_25 sentry_sim
# 或
./build/sp_vision_25/src/sentry configs/sim_sentry.yaml
```

---

### 调试和验证

#### 1. 检查话题是否发布
```bash
ros2 topic list | grep -E "image|imu|gimbal"
```

#### 2. 查看消息内容
```bash
ros2 topic echo /red_standard_robot1/front_industrial_camera/image
ros2 topic echo /red_standard_robot1/livox/imu
```

#### 3. 启用日志输出
```cpp
// 在 sim_camera.cpp / sim_board.cpp 中
tools::logger()->debug("...");  // 已在代码中添加
```

设置日志级别：
```bash
ros2 run sp_vision_25 sentry --log-level DEBUG
```

---

### 注意事项

1. **时间戳精度**：
   - 确保仿真世界和视觉程序的时间同步（通过 ROS2 Clock）
   - 仿真时间戳精度应至少为毫秒级

2. **消息格式**：
   - 图像格式应为标准 `sensor_msgs/Image`（自动转为 BGR 格式）
   - IMU 四元数应为标准 `geometry_msgs/Quaternion` (x, y, z, w 顺序)

3. **性能**：
   - ROS2 回调线程与算法主线程异步运行
   - 确保队列大小足够（默认为 10 帧图像，5000 帧 IMU）

4. **向后兼容性**：
   - 所有原有代码（实车驱动、CAN 通信）保持不变
   - 可同时编译支持实车和仿真

---

### 常见问题 (FAQ)

**Q: 能否同时支持实车和仿真？**  
A: 可以。通过配置文件或启动参数选择。二进制程序通过 `image_topic` 是否为空来判断模式。

**Q: 时间戳如何与 Kalman 滤波器同步？**  
A: `steady_clock::time_point` 是仅有的时间参考，Kalman 滤波器的时间间隔计算基于此。

**Q: 仿真中云台如何发送命令？**  
A: 当前版本中 `send()` 在仿真模式下只输出日志。可扩展为发布到 `cmd_gimbal_joint` 话题。

**Q: 如果 ROS 消息晚到怎么办？**  
A: 队列会阻塞等待。如果队列满，会覆盖最旧的数据（`ThreadSafeQueue` 行为）。

---

### 扩展方向

1. **发送控制指令到仿真**：
   - `SimBoard::send()` 发布到 `cmd_gimbal_joint`
   - 实现云台/射击反馈

2. **集成其他仿真话题**：
   - 敌方位置信息 (tf 坐标系)
   - 裁判系统状态 (referee_system/*)
   - 底盘反馈 (chassis_odometry_gt)

3. **性能优化**：
   - 预处理（去畸变、色彩矫正）
   - 多线程图像处理

---

### 文件清单

**新增文件**：
- `tools/time_converter.hpp` - 时间戳转换工具
- `io/sim_camera.hpp` / `io/sim_camera.cpp` - 仿真相机驱动
- `io/sim_board.hpp` / `io/sim_board.cpp` - 仿真控制板
- `configs/sim_sentry.yaml` (可选) - 仿真配置示例

**修改文件**：
- `io/camera.hpp` - 添加第二个构造函数
- `io/camera.cpp` - 添加仿真模式实现
- `io/cboard.hpp` - 添加第二个构造函数和仿真指针
- `io/cboard.cpp` - 添加仿真模式实现
- `io/CMakeLists.txt` - 添加新源文件和依赖

**保留原有**：
- 所有 src/* 文件 - 无需改动
- 所有 tasks/* 文件 - 无需改动
- 所有硬件驱动 - 原有逻辑完全保留（注释标记清晰）
