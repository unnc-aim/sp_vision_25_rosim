# 快速开始指南 - 仿真环境时间戳集成

## ✅ 已完成的工作

### 1. 新建文件（3个）
- ✅ `tools/time_converter.hpp` - 时间戳转换工具
- ✅ `io/sim_camera.hpp/cpp` - 仿真相机驱动（订阅 ROS 图像话题）
- ✅ `io/sim_board.hpp/cpp` - 仿真控制板（订阅 ROS IMU 话题）
- ✅ `src/sim_sentry.cpp` - 仿真版本的 sentry 示例程序
- ✅ `SIMULATION_GUIDE.md` - 完整集成指南
- ✅ `configs/sim_sentry.yaml` - 仿真配置示例

### 2. 修改现有文件（最小化改动）
- ✅ `io/camera.hpp` - 添加仿真构造函数（1 个新方法）
- ✅ `io/camera.cpp` - 添加仿真模式分支（30 行，原代码注释保留）
- ✅ `io/cboard.hpp` - 添加仿真指针和构造函数（3 行新成员）
- ✅ `io/cboard.cpp` - 添加仿真模式分支（60 行，原代码注释保留）
- ✅ `io/CMakeLists.txt` - 添加新源文件和 ROS2 依赖

### 3. 所有原有代码保持不变
- ✅ `src/sentry.cpp` - 无需修改
- ✅ `src/standard.cpp` - 无需修改
- ✅ `tasks/` - 所有算法无需修改
- ✅ `io/mindvision/` - 实车驱动保留
- ✅ `io/hikrobot/` - 实车驱动保留
- ✅ `io/cboard.cpp` 的原有逻辑 - 用条件判断保留

---

## 🚀 快速使用

### 仅修改启动代码（推荐方式 A）

```cpp
// 在主程序中，将原来的：
io::Camera camera("configs/sentry.yaml");
io::CBoard cboard("configs/sentry.yaml");

// 改为：
io::Camera camera("configs/sentry.yaml", 
                  "/red_standard_robot1/front_industrial_camera/image");
io::CBoard cboard("configs/sentry.yaml", 
                  "/red_standard_robot1/livox/imu");

// 下面的代码完全不变
camera.read(img, timestamp);
auto q = cboard.imu_at(timestamp - 1ms);
// ...
```

### 使用配置文件（推荐方式 B）

```bash
# 启动仿真程序
ros2 run sp_vision_25 sim_sentry configs/sim_sentry.yaml
```

---

## 📊 时间戳流程总结

### 关键转换点

```
Gazebo 仿真世界
  ↓
ROS2 消息 (sensor_msgs/Image, sensor_msgs/Imu)
  ├─ header.stamp: {sec, nanosec}  ← 仿真时间
  ↓
SimCamera / SimBoard (ROS 回调)
  ↓
TimeConverter::from_nanoseconds()  ← 时间戳转换
  ↓
std::chrono::steady_clock::time_point  ← 内部统一时间格式
  ↓
queue_.push({data, timestamp})  ← 推入队列
  ↓
camera.read() / cboard.imu_at()  ← 上层业务无感
```

### 核心原则

1. **统一时间基准**：所有传感器（图像、IMU）使用相同的时间戳格式
2. **最小化改动**：原有硬件驱动代码保持不变，通过多态分发
3. **向后兼容**：既支持实车（CAN），也支持仿真（ROS2）

---

## ✨ 架构设计亮点

### 1. 多态工厂模式
```
CameraBase (抽象)
├── MindVision   (实车驱动 1)
├── HikRobot     (实车驱动 2)
├── USBCamera    (实车驱动 3)
└── SimCamera    (仿真驱动)  ← 新增，完全兼容接口

Camera (工厂)
└── 根据参数选择具体实现 ← 上层无感
```

### 2. 时间转换透明化
- 仿真：ROS 时间 → `steady_clock::time_point` → 算法逻辑
- 实车：`steady_clock::now()` → `steady_clock::time_point` → 算法逻辑
- **效果**：算法层完全不知道时间来源

### 3. 编译级别兼容性
- 所有 ROS2 依赖都是 QUIET 型（可选）
- 仿真文件不需要硬件库（MindVision、HikRobot）
- 实车文件不需要 cv_bridge、sensor_msgs

---

## 🔧 编译命令

### 完整编译（包含仿真和实车）
```bash
cd ~/ros2_ws
colcon build --packages-select sp_vision_25
```

### 仅编译仿真版本
```bash
colcon build --packages-select sp_vision_25 \
  --cmake-args -DWITH_SIMULATION=ON
```

---

## 🧪 验证检查清单

### 编译验证
- [ ] `colcon build` 成功，无错误
- [ ] `SimCamera` 和 `SimBoard` 库编译成功
- [ ] 原有 `sentry.cpp` 仍可正常编译

### 运行验证
- [ ] Gazebo 仿真环境启动
  ```bash
  ros2 launch rm_simulation sim.launch.py
  ```

- [ ] 检查话题是否发布
  ```bash
  ros2 topic list | grep -E "image|imu"
  ```

- [ ] 启动视觉程序（仿真模式）
  ```bash
  ros2 run sp_vision_25 sim_sentry configs/sim_sentry.yaml
  ```

- [ ] 观察日志输出
  ```
  [SimCamera] Subscribed to topic: /red_standard_robot1/front_industrial_camera/image
  [SimBoard] Subscribed to IMU topic: /red_standard_robot1/livox/imu
  [SimSentry] Starting main loop...
  ```

---

## 📝 关键文件对照表

| 功能 | 实车文件 | 仿真文件 | 说明 |
|------|--------|--------|------|
| 相机驱动 | `io/mindvision.cpp` | `io/sim_camera.cpp` | 多态继承 CameraBase |
| 控制板 | `io/cboard.cpp` | `io/sim_board.cpp` | CBoard 工厂模式 |
| 时间戳 | `steady_clock::now()` | `TimeConverter::from_nanoseconds()` | 统一格式 |
| 消息源 | 硬件（USB、CAN） | ROS2 话题 | 底层差异 |
| 主程序 | `src/sentry.cpp` | `src/sim_sentry.cpp` | 仅初始化不同 |

---

## 🎯 下一步集成步骤

### 第 1 阶段（当前）✅
- [x] 时间戳统一
- [x] 图像和 IMU 数据接收

### 第 2 阶段（可选）
- [ ] 控制指令发送到仿真
  - 云台关节指令 → `/cmd_gimbal_joint`
  - 射击信号 → `/cmd_shoot`

### 第 3 阶段（可选）
- [ ] 集成其他仿真话题
  - 敌方位置 (TF 坐标系)
  - 裁判系统 (referee_system/*)
  - 底盘反馈 (chassis_odometry_gt)

### 第 4 阶段（可选）
- [ ] 性能优化
  - 图像预处理 (去畸变、色彩矫正)
  - 多线程推理

---

## 💡 故障排查

### 编译错误

**问题**：`error: cv_bridge not found`
```bash
# 解决
sudo apt-get install ros-humble-cv-bridge
colcon build --packages-select sp_vision_25
```

**问题**：`error: sensor_msgs not found`
```bash
# 解决
sudo apt-get install ros-humble-sensor-msgs
colcon build --packages-select sp_vision_25
```

### 运行错误

**问题**：`[SimCamera] Subscribed to topic: ... (timeout)`
```
原因：Gazebo 未启动或话题名不匹配
检查：ros2 topic list | grep image
```

**问题**：`[SimBoard] Waiting for IMU data... (hang)`
```
原因：IMU 数据未发布或延迟高
调试：ros2 topic hz /red_standard_robot1/livox/imu
```

---

## 📞 联系与反馈

- 问题：仿真话题映射、时间戳对齐、性能优化
- 参考文件：`SIMULATION_GUIDE.md`（详细文档）
- 示例代码：`src/sim_sentry.cpp`（完整示例）

---

**最后提醒**：所有改动都标记了【新增 - 仿真支持】或【原有 - 实车模式】，方便审查和回滚。❤️
