# px4ctrl 迁移与接口使用指南

## 1. 控制边界

`px4ctrl` 是 ROS 2 外部位置控制器。规划器发布期望位置、速度、加速度和航向；
控制器使用 PX4 本地位置/速度反馈，计算位置 P、速度 PI 和加速度前馈，输出
`OffboardControlMode(attitude=true)` 与 `VehicleAttitudeSetpoint`。PX4 继续执行
姿态环、角速度环和电机分配；PX4 原生位置环不处理这段轨迹。控制器不发布
`VehicleRatesSetpoint`，也不提供三轴角速度设定值，只有 yaw 角速度前馈。

控制器包只负责控制与安全检查。定位源、规划器、PX4/Gazebo、Micro XRCE-DDS
Agent，以及起飞/解锁/切换 Offboard 的流程必须由目标项目按下文配置提供。
`eight` 是本工作区的仿真试验包，不是 `px4ctrl` 的编译或运行依赖。

## 2. 移植依赖

将整个 `src/px4ctrl` 复制到目标 ROS 2 工作空间的 `src` 下，并提供与目标 PX4
固件 uORB/DDS 接口版本匹配的 `px4_msgs`。构建依赖见 `package.xml` 和
`CMakeLists.txt`，核心包括 `rclcpp`、`px4_msgs`、`geometry_msgs`、
`nav_msgs`、`std_msgs`、Eigen3 和 ROS 2 消息生成工具。独立运行控制器不需要
EGO、FAST-LIO 或 `eight`；EGO 适配节点需要另行安装 `quadrotor_msgs`。

```bash
source /opt/ros/jazzy/setup.bash  # 替换为目标项目使用的 ROS 2 发行版
cd /path/to/target_ws
colcon build --packages-up-to px4ctrl
source install/setup.bash
MicroXRCEAgent udp4 -p 8888  # 独立终端运行；端口须与 PX4 的 DDS 客户端一致
```

先核对目标 PX4 固件的 `dds_topics.yaml` 和 ROS 2 侧 `ros2 topic list -t`。
本仓库使用带版本后缀的 `/fmu/out/vehicle_local_position_v1`、
`/fmu/out/vehicle_status_v4` 和 `/fmu/in/vehicle_attitude_setpoint_v1`；
其他固件/消息版本可能使用不同名称，必须同步修改配置文件中的话题参数。

## 3. 规划器输入

规划器向 `desired_state_topic`（默认 `/px4ctrl/desired_state`）持续发布
`px4ctrl/msg/DesiredState`：

```text
std_msgs/Header header                 # stamp 与 frame_id
geometry_msgs/Point position           # m
geometry_msgs/Vector3 velocity         # m/s
geometry_msgs/Vector3 acceleration     # m/s^2
float64 yaw                             # rad
float64 yaw_rate                        # rad/s
```

`position`、`velocity`、`acceleration` 必须在同一个世界坐标系中；`yaw` 是该
世界坐标系水平面的航向角，`yaw_rate` 是对应航向角的变化率。使用 X500 示例配置
`planner_frame: px4_local_ned` 时，X 指北、Y 指东、Z 指下，因此上升 2 m 对应
`position.z` 减少 2；航向也按 NED 水平面定义。若规划器使用其他坐标系，设置
`planner_frame`、`transform_rotation_row_major` 和必要的原点对齐参数，不能只
改 `frame_id` 而不转换数值。

`header.frame_id` 必须与 `planner_frame` 完全相同。`header.stamp` 使用控制器
ROS 时钟，仿真中须启用 `use_sim_time` 并接收 `/clock`；时间戳必须大于 0、
严格递增，接收时不能早于 `command_timeout_sec`（X500 配置为 0.5 s），也
不能超前超过 0.05 s。所有数值必须有限，不能用 NaN 表示未指定的速度或
加速度；没有前馈时显式填 0。建议规划器连续发布，频率显著高于超时阈值
对应的 2 Hz，例如 20–50 Hz。指令超时后，控制器会捕获当前位置进入悬停。
大的位置/速度指令跳变由 `command_blend_sec` 平滑过渡，不等同于轨迹规划。

例如使用 PX4 本地 NED 作为规划坐标系的 Python 发布片段：

```python
from px4ctrl.msg import DesiredState

publisher = node.create_publisher(DesiredState, "/px4ctrl/desired_state", 20)
command = DesiredState()
command.header.stamp = node.get_clock().now().to_msg()
command.header.frame_id = "px4_local_ned"
command.position.x = target_x_north_m
command.position.y = target_y_east_m
command.position.z = target_z_down_m
command.velocity.x = vx_north_mps
command.velocity.y = vy_east_mps
command.velocity.z = vz_down_mps
command.acceleration.x = ax_north_mps2
command.acceleration.y = ay_east_mps2
command.acceleration.z = az_down_mps2
command.yaw = yaw_ned_rad
command.yaw_rate = yaw_rate_radps
publisher.publish(command)
```

上述片段应放在持续运行的定时回调中，变量由规划器计算。`DesiredState` 是
本包自定义消息；直接向 PX4 发布 `TrajectorySetpoint` 不会经过本控制器。

## 4. PX4 反馈与飞控输出

| 方向 | 默认话题 | 消息 | 用途 |
| --- | --- | --- | --- |
| PX4 → 控制器 | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` | 位置、速度、航向与有效性，位置/速度闭环的主要反馈 |
| PX4 → 控制器 | `/fmu/out/vehicle_status_v4` | `px4_msgs/msg/VehicleStatus` | 解锁、Offboard、预检和 failsafe 状态 |
| PX4 → 控制器 | `/fmu/out/vehicle_land_detected` | `px4_msgs/msg/VehicleLandDetected` | 空中接管时确认已离地；稳定状态约 1 Hz |
| PX4 → 控制器 | `/fmu/out/vehicle_attitude` | `px4_msgs/msg/VehicleAttitude` | 更新 PX4 微秒时间戳；姿态闭环仍由 PX4 执行 |
| 控制器 → PX4 | `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | `attitude=true`，告知 PX4 使用外部姿态目标并维持 Offboard 心跳 |
| 控制器 → PX4 | `/fmu/in/vehicle_attitude_setpoint_v1` | `px4_msgs/msg/VehicleAttitudeSetpoint` | `q_d` 期望姿态四元数、`thrust_body=[0,0,-T]`、`yaw_sp_move_rate` |
| 控制器 → PX4 | `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | 仅在配置允许的自动切模式/解锁路径或安全故障降落时发送 |

`T` 是归一化推力，受 `min_thrust`/`max_thrust` 限制，且高度控制依赖
`hover_thrust` 的机型标定。`yaw_sp_move_rate` 是 yaw 前馈，不能当作完整三轴
角速度目标。控制器在 X500 示例配置中以 100 Hz 发布姿态目标；PX4 的
`OffboardControlMode` 流不能中断，否则 PX4 会退出 Offboard。控制器必须拿到
有效、及时的 PX4 本地位置及状态后才会发布有效姿态目标。

## 5. 其他 ROS 接口

| 方向 | 默认话题 | 消息 | 作用 |
| --- | --- | --- | --- |
| 规划器 → 控制器 | `/px4ctrl/desired_state` | `px4ctrl/msg/DesiredState` | 轨迹或定点目标 |
| 可选定位 → 控制器 | `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | 仅在 `align_to_px4_local_origin: true` 时用于初始坐标对齐；之后反馈仍使用 PX4 本地位置 |
| 上层 → 控制器 | `/planning/emergency_stop` | `std_msgs/msg/Empty` | 捕获当前位置悬停；定位无效时锁定安全故障 |
| 上层 → 控制器 | `/px4ctrl/manual_yaw` | `std_msgs/msg/Float64` | 临时覆盖跟踪阶段的目标 yaw，单位 rad |
| 上层 → 控制器 | `/px4ctrl/clear_manual_yaw` | `std_msgs/msg/Empty` | 清除 yaw 覆盖 |
| 起飞流程 → 控制器 | `/eight/takeover_allowed` | `std_msgs/msg/Bool` | 仅 `require_takeover_permission: true` 时作为空中接管许可；话题名可改 |
| 控制器 → 观察者 | `/px4ctrl/status` | `std_msgs/msg/String` | 人可读状态，例如 `hover_airborne_takeover_offboard_ready`、`tracking_attitude_thrust_tracking`、`control_suspended_*` |
| 控制器 → 观察者 | `/px4ctrl/debug` | `px4ctrl/msg/ControlDebug` | 误差、参考速度、期望/可行加速度、推力、倾角、状态与饱和标记 |

`ControlDebug` 中的向量均为 PX4 本地 NED；`saturation_flags` 的 bit 0/1/2/3
分别表示总加速度、垂直力、水平倾角/力、最终推力裁剪。详情见
`msg/ControlDebug.msg`。接管前不一定有调试消息；调试话题不替代 PX4 状态检查。

## 6. 两种接入模式

### A. 已起飞后接管（适合独立迁移）

从 `config/px4ctrl_x500.yaml` 复制一份项目专用参数。至少核对机型推力、
话题名、坐标系和如下项目：

```yaml
entry_mode: airborne_takeover
planner_frame: px4_local_ned
align_to_px4_local_origin: false
start_offboard_on_first_command: false
arm_on_first_command: false
require_takeover_permission: false  # 不使用 eight 时关闭其专用许可
```

独立启动：`ros2 run px4ctrl px4ctrl_node --ros-args --params-file /path/to/your.yaml`。
如采用本仓库 `eight` 的位置 Offboard 起飞流程，可保留
`require_takeover_permission: true`，由起飞节点在到达目标后发布许可。

独立接管时，先由目标项目完成起飞，并确保 PX4 已解锁、已离地、本地定位有效、
速度低于 `takeover_velocity_tolerance_mps`，且无 failsafe。控制器捕获当前
NED 位置和航向，持续发布姿态目标；等待 `/px4ctrl/status` 出现
`hover_airborne_takeover_ready`（默认预流 1 s）后，再由外部模式管理器请求
Offboard。状态变为 `hover_airborne_takeover_offboard_ready` 后开始发布
`DesiredState`。`airborne_takeover` 模式本身不会发起飞、解锁或切模式命令。
若目标项目先用位置 Offboard 起飞，应由模式管理器保证位置设定值与姿态设定值
按许可/状态完成交接，避免两个节点同时竞争 Offboard 控制模式。

### B. 首条规划指令触发（本仓库 EGO 流程）

`config/px4ctrl_sitl.yaml` 使用 `entry_mode: auto_takeoff`、Lidar 规划坐标系、
FAST-LIO 初始原点对齐，并可在收到首条指令后自行请求 Offboard 和解锁。
`bringup.launch.py` 还会启动 EGO、里程计桥接和
`ego_position_command_adapter`。该适配节点将
`/drone_0_planning/pos_cmd` 的 `quadrotor_msgs/msg/PositionCommand` 转为
`DesiredState`；若目标项目的规划器能直接发布 `DesiredState`，无需适配节点。
`px4ctrl_sitl.yaml` 的 `hover_thrust: 0.864` 针对带云台/MID-360 的机型，
不能直接用于裸 X500 或其他机型。

## 7. 迁移检查

1. 确认 PX4 固件与 `px4_msgs` 版本一致，ROS 2 DDS Agent 与目标 PX4 连接，PX4 反馈话题持续更新。
2. 确认 `planner_frame` 与指令 `header.frame_id` 一致，NED 转换与 `/clock` 时间域正确。
3. 校准 `hover_thrust`，并复核 `kp`、`kv`、`ki`、最大倾角、推力和加速度限制；这些参数随机型变化。
4. 确认起飞/Offboard 模式管理器只在控制器预流完成后切换，且飞行中仅有一个节点拥有 Offboard 控制模式。
5. 观察 `/px4ctrl/status`、`/px4ctrl/debug` 和 PX4 状态；超时、定位无效、Offboard 丢失或 failsafe 时以飞控故障逻辑为准。

本包没有 ROS service/action 接口；其集成接口是上述话题和 ROS 参数。
