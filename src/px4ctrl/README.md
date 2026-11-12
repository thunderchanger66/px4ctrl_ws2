# px4ctrl — PX4 外部姿态控制器（中文使用说明）

> `px4ctrl` 是一个**与规划器完全解耦**的 ROS 2 外部控制器。
> 它接收一条"期望位置/速度/加速度/偏航"参考，自己算出"机体该摆成什么姿态、
> 油门给多少"，直接发给 PX4 执行。PX4 的原生位置环不参与这段飞行。

---

## 目录

1. [它到底在干什么](#1-它到底在干什么)
2. [五分钟快速开始](#2-五分钟快速开始)
3. [输入契约：参考量](#3-输入契约参考量)
4. [输出契约：PX4 指令](#4-输出契约px4-指令)
5. [两种接入模式](#5-两种接入模式)
6. [状态机与状态字符串](#6-状态机与状态字符串)
7. [控制律](#7-控制律)
8. [参数完全手册（56 个）](#8-参数完全手册56-个)
9. [配置文件](#9-配置文件)
10. [第一步必做：标定 hover_thrust](#10-第一步必做标定-hover_thrust)
11. [安全机制清单](#11-安全机制清单)
12. [排障手册](#12-排障手册)
13. [与 eight 试验流程集成](#13-与-eight-试验流程集成)
14. [从旧版本迁移](#14-从旧版本迁移)
15. [深入阅读](#15-深入阅读)

---

## 1. 它到底在干什么

```
 任意参考源（规划器 / 轨迹生成器 / 测试脚本）
        │
        │  /px4ctrl/desired_state      px4ctrl/msg/DesiredState
        │  位置 / 速度 / 加速度 / 偏航   一律 PX4 本地 NED
        ▼
 ┌──────────────────────────────┐
 │  px4ctrl_node                │  串级 PID
 │  · 位置误差 → 速度参考        │  + 加速度前馈
 │  · 速度误差 → 加速度指令      │  + 积分补偿
 │  · 加速度 → 姿态 + 推力      │  + 可行域投影
 └──────────────────────────────┘
        │  /fmu/in/vehicle_attitude_setpoint_v1
        │      q_d (期望姿态四元数)
        │      thrust_body = (0, 0, −T)
        │      yaw_sp_move_rate (偏航角速度前馈)
        ▼
 PX4 飞控：姿态环 → 角速度环 → 电机分配
        ▲
        │  /fmu/out/vehicle_local_position_v1（唯一反馈源）
```

**关键设计取舍：绕过 PX4 的位置环，自己重写一套。**

PX4 的 `mc_pos_control` 本来就有一整套成熟的位置/速度/加速度级联控制器。
`px4ctrl` 把它在 ROS 2 里重写了一遍（结构几乎逐行对应，见 `MIGRATION_GUIDE.md`）。
代价是**PX4 的安全兜底也跟着一起绕过了**，所以倾角限制、推力限幅、抗积分饱和、
EKF 复位补偿、失效保护这些全都必须由本控制器自己实现。

**收益**：轨迹跟踪的每一个细节都可控、可调、可观测，不受 PX4 内部实现的约束。

**代价**：`px4ctrl` 有 1194 行代码，而且**你必须自己把参考量算对**。

### 与规划器的边界

| px4ctrl **知道** | px4ctrl **不知道** |
|---|---|
| PX4 的话题、姿态/推力的几何约定 | 参考来自哪个规划器 |
| PX4 本地 NED 的坐标约定 | 参考源用的是什么内部坐标系 |
| 串级 PID 的结构 | 参考源有没有做路径规划 |
| 安全阈值与失效策略 | 参考源是不是状态连续的 |

一句话：**谁产生参考，谁负责把参考换算成 PX4 本地 NED。**

---

## 2. 五分钟快速开始

### 2.0 当前 MID-360 + FAST-LIO + EGO SITL 一键启动

PX4/Gazebo 与 FAST-LIO/PGO 仍按项目现有方式先启动。随后运行：

```bash
cd /home/thunder/GRA_1/px4ctrl_ws2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4ctrl bringup.launch.py
```

该 launch 启动 EGO、FAST-LIO 到 PX4 EKF 的视觉里程计桥、EGO 参考适配器和
`px4ctrl`。适配器将 `/drone_0_planning/pos_cmd` 从 `lidar` 转成控制器需要的
PX4 本地 NED，并与 `/fmu/out/vehicle_local_position_v1` 对齐原点。启动时飞机
应静止在地面，且 PX4 本地位置和 FAST-LIO 里程计都有效；成功后可检查
`/px4ctrl/reference_adapter_status`，看到 `publishing_px4_local_ned_reference`。
先单独起飞并等待悬停稳定，再发送 EGO 导航目标。`takeoff` 向 px4ctrl 提供原地爬升
参考，不会触发 EGO 规划；返回成功后再用 `manual_goal`。

```bash
ros2 run px4ctrl takeoff --altitude 0.5 --wait-timeout 45

# takeoff 命令成功返回后，再开始 EGO 规划
ros2 run px4ctrl manual_goal 4.0 -4.0 0.5 90
# Omit yaw_deg (or pass --auto-yaw) to face along EGO's path
ros2 run px4ctrl manual_goal 4.0 -4.0 0.5 --auto-yaw
```

`takeoff` 等待 PX4 本地位置有效、控制器进入跟踪状态，并在目标高度附近保持低速
0.8 s。若定位无效、解锁/Offboard 失败或没有稳定到达高度，会超时并报告最后状态；
此时不要发送 EGO 导航目标。

该命令把目标发到 EGO 的 `/goal_pose`，数值属于 `lidar` 规划系；它不是 PX4 NED
目标消息。传入 `yaw_deg` 时同时向控制器发送固定的 PX4 NED 航向；省略 yaw 或使用
`--auto-yaw` 时清除固定航向覆盖，控制器跟随 EGO 根据轨迹方向生成的自动航向。

PX4 启动文件已经运行 `MicroXRCEAgent` 时，使用
`start_agent:=false` 避免重复占用 UDP 8888。默认不另起 Gazebo `/clock` bridge，
因为 MID-360 SITL launch 已经桥接 `/clock`。`start_rviz:=true` 可选启动 EGO RViz。

**不要把 `manual_goal` 的数值直接解释成 NED 数值。** 本配置中的 EGO 目标是在
`lidar` 规划系中指定，适配器负责映射到 PX4 NED。若 EGO 和 PX4 已经在运行中，
先确认飞机静止并重启 bringup，使 FAST-LIO/PX4 起始原点重新对齐。

### 2.1 构建

```bash
source /opt/ros/jazzy/setup.bash          # 按你的 ROS 2 发行版调整
cd /home/zone/px4ctrl_ws-main
colcon build --packages-up-to px4ctrl --symlink-install --executor sequential
source install/setup.bash
```

控制器核心依赖 `rclcpp`、`px4_msgs`、`geometry_msgs`、`std_msgs` 和 Eigen3，
不依赖规划器。`bringup.launch.py` 额外启动 EGO 参考适配器，因此还需要工作区中
已有的 `nav_msgs`、`quadrotor_msgs` 和 `fastlio2_px4_odom_bridge`。

### 2.2 启动前的三项检查

```bash
# ① PX4 的 DDS 客户端要连上（MicroXRCEAgent 或 PX4 自带的 uXRCE-DDS Client）
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once

# ② PX4 本地位置必须有效（xy_valid / z_valid / v_xy_valid / v_z_valid 全为 true）
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once | grep valid

# ③ 时钟要一致（SITL 必须有 /clock 且 use_sim_time: true）
ros2 topic echo /clock --once
```

### 2.3 启动控制器

```bash
# 用通用模板（先改 hover_thrust 等机型参数）
ros2 run px4ctrl px4ctrl_node --ros-args --params-file /path/to/your.yaml

# 或用现成的裸 X500 SITL 配置
ros2 run px4ctrl px4ctrl_node --ros-args \
  --params-file /home/zone/px4ctrl_ws-main/src/px4ctrl/config/px4ctrl_x500.yaml
```

也可以用 launch（会顺带起 Gazebo `/clock` 桥接）：

```bash
ros2 launch px4ctrl controller_only.launch.py \
  px4ctrl_params:=/path/to/your.yaml start_agent:=false
```

### 2.4 喂一条参考

```python
from px4ctrl.msg import DesiredState

cmd = DesiredState()
cmd.header.stamp = node.get_clock().now().to_msg()
cmd.header.frame_id = "px4_local_ned"     # 必须等于 reference_frame
cmd.position.x = 1.0      # 北向 1 m
cmd.position.y = 0.0      # 东向 0 m
cmd.position.z = -2.0     # 向上 2 m（NED 的 z 朝下！）
cmd.velocity.x = cmd.velocity.y = cmd.velocity.z = 0.0
cmd.acceleration.x = cmd.acceleration.y = cmd.acceleration.z = 0.0
cmd.yaw = 0.0             # 航向 0 rad
cmd.yaw_rate = 0.0
publisher.publish(cmd)    # 持续以 20~50 Hz 发布
```

### 2.5 观察

```bash
ros2 topic echo /px4ctrl/status        # 状态机走到哪一步了
ros2 topic echo /px4ctrl/debug         # 误差、推力、倾角、饱和标志
```

---

## 3. 输入契约：参考量

### 3.1 消息定义

`px4ctrl/msg/DesiredState`：

| 字段 | 类型 | 单位 | 约定 |
|---|---|---|---|
| `header.stamp` | `builtin_interfaces/Time` | ROS 时间 | 必须 > 0、严格递增；仿真中用 `/clock` |
| `header.frame_id` | `string` | — | **必须等于 `reference_frame`**（默认 `px4_local_ned`） |
| `position` | `geometry_msgs/Point` | m | PX4 本地 NED |
| `velocity` | `geometry_msgs/Vector3` | m/s | PX4 本地 NED |
| `acceleration` | `geometry_msgs/Vector3` | m/s² | PX4 本地 NED（无前馈时显式填 0） |
| `yaw` | `float64` | rad | NED 水平面航向角 |
| `yaw_rate` | `float64` | rad/s | 航向角变化率 |

### 3.2 坐标系：唯一且硬性的要求

**参考量必须已经表达在 PX4 本地 NED 中。**

```
NED:  x = 北   y = 东   z = 下
```

因此：

| 常见错误 | 正确写法 |
|---|---|
| 想往上飞 2 m | `position.z -= 2.0`（z 朝下） |
| 高度 10 m（海拔） | `position.z = -10.0` |
| 航向朝北 | `yaw = 0.0` |
| 航向朝东 | `yaw = +π/2`（NED 航向是**顺时针**为正） |

**控制器不对参考量做任何旋转或平移。** 没有旋转矩阵参数，没有平移向量参数，
也没有原点对齐流程。

> **为什么这么严格？**
> 控制器一旦要支持"从任意坐标系到 NED"的变换，就必须知道变换矩阵，
> 而矩阵又依赖外部里程计或标定过程——那等于把规划器的坐标系知识漏回控制器，
> 控制器就不再是"黑盒子"了。
>
> 更实际的理由：**用错坐标系是致命的静默失败。** 把 ENU 的数塞进 NED，
> 误差能有几百米。唯一的防线就是 `header.frame_id` 校验——不匹配直接丢弃，
> 飞机纹丝不动。**响亮地不动，好过安静地飞错方向。**

### 3.3 指令准入检查

一条参考要真正被采用，必须同时通过（`command_callback`）：

| 检查 | 阈值 | 不通过的后果 |
|---|---|---|
| `frame_id` == `reference_frame` | 精确匹配 | 丢弃 |
| 时间戳 > 0 | — | 丢弃 |
| 不早于上一条 | 严格递增 | 丢弃（防乱序/重复） |
| `now − stamp ≤ command_timeout_sec` | 0.5 s | 丢弃 → 回悬停 |
| `stamp ≤ now + 0.05 s` | 不能来自未来 | 丢弃 |
| 所有数值 finite | 非 NaN/Inf | 丢弃 |
| 位置跳变 ≤ `max_command_position_jump_m` | 0.35 m | 触发 `command_blend` 平滑 |
| 速度跳变 ≤ `max_command_velocity_jump_mps` | 1.5 m/s | 触发 `command_blend` 平滑 |

> **跳变检查的用意**：正常的重规划是状态连续的，位置不会突变。突变说明参考源
> 复位或换了个起点，此时控制器从"当前正在施加的参考"平滑过渡过去，
> 而不是把一个阶跃直接塞进姿态环。

---

## 4. 输出契约：PX4 指令

### 4.1 话题

| 方向 | 默认话题 | 消息 | 说明 |
|---|---|---|---|
| PX4 → 控制器 | `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` | **唯一反馈源**（位置/速度/航向/有效性） |
| PX4 → 控制器 | `/fmu/out/vehicle_status_v4` | `VehicleStatus` | 解锁 / Offboard / 预检 / failsafe |
| PX4 → 控制器 | `/fmu/out/vehicle_land_detected` | `VehicleLandDetected` | 空中接管时确认已离地（约 1 Hz） |
| PX4 → 控制器 | `/fmu/out/vehicle_attitude` | `VehicleAttitude` | 仅用于刷新 PX4 微秒时间戳 |
| 控制器 → PX4 | `/fmu/in/offboard_control_mode` | `OffboardControlMode` | `attitude = true`，声明使用外部姿态目标 |
| 控制器 → PX4 | `/fmu/in/vehicle_attitude_setpoint_v1` | `VehicleAttitudeSetpoint` | **核心输出** |
| 控制器 → PX4 | `/fmu/in/vehicle_command` | `VehicleCommand` | 仅在自动切模式/解锁路径或安全故障降落时发送 |

> **`_v1` 后缀**：PX4 1.x 的 uORB 输入带版本号。发到不存在的旧名字，PX4
> 根本没有订阅者，指令会被静默丢弃。如果你的固件用别的名字，改
> `attitude_setpoint_topic` / `px4_local_position_topic` 等参数。

### 4.2 姿态设定点的几何约定

```cpp
setpoint.q_d            = {w, x, y, z}      // 期望姿态四元数（世界系 → 机体系）
setpoint.thrust_body    = {0, 0, −T}        // 注意负号
setpoint.yaw_sp_move_rate = ω              // 偏航角速度前馈，rad/s
```

- **`thrust_body` 的 z 分量是负的**：PX4 的机体系 z 轴**朝下**，所以向上的推力
  是负值。`T` 是归一化总推力，`1.0` = 满油门。
- **`T` 与 `hover_thrust` 的关系**：`hover_thrust` 是"悬停时需要的归一化总推力"。
  它是整个推力换算的基准，**必须按机型实测标定**（见 §10）。
- **PX4 只执行姿态**：`OffboardControlMode.attitude = true` 意味着 PX4 的
  `mc_pos_control`（位置环）整个停摆，只跑 `mc_att_control`（姿态环）→
  `mc_rate_control`（角速率环）→ 电机分配。

### 4.3 观察者话题

| 话题 | 消息 | 内容 |
|---|---|---|
| `/px4ctrl/status` | `std_msgs/String` | 状态机字符串（见 §6） |
| `/px4ctrl/debug` | `px4ctrl/msg/ControlDebug` | 误差、参考速度、期望/可行加速度、推力、倾角、偏航角速度、指令延迟、饱和标志、PX4 健康状态 |

`ControlDebug` 里的向量**全部是 PX4 本地 NED**。

`saturation_flags` 位定义：

| 位 | 含义 | 触发条件 |
|---|---|---|
| bit 0 | 总加速度限幅 | `\|a\| > max_acceleration_mps2` |
| bit 1 | 垂直力包络限幅 | 需要的推力超出 `[min_thrust, max_thrust]` |
| bit 2 | 水平倾角/力限幅 | 超出倾角或推力余量所能提供的水平力 |
| bit 3 | 最终推力裁剪 | 撞到 `min_thrust` 或 `max_thrust` |

**任何一位非零 = 这一帧的指令被约束了。** 频繁置位说明参数或标定有问题。

---

## 5. 两种接入模式

由 `entry_mode` 选择。

### 5.1 `auto_takeoff`（默认）——控制器自己起飞

```
waiting_for_command
   │ 收到第一条有效参考
   ▼
takeoff  ──► 切 Offboard（若 start_offboard_on_first_command）→ 解锁（若 arm_on_first_command）
   │          爬升到 auto_takeoff_altitude_m（NED 高度增加）
   ▼
hover  ──► 悬停稳定（速度 < hover_velocity_tolerance_mps 持续 hover_settle_sec）
   │ 收到新参考
   ▼
tracking ──► 交接平滑（tracking_handoff_sec）→ 正式跟踪
```

适合：控制器是系统里唯一负责"起飞 + 跟踪"的角色。

**前提条件**：PX4 预检通过、本地定位有效、`hover_thrust` 已标定。
预检不通过时 PX4 会拒绝解锁，控制器会一直重试（1 秒一次）。

### 5.2 `airborne_takeover`——接管已在空中的机子

```
waiting_for_command
   │ 七道门全部满足（见下）
   ▼
hover  ──► 抓住当前位置作为悬停点，开始发姿态设定值
   │ 收到新参考
   ▼
tracking
```

**七道门**（`begin_airborne_takeover()`，必须同时满足）：

1. `local_position_healthy()` —— EKF 状态有效
2. 已 arm
3. 无 failsafe
4. `VehicleLandDetected` 新鲜（≤ `land_detected_timeout_sec`）且
   `landed` / `ground_contact` / `maybe_landed` **三者全为 false**（确认在空中）
5. 速度 ≤ `takeover_velocity_tolerance_mps`
6. 连续稳定 ≥ `max(1.0, hover_settle_sec)` 秒
7. `require_takeover_permission` 为真时，`takeover_permission_topic` 必须发来 `true`

**这个模式绝不**发 `DO_SET_MODE`、**绝不**发 ARM、**绝不**自己起飞。
切模式和解锁由上层负责。

> **门 4 为什么这么严？** 旧流程里"自动起飞"曾因为 `landed` 一直为真、
> NED 高度一直是 0 而失败——飞机没真正离地，PX4 的落地检测就不会翻转，
> 控制器就永远进不了接管状态。改成"位置 Offboard 起飞"后才解决。
> 另外 PX4 的落地检测在状态不变时**只以 1 Hz 发布**，所以超时阈值要放宽到 2 s。

### 5.3 指令超时 → 自动回悬停

`tracking` 状态下如果参考超时（`command_timeout_sec`），控制器会：
捕获当前位置 → 速度/加速度/偏航角速度清零 → 回到 `hover`。
**不会**继续追着最后一条指令飞。这是重要的安全兜底。

---

## 6. 状态机与状态字符串

`/px4ctrl/status` 是人可读的状态机输出，排查问题第一眼看它。

| 状态字符串 | 含义 | 该做什么 |
|---|---|---|
| `waiting_for_px4_local_position_and_reference_command` | 启动时（auto_takeoff 模式），等 PX4 位置 + 第一条参考 | 检查 PX4 连接、参考的 `frame_id` |
| `waiting_for_airborne_takeover` | 启动时（airborne_takeover 模式） | — |
| `waiting_for_armed_stationary_airborne_state` | 七道门还没过 | 看下面"接管排查" |
| `waiting_for_fresh_px4_local_position` | PX4 位置反馈超时 | 检查 EKF、话题 |
| `waiting_for_healthy_px4_local_position` | EKF 状态无效 | 检查 `xy_valid` 等标志 |
| `waiting_for_fresh_px4_status` | VehicleStatus 超时 | 检查 PX4 连接 |
| `waiting_for_px4_clock` | 还没拿到 PX4 时间戳 | 检查 PX4 状态流 |
| `waiting_for_command_waiting_for_goal` | 等第一条参考 | 检查参考源 |
| `state_takeoff` | 爬升中 | — |
| `state_hover` | 悬停稳定 | — |
| `state_hover_minimum_altitude` | 因"最低爬升量"分支提前结束起飞 | 正常行为，见 §8.5 |
| `state_tracking` | 跟踪中 | — |
| `state_hover_command_timeout` | 参考断了，已回悬停 | 检查参考源是否还在发 |
| `tracking_attitude_thrust_tracking` | 跟踪中（含相位前缀的完整状态） | 正常 |
| `hover_command_stale_hovering` / `tracking_command_stale_hovering` | 参考已过期，正在悬停 | 检查参考源 |
| `takeoff_attitude_thrust_tracking` | 起飞阶段的相位前缀状态 | — |
| `hover_airborne_takeover_prestream` | 刚抓住悬停点，开始预发设定值 | — |
| `hover_airborne_takeover_ready` | 预发满 `prestream_seconds` | 上层可以请求切 Offboard 了 |
| `hover_airborne_takeover_offboard_ready` | 已在 Offboard 且状态就绪 | **可以开始发参考了** |
| `attitude_offboard_requested` | 已请求切 Offboard | 等 PX4 响应 |
| `attitude_offboard_arm_requested` | 已请求解锁 | 等 PX4 响应 |
| `safety_fault_<reason>` | **锁存的安全故障，永久停止输出** | 见 §11 |
| `safety_latched_no_setpoint` | 故障锁存中，不再发任何设定值 | — |
| `control_suspended_px4_status_timeout` | PX4 状态流超时，挂起 | 检查 DDS 连接 |
| `control_suspended_px4_failsafe` | PX4 进 failsafe，挂起 | 按飞控故障逻辑处理 |
| `control_suspended_offboard_lost` | 掉出 Offboard，挂起 | 见 §12 |
| `control_suspended_vehicle_disarmed` | 掉 arm，挂起 | — |

**`safety_fault_*` 是锁存的（latched）**：一旦发生就不会自动恢复，
必须重启节点。原因是这类故障说明状态已经不可信了。

---

## 7. 控制律

### 7.1 串级结构

```
                 ┌────────────── 参考 (p_d, v_d, a_d) ──────────────┐
                 │                                                │
                 ▼                                                │
   ┌──────────────────────────┐                                    │
   │ 位置环                    │  v_ref = v_d + (kp/kv)·(p_d − p)   │
   └──────────────────────────┘                                    │
                 ▼                                                │
   ┌──────────────────────────┐                                    │
   │ 速度环 (PI)               │  e_v = v_ref − v                  │
   └──────────────────────────┘                                    │
                 ▼                                                │
   ┌──────────────────────────┐                                    │
   │ 加速度合成                │  a = a_d + kv·e_v + ∫ki·e_v dt     │
   └──────────────────────────┘                                    │
                 ▼                                                │
        可行域投影（加速度 → 姿态 + 推力） ◄─────────────────────────┘
```

**三个增益的物理含义**（这是最容易搞错的地方）：

| 参数 | 含义 | 单位 | 默认 |
|---|---|---|---|
| `kp` | **加速度 / 位置误差** | (m/s²)/m | `[1.5, 1.5, 1.8]` |
| `kv` | **加速度 / 速度误差** | (m/s²)/(m/s) | `[1.2, 1.2, 1.4]` |
| `ki` | 积分增益 | (m/s²)/(m/s)/s | `[0.10, 0.10, 0.15]` |

位置环的等效增益是 `kp/kv = [1.25, 1.25, 1.286]`。
代码里 `velocity_reference(i) += kp_(i) / kv_(i) * position_error(i)` 就是这个意思。

### 7.2 积分的两个保护

1. **只在小误差时积分**：`|e_v| > integral_enable_velocity_error_mps`（0.6）时不积分，
   防止大误差期间积分饱和。
2. **饱和时衰减**：一旦本帧输出被约束（`saturation_flags != 0`），
   积分项按 `× 0.995` 衰减。这是对抗积分饱和（anti-windup）。

积分项还被 `integral_limit_mps2` 限幅。

### 7.3 可行域投影（这个控制器最有价值的设计）

**不要先算推力再硬夹。** 那样做的话：你要求 4 m/s² 的水平加速，但推力只剩 3% 余量
→ 推力被夹到很小 → 但姿态还朝着 4 m/s² 的方向倾斜 → 结果是"用力气朝一个方向顶"，
产生巨大超调。

正确做法：**先把加速度降到物理可达的范围，再由可达的加速度反推姿态和推力。**

```
垂直方向：
  requested_vertical_force = g − a_z            （a_z 负 = 上升，需要更大力）
  vertical_force = clamp(requested, min_force, max_force)
  a_z = g − vertical_force

  max_force = g · max_thrust / hover_thrust      （推力归一化 → 力的换算）
  min_force = g · min_thrust / hover_thrust

水平方向（两条约束取较小者）：
  from_tilt   = tan(max_tilt) · vertical_force           （倾角约束）
  from_thrust = sqrt(max_force² − vertical_force²)       （推力余量约束）
  max_horizontal_accel = min(from_tilt, from_thrust)
```

### 7.4 加速度 → 姿态 + 推力

```cpp
force      = acceleration − [0, 0, g]        // 与 PX4 位置控制器同一个表达式
body_z     = limit_body_z_tilt(−force, max_tilt_rad)
force_down = max(0.1, g − a_z)
thrust     = hover_thrust · force_down / (g · body_z_down)
thrust     = clamp(thrust, min_thrust, max_thrust)
```

自洽性检查（悬停）：`a = 0` → `force = [0,0,−g]` → `body_z = [0,0,1]`
→ `thrust = hover_thrust · g / (g·1) = hover_thrust` ✓

### 7.5 偏航角伺服

偏航不是直接设定，而是走一个**带限幅和限加速度的 PI 环**：

```cpp
yaw_error        = remainder(yaw_target − yaw_setpoint, 2π)
yaw_feedforward  = 参考的 yaw_rate（仅 tracking 且无人工覆盖时）
desired_yaw_rate = clamp(yaw_feedforward + 2·yaw_error, ±max_yaw_rate_radps)
yaw_rate_setpoint += clamp(desired − current, ±max_yaw_acceleration_radps2·dt)
yaw_setpoint     += yaw_rate_setpoint · dt
```

两个作用：
- 参考源要求 180° 转身时不会一步到位，而是以 ≤ 0.6 rad/s 转过去；
- 偏航角速度也有加速度限制，避免抖动。

起飞和悬停阶段保持起飞时的航向，不跟随参考的航向。

---

## 8. 参数完全手册（56 个）

> 参数在**启动时读一次**并固化到内部变量。
> `ros2 param set` **不会生效**，必须改 YAML 后重启节点。
> 改完不重启是最常见的"改了没用"困惑来源。

### 8.1 话题类

| 参数 | 默认 | 说明 |
|---|---|---|
| `reference_frame` | `px4_local_ned` | 参考量必须声明的 `frame_id`。**只校验不改数值** |
| `desired_state_topic` | `/px4ctrl/desired_state` | 参考输入 |
| `status_topic` | `/px4ctrl/status` | 状态字符串输出 |
| `debug_topic` | `/px4ctrl/debug` | 调试输出 |
| `px4_status_topic` | `/fmu/out/vehicle_status_v4` | PX4 状态 |
| `px4_attitude_topic` | `/fmu/out/vehicle_attitude` | 仅刷新 PX4 时间戳 |
| `px4_local_position_topic` | `/fmu/out/vehicle_local_position_v1` | **唯一反馈源** |
| `px4_land_detected_topic` | `/fmu/out/vehicle_land_detected` | 空中接管判定落地 |
| `offboard_control_mode_topic` | `/fmu/in/offboard_control_mode` | Offboard 模式声明 |
| `attitude_setpoint_topic` | `/fmu/in/vehicle_attitude_setpoint_v1` | **核心输出**（注意 `_v1`） |
| `vehicle_command_topic` | `/fmu/in/vehicle_command` | 模式/解锁/降落指令 |
| `manual_yaw_topic` | `/px4ctrl/manual_yaw` | 人工覆盖目标 yaw（`Float64`，NED 弧度） |
| `clear_manual_yaw_topic` | `/px4ctrl/clear_manual_yaw` | 取消人工 yaw（`Empty`） |
| `takeover_permission_topic` | `/eight/takeover_allowed` | 空中接管许可（`Bool`），默认指向 eight 包 |

### 8.2 时序 / 超时类

| 参数 | 默认 | 说明 |
|---|---|---|
| `publish_rate_hz` | 100.0 | 控制环频率。SITL 用 Gazebo `/clock`，仿真变慢会一起变慢 |
| `command_timeout_sec` | 0.50 | 参考超时 → 回悬停 |
| `px4_status_timeout_sec` | 1.50 | VehicleStatus 超时（该流在 SITL 频率低） |
| `local_position_timeout_sec` | 0.50 | PX4 位置反馈超时 → 安全故障 |
| `land_detected_timeout_sec` | 2.0 | 落地检测话题超时。**该流在状态不变时只有 1 Hz**，别调小 |
| `prestream_seconds` | 1.0 | 切 Offboard 前预发设定值的时长 |
| `local_position_fault_delay_sec` | 1.00 | 持续异常多久才判安全故障（防抖） |
| `command_blend_sec` | 0.25 | 指令跳变时的平滑时长 |
| `tracking_handoff_sec` | 0.75 | hover→tracking 交接平滑时长 |
| `takeover_velocity_tolerance_mps` | 0.20 | 空中接管速度门限 |

### 8.3 开关类

| 参数 | 默认 | 说明 |
|---|---|---|
| `entry_mode` | `auto_takeoff` | `auto_takeoff`=自己起飞；`airborne_takeover`=接管已在空中的机子 |
| `start_offboard_on_first_command` | true | 收到第一条参考后是否请求切 Offboard |
| `arm_on_first_command` | true | 是否自动解锁 |
| `require_takeover_permission` | false | 是否需要 `takeover_permission_topic` 许可 |
| `require_local_position_valid` | true | 是否强制要求 EKF 位置有效 |
| `land_on_safety_fault` | true | 安全故障时是否下发 LAND |

### 8.4 物理 / 推力类

| 参数 | 默认 | 说明 |
|---|---|---|
| `gravity_mps2` | 9.81 | 重力加速度 |
| `hover_thrust` | 0.864 | **悬停时的归一化总推力**（1.0 = 满油门）。**必须实测标定！** |
| `min_thrust` | 0.08 | 最小推力（下限，防熄火） |
| `max_thrust` | 0.95 | 最大推力（上限，留余量） |
| `max_tilt_deg` | 30.0 | 最大倾角 |
| `max_acceleration_mps2` | 4.0 | 最大加速度指令 |
| `max_ekf_reset_delta_m` | 1.0 | EKF 平移重置补偿上限，超过 → 安全故障 |
| `max_ekf_reset_heading_rad` | 0.35 | EKF 航向重置补偿上限 |
| `max_local_position_m` | 100.0 | 位置合理性上限 |
| `max_local_velocity_mps` | 15.0 | 速度合理性上限 |

### 8.5 起飞 / 高度类

| 参数 | 默认 | 说明 |
|---|---|---|
| `auto_takeoff_altitude_m` | 0.50 | 起飞目标高度（NED 高度增加量）。0 = 不起飞，直接 hover |
| `minimum_takeoff_altitude_m` | 0.25 | 判定"已离地"的最小爬升量（补偿标定误差） |
| `minimum_tracking_altitude_m` | 0.25 | 跟踪阶段的最低飞行高度（相对起飞原点） |
| `takeoff_duration_sec` | 3.0 | 起飞最短持续时间 |
| `takeoff_tolerance_m` | 0.08 | 到达目标高度的位置容差 |
| `hover_settle_sec` | 0.80 | 悬停稳定判定时间 |
| `hover_velocity_tolerance_mps` | 0.15 | 悬停稳定的速度门限 |

> **起飞有两个"完成"判据**：到达目标高度（`at_height`）**或**至少爬升了
> `minimum_takeoff_altitude_m`（`safely_airborne`）。后者是给标定有偏差的载荷用的
> ——如果标定的悬停推力偏高，实际爬升可能到不了名义目标高度。
> 走第二条路径时状态字符串是 `state_hover_minimum_altitude`。

### 8.6 指令合理性 / 偏航类

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_command_position_jump_m` | 0.35 | 单帧位置跳变上限，超过 → 触发 blend |
| `max_command_velocity_jump_mps` | 1.5 | 单帧速度跳变上限 |
| `max_yaw_rate_radps` | 0.6 | 偏航角速度上限 |
| `max_yaw_acceleration_radps2` | 0.8 | 偏航角加速度上限 |

### 8.7 PID 类

| 参数 | 默认 | 含义 |
|---|---|---|
| `kp` | `[1.5,1.5,1.8]` | 加速度 / 位置误差 (m/s² per m) |
| `kv` | `[1.2,1.2,1.4]` | 加速度 / 速度误差 (m/s² per m/s) |
| `ki` | `[0.10,0.10,0.15]` | 积分增益 (m/s² per m/s per s) |
| `integral_limit_mps2` | `[0.40,0.40,0.50]` | 积分项限幅 (m/s²) |
| `integral_enable_velocity_error_mps` | 0.60 | 速度误差超过此值就不积分 |

---

## 9. 配置文件

| 文件 | 用途 |
|---|---|
| `config/px4ctrl_default.yaml` | **通用模板**。新机型从这里改 |
| `config/px4ctrl_x500.yaml` | 裸 Gazebo X500 的 SITL 配置，被 `eight` 包引用 |

### `px4ctrl_default.yaml` 的关键项

```yaml
use_sim_time: false              # 硬件默认；SITL 改成 true
reference_frame: px4_local_ned
entry_mode: auto_takeoff
start_offboard_on_first_command: true
arm_on_first_command: true
require_takeover_permission: false
auto_takeoff_altitude_m: 0.50
hover_thrust: 0.60                # ← 占位值！必须实测标定
```

### `px4ctrl_x500.yaml` 的关键项

```yaml
use_sim_time: true
entry_mode: airborne_takeover    # 接管已在空中的机子
auto_takeoff_altitude_m: 0.0     # 不自己起飞
start_offboard_on_first_command: false   # 不抢外部的控制权
arm_on_first_command: false
require_takeover_permission: true        # 等 eight 包批准
hover_thrust: 0.60               # = PX4 裸 X500 的 MPC_THR_HOVER，仍需实测确认
```

---

## 10. 第一步必做：标定 hover_thrust

**这是最重要的一次调参。** `hover_thrust` 是整个推力换算的基准：
它错了，垂向控制就是错的，表现为持续下沉/上浮或频繁触发饱和。

```bash
ros2 run px4ctrl offboard_thrust_probe --csv /tmp/thrust.csv
```

脚本会按 `--thrust-steps`（默认 `0.55 0.65 0.75 0.85 0.90`）逐级加推力爬升，
把每次的稳态推力记进 CSV。挑**悬停稳定且不持续饱和**的那一档，填回 YAML。

手工标定也可以：让飞机稳定悬停，直接读

```bash
ros2 topic echo /px4ctrl/debug --field thrust
```

**注意**：
- 装了云台/激光雷达后 `hover_thrust` 会明显变大（带载荷的 X500 在 SITL 里约 0.864，
  裸机约 0.60）。
- 每次改机型（换电池、换载荷、换桨）都要重新标定。
- `hover_thrust` 落在 `[min_thrust, max_thrust]` 之外时，节点会在启动时报
  `invalid px4ctrl parameters` 并拒绝启动。

---

## 11. 安全机制清单

| 机制 | 触发条件 | 动作 |
|---|---|---|
| 参考超时 | 参考超过 `command_timeout_sec` 未更新 | 回悬停（位置保持），速度/加速度清零 |
| 参考跳变平滑 | 单帧位置跳变 > 0.35 m | 从当前参考 smoothstep 过渡 `command_blend_sec` |
| 位置反馈超时 | `VehicleLocalPosition` 超 0.5 s | 持续 `local_position_fault_delay_sec` → 安全故障 |
| EKF 状态无效 | `xy_valid`/`z_valid`/速度标志为假，或 `dead_reckoning` | 同上 |
| 位置/速度不合理 | 超出 `max_local_position_m` / `max_local_velocity_mps` | 同上 |
| EKF 重置过大 | 重置量 > `max_ekf_reset_delta_m` 或航向 > `max_ekf_reset_heading_rad` | **锁存安全故障** |
| 正常 EKF 重置 | 计数器变化但在阈值内 | 自动补偿到所有目标点（不报错） |
| Offboard 丢失 | 已见过 Offboard 但 `nav_state` 变了 | 挂起控制，停止发设定值 |
| PX4 failsafe | `VehicleStatus.failsafe` | 挂起控制 |
| 掉 arm | airborne 模式下 `arming_state` 变 DISARMED | 挂起控制 |
| 抗积分饱和 | 输出被约束时 | 积分项 × 0.995 衰减 |
| 推力/倾角/加速度限幅 | 超出物理能力 | 投影到可行域，置饱和标志 |
| 高度下限保护 | 参考的 z 低于安全下限（仅 `auto_takeoff` 模式） | 保持当前高度，不下降 |
| 安全故障降落 | 锁存故障且 `land_on_safety_fault: true` | 下发 `VEHICLE_CMD_NAV_LAND` |

**锁存（latched）的故障**：`safety_fault_*` 一旦发生就永久停止输出，
必须重启节点。因为这类故障说明状态已经不可信。

**PX4 自己的 failsafe 仍然有效**。本控制器只是不再额外接管那些职责，
不意味着 PX4 的保护被关掉了。

---

## 12. 排障手册

### 12.1 节点起不来

| 现象 | 原因 |
|---|---|
| `parameter 'xxx' was removed` | 配置里还有旧版本的参数（§14），按提示删掉 |
| `invalid px4ctrl parameters` | 参数范围非法。最常见是 `hover_thrust` 不在 `(min_thrust, max_thrust)` 内，或 `minimum_takeoff_altitude_m > auto_takeoff_altitude_m` |
| `publish_rate_hz must be >= 10` | 频率设得太低 |
| 节点起来了但什么也不发 | 看 `/px4ctrl/status`，见 §12.2 |

### 12.2 节点在跑但没有设定值

```bash
ros2 topic echo /px4ctrl/status --once
```

| 状态 | 检查 |
|---|---|
| `waiting_for_px4_local_position_and_reference_command` | ① PX4 位置来了吗 ② 参考的 `frame_id` 是不是等于 `reference_frame`（**最常见**） |
| `waiting_for_fresh_px4_local_position` / `_healthy_` | PX4 本地位置没来或 EKF 状态无效 |
| `waiting_for_fresh_px4_status` | PX4 状态流超时，检查 DDS |
| `waiting_for_armed_stationary_airborne_state` | 见 §12.3 |
| `waiting_for_px4_clock` | PX4 时间戳还没拿到 |
| `state_hover_command_timeout` | 参考断了，检查参考源还在发吗 |

### 12.3 接管进不去（airborne_takeover）

逐条核对七道门。最常见的三个：

1. **`landed` 一直为真** → 飞机没真正离地，或者落地检测消息太稀
   （`land_detected_timeout_sec` 默认 2.0 s 就是为此放宽的）。
2. **`permission` 一直是 false** → 上层没发许可，或飞机还没到目标点。
3. **速度超 0.20 m/s** → 飞机还在动。

### 12.4 飞的方向不对

**参考源没换算到 NED。** 自检方法：让参考源发一个"当前位置 + 1 m 北向"的定值，
观察飞机是否往北飞。

| 症状 | 大概率原因 |
|---|---|
| 位置差一个固定的大偏差 | 平移没换算（尤其高度：ENU 的 z 朝天，NED 的 z 朝下） |
| 位置对但走向反了 | x/y 没交换（`NED_x = lidar_y`、`NED_y = lidar_x`） |
| 转向反了 | 偏航角符号错（NED 航向顺时针为正） |
| 高度越飞越低 | `hover_thrust` 偏小 → 推力不足，重新标定 |

### 12.5 跟踪精度差

| 现象 | 调整 |
|---|---|
| 跟踪太慢、误差大 | 增 `kp` |
| 振荡 | 减 `kp` 或增 `kv` |
| 稳态有固定偏差 | 增 `ki`（注意积分只在 `\|e_v\| < 0.6` 时生效） |
| 饱和标志频繁置位 | 降 `max_acceleration_mps2`，或增 `hover_thrust`（留推力余量） |
| 交接瞬间有冲击 | 增 `tracking_handoff_sec`；注意交接时加速度前馈本来就被淡出了 |

### 12.6 PX4 不响应设定值

```bash
ros2 topic info -v /fmu/in/vehicle_attitude_setpoint_v1
```

- 没有订阅者 → 话题名不对（`_v1` 后缀），或 DDS 没连上
- 有订阅者但不执行 → PX4 可能拒绝了，检查 `VehicleStatus.failsafe` 和 PX4 事件日志

---

## 13. 与 eight 试验流程集成

`eight` 是本工作区的裸 X500 八字试验包，是 px4ctrl 唯一的使用者。
它演示了 px4ctrl 的**标准接入方式**：

```
阶段 A：eight 主导（位置 Offboard）
   eight → /fmu/in/trajectory_setpoint
   → PX4 自己的位置环算姿态，爬升到 2 m
   → 到达后发 /eight/takeover_allowed = true

阶段 B：px4ctrl 主导（姿态 Offboard）
   px4ctrl 抓取当前悬停点 → 开始发 /fmu/in/vehicle_attitude_setpoint_v1
   eight 停止发位置设定值（避免抢 Offboard 控制模式）
   → px4ctrl 开始发 /px4ctrl/desired_state（八字轨迹，NED）
   → px4ctrl 跟踪
```

**任一时刻只有一个节点在给 PX4 发控制指令。**

四终端流程见 `../eight/README.md`。简要：

```bash
# 终端 1：裸 X500 仿真
cd /home/zone/PX4-Autopilot && make px4_sitl gz_x500

# 终端 2：DDS Agent
MicroXRCEAgent udp4 -p 8888

# 终端 3：起飞 + 交接（保持运行直到降落结束）
ros2 launch eight takeoff.launch.py
#   看到 "Offboard ready; keep this launch running" 后再开终端 4

# 终端 4：发八字 + 评估
ros2 launch eight eight.launch.py output_dir:=/tmp/eight_runs
```

> **不需要 QGC。** `eight/sitl_gcs.py` 提供了 MAVLink 心跳和 PX4 的
> `COM_RCL_EXCEPT` Offboard 例外位。**该脚本只用于无遥控器的裸 X500 SITL，
> 实机绝对不要用。**

---

## 14. 从旧版本迁移

### 14.1 已删除的参数

| 旧参数 | 现在的做法 |
|---|---|
| `planner_frame` | `reference_frame`，且参考量必须已在 NED |
| `transform_rotation_row_major` | 参考源自行换算到 NED |
| `align_to_px4_local_origin` | 不再需要原点对齐 |
| `frame_alignment_velocity_tolerance_mps` | 同上 |
| `odom_topic` | 控制器不再订阅任何里程计 |
| `state_timeout_sec` | 同上 |

**只要配置里还有这些参数，节点就会以致命错误拒绝启动**：

```
[FATAL] parameter 'planner_frame' was removed: the reference must already be
expressed in PX4 local NED. Use 'reference_frame' for the frame id and remove 'planner_frame'.
```

这是**刻意设计**的。ROS 2 默认会**静默忽略**未声明的参数——如果只是把参数
删掉不检查，旧配置会"看起来在跑"，实际把 lidar 系的参考当 NED 解释，
飞向完全错误的方向。宁可启动失败，也不要静默误用。

### 14.2 迁移步骤

1. 从 `config/px4ctrl_default.yaml` 复制一份。
2. 确认参考源发布的 `DesiredState` **已经在 PX4 本地 NED**，
   `header.frame_id` 设为 `px4_local_ned`。
3. 删掉配置里所有 §14.1 列出的旧参数。
4. 标定 `hover_thrust`（§10）。
5. 跑起来看 `/px4ctrl/status`。

### 14.3 已删除的文件

`scripts/ego_position_command_adapter.py`、`scripts/manual_goal.py`、
`scripts/track_eval.py`、`launch/bringup.launch.py`、`config/px4ctrl_sitl.yaml`。

若要重新接入 EGO 之类的规划器，需要自己写一个转换节点，把规划器的输出
换算到 NED 后发布到 `/px4ctrl/desired_state`——换算和原点对齐由那个节点负责，
控制器不再参与。

---

## 15. 深入阅读

| 文档 | 内容 |
|---|---|
| [`LEARNING_GUIDE_CN.md`](LEARNING_GUIDE_CN.md) | **学习文稿**：从零到能改控制律，7 个里程碑 + 6 个实验 + 自测题库 |
| [`MIGRATION_GUIDE.md`](MIGRATION_GUIDE.md) | 中文迁移与接口指南（坐标约定、话题表、接入模式、迁移检查清单） |
| [`READING_GUIDE_CN.md`](READING_GUIDE_CN.md) | 源码逐行精读（数据结构、控制律推导、每个参数对应的代码位置） |
| [`../eight/CHAIN_X500_CN.md`](../eight/CHAIN_X500_CN.md) | eight 流程下的完整链路时序与排障 |

**建议的阅读顺序**：本文 §1–§3（理解契约）→ §7（理解控制律）→
`READING_GUIDE_CN.md` §5.6（逐行看 `calculate_output`）→ §10（标定）。
