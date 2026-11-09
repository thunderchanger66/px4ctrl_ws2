# px4ctrl 源码通读指南（从零开始）

> 面向"第一次接触 PX4 外部控制器 / ROS 2 话题 / 姿态控制"的同学。
> 全文按"数据从哪来 → 怎么变 → 怎么发给 PX4"的顺序讲，所有结论都能在
> `src/px4ctrl_node.cpp` 里按行号找到对应代码。

---

## 0. 一句话理解这个包在干什么

```
EGO 规划器（算 B 样条轨迹）
        │  /drone_0_planning/pos_cmd   (quadrotor_msgs/PositionCommand)
        ▼
ego_position_command_adapter（格式翻译）
        │  /px4ctrl/desired_state       (px4ctrl/DesiredState)
        ▼
px4ctrl_node（本包核心：把"位置轨迹"翻译成"姿态+推力"）
        │  /fmu/in/vehicle_attitude_setpoint_v1
        ▼
PX4 飞控（只执行姿态，不做位置控制）
        ▲
        │  /fmu/out/vehicle_local_position_v1 （反馈当前位姿）
        │
FAST-LIO（激光里程计）─ /fastlio2/lio_odom ─▶ 规划器 + px4ctrl（坐标系对齐）
```

**关键点：PX4 的位置控制环被完全绕过了。**
本控制器直接算"机体应该摆成什么姿态、油门给多少"，然后以 Offboard 姿态模式
（`OffboardControlMode.attitude = true`）塞给 PX4。PX4 只负责把这套姿态指令
变成电机转速。位置误差、速度误差全部由本节点自己消化。

这就是 README 里说的 `bypassing PX4's position controller`。

---

## 1. 坐标系扫盲（不懂这块，后面全乱）

| 坐标系 | 约定 | 谁在用 |
|---|---|---|
| **ENU** | x 东、y 北、z 天 | ROS/ROS2 通用，但本项目不用 |
| **NED** | x 北、y 东、z **下** | **PX4 的一切**：local position、attitude setpoint、里程计 |
| **lidar** | FAST-LIO 的里程计/点云坐标系，本项目当作"规划系" | EGO 规划器、`/fastlio2/lio_odom`、`DesiredState` |
| **body / FRD** | x 前、y 右、z 下 | PX4 姿态解算、`VehicleAttitudeSetpoint` |

**NED 的 z 是朝下的**，所以：
- 高度越高 → z 越小（负值）
- `takeoff_target_.z() -= auto_takeoff_altitude_m_` 是在"抬高"
- 代码里到处出现 `z > minimum_z` 的判断，注释特意提醒 "In NED, smaller z means higher altitude"

**lidar → NED 的换算**（`transform_rotation_row_major`）：

```
[ 0  1  0 ]        NED_x = lidar_y
[ 1  0  0 ]   R    NED_y = lidar_x
[ 0  0 -1 ]        NED_z = -lidar_z
```

注意 x/y 被**交换**了，这会让偏航角正负号翻转（yaw 从 lidar 的"相对 +x 逆时针为正"
变成 NED 的"相对 +x 顺时针为正"）。所以代码里 yaw 不是简单乘 R，而是先把
yaw 变成方向向量再转（见 §5.6）。

---

## 2. 话题全景表

### 2.1 输入（订阅）

| 话题 | 类型 | 来源 | 代码位置 | 作用 |
|---|---|---|---|---|
| `/px4ctrl/desired_state` | `px4ctrl/DesiredState` | `ego_position_command_adapter` | L360 `command_callback` | **规划器轨迹参考**（位置/速度/加速度/偏航） |
| `/fastlio2/lio_odom` | `nav_msgs/Odometry` | FAST-LIO2 | L414 `odom_callback` | 仅用于**坐标系对齐**（算平移量） |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | PX4 EKF2 | L480 `local_position_callback` | **主反馈**（p, v, heading）+ EKF 重置补偿 |
| `/fmu/out/vehicle_status_v4` | `px4_msgs/VehicleStatus` | PX4 | L442 | 是否有 arm / 是否 offboard / failsafe |
| `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | PX4 | L450 | 只用来刷新 PX4 时间戳 |
| `/fmu/out/vehicle_land_detected` | `px4_msgs/VehicleLandDetected` | PX4 | L318 | 空中接管模式判断"是否真的离地" |
| `/eight/takeover_allowed` | `std_msgs/Bool` | `eight` 包 | L325 | 人工批准接管许可 |
| `/planning/emergency_stop` | `std_msgs/Empty` | EGO / 桥接 | L328 / L588 | 规划器急停 → 原地悬停 |
| `/px4ctrl/manual_yaw` | `std_msgs/Float64` | 运维脚本 | L331 | 人工覆盖偏航角（弧度，规划系） |
| `/px4ctrl/clear_manual_yaw` | `std_msgs/Empty` | 运维脚本 | L336 | 取消人工偏航 |

### 2.2 输出（发布）

| 话题 | 类型 | 去向 | 作用 |
|---|---|---|---|
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | PX4 | 声明"我要用姿态 Offboard" |
| `/fmu/in/vehicle_attitude_setpoint_v1` | `px4_msgs/VehicleAttitudeSetpoint` | PX4 | **核心输出**：目标四元数 + 推力 + 偏航角速度 |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | PX4 | 切 Offboard / 解锁 / 降落 |
| `/px4ctrl/status` | `std_msgs/String` | 人类 / `eight` 包 | 状态机字符串（见 §7） |
| `/px4ctrl/debug` | `px4ctrl/ControlDebug` | rviz / 调参 | 误差、饱和标志、推力、倾角 |

> **为什么 topic 带 `_v1` 后缀？** PX4 1.x 的 uORB 桥接会给输入话题加版本号，
> 发到不存在的名字 PX4 根本没有订阅者，指令会被静默丢弃。详见 L143 注释。

---

## 3. 目录结构与文件职责

```
src/px4ctrl/
├── src/px4ctrl_node.cpp        ← 1328 行，全部控制逻辑（本指南主角）
├── msg/DesiredState.msg        ← 规划器 → 控制器的参考消息
├── msg/ControlDebug.msg        ← 控制器 → 人类的调试消息
├── config/px4ctrl_sitl.yaml    ← 带 EGO 规划器的仿真参数（主用）
├── config/px4ctrl_x500.yaml    ← 裸 X500 空中接管参数
├── launch/bringup.launch.py    ← 起 EGO + 适配器 + 里程计桥 + px4ctrl + XRCE Agent
├── launch/controller_only.launch.py ← 只起 px4ctrl（QGC 飞过的机子）
├── scripts/ego_position_command_adapter.py ← EGO 消息 → DesiredState
├── scripts/manual_goal.py      ← 手动发一个目标点
├── scripts/offboard_thrust_probe.py ← 标定 hover_thrust 用
├── scripts/track_eval.py       ← 轨迹跟踪精度评估
└── test/test_px4ctrl_integration.py ← 集成测试
```

外部依赖（同一个 workspace 里的其它包）：
- `ego`（EGO planner）→ 产生 B 样条 → `traj_server` → `PositionCommand`
- `fastlio2_px4_odom_bridge` → 把 FAST-LIO 里程计喂给 PX4 EKF2（视觉/激光外部定位）
- `eight` → 高层任务状态机，负责"起飞 → 允许接管 → 评估"

---

## 4. 消息定义

### `msg/DesiredState.msg`
```
std_msgs/Header header        # frame_id 必须是 planner_frame（"lidar"）
geometry_msgs/Point position  # 规划系位置（m）
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float64 yaw                   # 偏航角（rad，规划系）
float64 yaw_rate
```
> 注意：**这个消息里的量全在"规划系"，不是 NED。** 转到 NED 是控制器干的活。

### `msg/ControlDebug.msg`
位置误差、速度误差、速度参考、加速度请求/可行值、推力、倾角、偏航角速度、
指令延迟、**饱和标志位**、local_valid / status_fresh / armed / offboard / failsafe。

`saturation_flags` 位定义（见 `calculate_output`）：

| 位 | 含义 | 触发条件 |
|---|---|---|
| 1 | 加速度被限幅 | `|a| > max_acceleration_mps2` |
| 2 | 垂直推力被限幅 | 需要推力超出 `[min_thrust, max_thrust]` 对应的力 |
| 4 | 水平加速度被限幅 | 超出倾角/推力所能提供的水平力 |
| 8 | 最终推力被 clamp | 撞到 `min_thrust` / `max_thrust` |

---

## 5. 源码逐段精读

### 5.1 文件头与数据结构（L1–L113）

```cpp
struct DesiredState { Vector3 position, velocity, acceleration; double yaw, yaw_rate; };
struct ControlOutput { Eigen::Quaterniond attitude; double thrust, yaw_rate, tilt_deg;
                       uint8_t saturation_flags; Vector3 position_error, ... };
```
纯数据结构，无逻辑。

**`limit_body_z_tilt()`（L65）**
把"机体 z 轴（朝下）方向"限制在最大倾角内。做法：算 body_z 与世界 z 的夹角，
超过 `max_tilt_rad` 就沿着"到 z 轴的垂足方向"旋回去。等价于 PX4 的 `limitTilt()`。

**`body_z_and_yaw_to_attitude()`（L84）**
已知"机体 z 轴朝向" + "航向角"，构造完整姿态四元数。
- 先按偏航角造一个水平参考向量 `y_c = (-sin yaw, cos yaw, 0)`
- `body_x = y_c × body_z`（保证机体 x 与 z 垂直、与航向一致）
- `body_y = body_z × body_x`
- 三列拼成旋转矩阵 → 四元数

> 这与 PX4 `ControlMath::thrustToAttitude()` 是同一套几何，所以外部姿态设定点
> 和 PX4 自己位置环生成的几何完全一致（注释 L61–63 特意说明）。

**`ControlState` 枚举（L118）**
```
waiting_for_command → takeoff → hover → tracking
                                   ↘ safety_fault (latch，不可恢复)
```

### 5.2 构造函数：参数声明 + 校验（L130–L362）

**L130–L186**：61 个 `declare_parameter`，全部有默认值。
**L188–L235**：`get_parameter` 读出来存到成员变量。
**L236–L300**：一大坨 `if (...) throw std::runtime_error("invalid px4ctrl parameters")`。
这是**参数合法性校验**：维度对不对、是否 finite、范围是否合理
（`hover_thrust ∈ (min_thrust, max_thrust)`、`max_tilt < π/2`、
`minimum_takeoff_altitude ≤ auto_takeoff_altitude` …）。启动时校验不过直接退出，
避免带着错误参数飞。

**L301–L306**：`transform_rotation_row_major` 必须是**真旋转矩阵**
（`R ≈ Rᵀ⁻¹` 且 `|det(R) − 1| < 1e-4`），否则报错。

**L307–L352**：建订阅/发布。注意 QoS：
- PX4 话题用 `KeepLast(1).best_effort()`（uXRCE-DDS 惯例）
- FAST-LIO 用 `KeepLast(20).best_effort()`（传感器流，兼容 RELIABLE 和 SENSOR_DATA）
- 指令订阅用 `QoS(20)`（默认 RELIABLE）

**L353–L361**：建 100 Hz 定时器 `timer_callback`。

### 5.3 `command_callback`：接收规划器轨迹（L360–L412）

关键动作，按顺序：

1. **frame 校验**：`msg->header.frame_id != planner_frame_` → 直接丢弃。
2. **时戳校验**（L365–L372）：
   - `stamp.nanoseconds() <= 0` → 丢
   - `age > command_timeout_sec` → 太旧，丢
   - `age < -0.05` → 来自未来，丢
   - `stamp <= last_desired_stamp_` → 乱序/重复，丢
3. **非有限值校验**（L376–L381）：NaN/Inf 丢。
4. **跳变检测**（L385–L402）：
   ```
   position_jump = |p_new − p_prev| > max_command_position_jump_m (0.35)
   velocity_jump = |v_new − v_prev| > max_command_velocity_jump_mps (1.5)
   ```
   正常情况下 EGO 的重规划是**状态连续**的，位置不会突变。突变说明规划器
   复位/重规划时给了个新起点。此时启动 **command_blend**（平滑过渡），
   而不是把阶跃直接送进姿态环。
5. 存 `desired_` / `reference_target_` / `pending_desired_`，更新时间戳，
   `command_seen_ = true`。

> **三个 desired 变量的区别**（容易混）：
> - `reference_target_`：**原始**收到的规划器指令（用于算跳变）
> - `desired_`：当前**正在施加**给控制器的参考（可能经过 blend / handoff / 状态机改写）
> - `pending_desired_`：最新一条指令，等状态机允许时（如 hover 稳定后）才切到 tracking

### 5.4 `odom_callback`：接收 FAST-LIO（L414–L440）

- frame 必须等于 `planner_frame_`（"lidar"），否则丢弃
- `twist` 是**机体系**速度 → 用姿态四元数旋转到世界系：`v = q * body_v`
- 只存位置/速度/时间，**不参与反馈控制**

> **为什么 FAST-LIO 不直接做反馈？** 因为 FAST-LIO 有绝对漂移，而 PX4 的 EKF2
> 已经在融合 FAST-LIO（通过 `fastlio2_px4_odom_bridge` 喂
> `/fmu/in/vehicle_visual_odometry`），PX4 的 local position 更稳。
> 见 `state_fresh()`（L610–L619）的注释：对齐之后 FAST-LIO 就不再是控制状态源了。

### 5.5 `local_position_callback`：主反馈 + EKF 重置补偿（L480–L535）

这是**最重要的反馈入口**。

**重置检测**：PX4 的 `VehicleLocalPosition` 带三个计数器
`xy_reset_counter` / `z_reset_counter` / `heading_reset_counter`。
EKF2 重新收敛时会自增，表示"局部坐标系原点被重定向了"。

**补偿逻辑**（L496–L528），模仿 PX4 自己位置控制器的做法：
```
translation_ += delta_xy / delta_z      // 平移量补偿到所有目标点
hover_position_ += ...
takeoff_target_ += ...
tracking_handoff_position_ += ...
heading_reset_offset_ += delta_heading  // 航向补偿
hold_yaw_ += delta_heading
yaw_setpoint_ += delta_heading
```
- 补偿量超过 `max_ekf_reset_delta_m` / `max_ekf_reset_heading_rad`
  → `latch_safety_fault("px4_local_position_reset_too_large")`（不可恢复）
- 正常范围内 → 打日志 "Compensating PX4 local reset"

> **为什么要补偿？** EKF2 把原点挪了 5 cm，你给 PX4 的目标点还是老坐标系下的，
> 于是飞机会往偏 5 cm。补偿掉这个差值就等价于"目标点跟着一起挪"。

### 5.6 `calculate_output()`：**控制律核心**（L656–L854）

一步步拆：

#### (a) 取状态、混合参考（L657–L664）
```cpp
p = [lpos.x, lpos.y, lpos.z]      // NED 当前位置
v = [lpos.vx, lpos.vy, lpos.vz]   // NED 当前速度
d = desired_.value_or(zero)       // 目标参考（规划系）
```

#### (b) Replan 平滑（L666–L684）
若 `reference_blending_`：
```cpp
u = elapsed / command_blend_sec
blend = u²(3−2u)                  // smoothstep
d.position = (1−blend)*from.position + blend*d.position
d.yaw     = from.yaw + blend*remainder(d.yaw − from.yaw, 2π)   // 走最短弧
```
> **为什么在坐标变换之前混合？** 注释 L666–670 解释：如果先变到 NED 再混合，
> 就会拿"上一次的规划系状态"和"这次的 NED 状态"混在一起，坐标轴换了会造出
> 假的阶跃。**先混（都在规划系），再变换。**

#### (c) 规划系 → NED（L686–L688）
```cpp
d.position = rotation_ * d.position + translation_;
d.velocity = rotation_ * d.velocity;
d.acceleration = rotation_ * d.acceleration;
```
`translation_` 来自 `align_frames_if_stationary()`。

#### (d) 最低飞行高度钳位（L689–L709）
只有 `auto_takeoff` 模式且在 tracking 时生效。
若规划器给的 z **低于**安全下限（NED 里 z 更大 = 更低）：
```cpp
d.position.z = min(p.z(), minimum_z)   // 保持当前高度，不下降
d.velocity.z = 0; d.acceleration.z = 0;
```
> 规划器失败重规划时可能给出"地面以下"的轨迹，这时**不要**跟着往下钻。

#### (e) 偏航角变换（L710–L722）—— 最绕的一段
```cpp
heading_in_ned = rotation_ * [cos(yaw), sin(yaw), 0]
heading_rate_in_ned = rotation_ * [-sin(yaw)*yaw_rate, cos(yaw)*yaw_rate, 0]
d.yaw = atan2(heading_in_ned.y(), heading_in_ned.x())
d.yaw = remainder(d.yaw + heading_reset_offset_, 2π)    // 叠加 EKF 航向重置补偿
d.yaw_rate = (heading_in_ned.x()*heading_rate_in_ned.y() − heading_in_ned.y()*heading_rate_in_ned.x())
              / |heading_in_ned|²
```
**为什么这么写？** 因为 R 把 x/y 交换了，偏航角的**正方向**也反了。
把"角度"转成"单位方向向量"→ 旋转 → 再转回角度，就自动处理了正负号。
偏航角速度用二维叉积公式 `dθ/dt = (a_x b_y − a_y b_x)/|a|²`。

#### (f) 指令失效 / 交接平滑（L723–L735）
- 指令超时（`command_fresh()` 为假）→ 速度、加速度、偏航角速度**全部清零**
  （只保留位置目标，形成纯位置保持）
- tracking 刚切换时 → 用 `tracking_handoff_sec` 做 smoothstep，
  把参考从"当前实测位姿"平滑过渡到"规划器目标"，**并把加速度乘以 blend**
  （L720 特别注明 `d.acceleration *= blend`）。

> 为什么交接时要把加速度也淡出？因为刚进 tracking 时位置/速度误差可能很大，
> 加上规划器给的加速度前馈会产生很大冲击。淡出加速度 = 更保守。

#### (g) 偏航角速度伺服（L736–L762）
```cpp
yaw_target = tracking ? d.yaw : hold_yaw_        // 起飞/悬停阶段保持起飞航向
if (tracking && manual_yaw_) yaw_target = 人工偏航

yaw_error = remainder(yaw_target − yaw_setpoint_, 2π)
yaw_feedforward = (tracking && !manual_yaw_) ? d.yaw_rate : 0
desired_yaw_rate = clamp(yaw_feedforward + 2*yaw_error, ±max_yaw_rate_radps)
yaw_rate_setpoint += clamp(desired − current, ±max_yaw_accel*dt)   // 限加速度
yaw_setpoint += yaw_rate_setpoint * dt
```
这是**一个带限幅和限加速度的偏航角 PI 环**。两个作用：
- 规划器要求的大角度转身（如 180°）不会一步到位，而是以 ≤0.6 rad/s 转过去
- 速率也有加速度限制，避免抖动。

#### (h) 级联 PID（L763–L795）
**这是理解本控制器最关键的几行。**

```cpp
// 1) 位置误差 → 速度参考
velocity_reference = d.velocity;
for (i) velocity_reference(i) += kp_(i) / kv_(i) * position_error(i);

// 2) 速度误差
velocity_error = velocity_reference − v;

// 3) 积分（条件性）
integral_enabled = tracking && command_fresh()
                   && |velocity_error| ≤ integral_enable_velocity_error_mps (0.6)
if (!integral_enabled || last_output_saturated_) integral *= 0.995;   // 衰减
else integral += ki ⊙ velocity_error * dt;  再 clamp 到 ±integral_limit

// 4) 加速度指令
acceleration = d.acceleration            // 规划器前馈
             + kv_ ⊙ velocity_error       // 速度反馈
             + integral_term;            // 积分补偿
for (i) if (kv_(i) <= 1e-9) acceleration(i) += kp_(i) * position_error(i);
```

**结构是"串级"（cascade）**：
```
位置误差 --kp/kv--> 速度参考 --(减去实测速度)--> 速度误差 --kv--> 加速度指令
```
等价于把"位置环增益"折算进"速度环"。代码注释 L764–766 说得很直白：

> kp is the historical acceleration-per-position gain. Divide by kv so existing
> YAML preserves its proportional acceleration contribution.

即 `kp` 的物理含义是 **加速度 / 位置误差**（单位 m/s² per m），
`kv` 的含义是 **加速度 / 速度误差**（单位 m/s² per m/s）。
默认 `kp=[1.5,1.5,1.8]`, `kv=[1.2,1.2,1.4]` → 位置环等效增益
`kp/kv = [1.25, 1.25, 1.286]`。

**积分的两个保护**：
1. 只有速度误差小（<0.6 m/s）才积分 → 防止大误差时积分饱和
2. 一旦输出饱和（saturation_flags ≠ 0），积分按 0.995 衰减 → 抗积分饱和（anti-windup）

#### (i) 加速度限幅（L796–L801）
`|a| > max_acceleration_mps2 (4.0)` → 等比缩放，置 flag 1。

#### (j) 投影到"推力 + 倾角"可行域（L803–L851）
这是本控制器最有价值的设计。**不要先算推力再硬夹**，而是把加速度投影到
物理可达的集合里：

```cpp
// 力 → 归一化推力的换算基准
max_force = g * max_thrust / hover_thrust      // 0.95 推力对应的力
min_force = g * min_thrust / hover_thrust

// 垂直方向
requested_vertical_force = g − a_z              // a_z 负 = 上升 → 需要更大力
vertical_force = clamp(requested_vertical_force, min_force, max_force)
a_z = g − vertical_force

// 水平方向：两条约束取较小者
horizontal_force_from_tilt   = tan(max_tilt) * vertical_force   // 倾角约束
horizontal_force_from_thrust = sqrt(max_force² − vertical_force²) // 推力余量约束
max_horizontal_accel = min(两者)
if (|a_xy| > max_horizontal_accel) 缩放，置 flag 4
```

**为什么重要？** 假设你要求 4 m/s² 的水平加速，而推力只剩 3% 余量。
天真做法：先算推力→clamp→得到一个很小的推力→但姿态还朝着 4 m/s² 的方向倾斜
→ 结果是"用力气往一个方向顶"，产生巨大超调。这里改成：先把水平加速度降到
可达范围，**再**由这个可达的加速度反推姿态和推力 → 结论一致，不超调。

注释 L807–810 明确说了这个动机：
> Hard-clipping collective thrust after attitude generation keeps an unreachable
> acceleration target in the loop and causes large overshoot when hover thrust
> is close to the configured maximum.

#### (k) 力 → 姿态 + 推力（L816–L850）
```cpp
force = acceleration − [0,0,g]          // 与 PX4 位置控制器同一个表达式
body_z = limit_body_z_tilt(−force, max_tilt_rad)
force_down = max(0.1, g − a_z)
thrust = hover_thrust * force_down / (g * body_z_down)
thrust = clamp(thrust, min_thrust, max_thrust)
```
自洽性检查（hover）：`a = 0` → `force = [0,0,−g]` → `body_z = [0,0,1]`
→ `thrust = hover_thrust * g / (g*1) = hover_thrust` ✔

> `hover_thrust` 是**实测标定值**：`config/px4ctrl_sitl.yaml` 里写 0.864，
> 注释说 "Measured equilibrium output for X500 + gimbal + MID-360 in SITL"。
> 意思是带云台和激光雷达的机器人在 SITL 里悬停需要 86.4% 的油门。
> 这是 `scripts/offboard_thrust_probe.py` 标定出来的。

### 5.7 状态机（L856–L967）

**`begin_takeoff()`（L856）**
- 记录当前点为 `hover_position_`（起飞平面）
- `hold_yaw_` = 当前航向
- `takeoff_target_ = hover_position_`，然后 `z -= auto_takeoff_altitude_m`（NED z 减小 = 抬高）
- 状态 → takeoff（若目标高度为 0 则直接 hover）

**`begin_airborne_takeover()`（L872）**——`px4ctrl_x500.yaml` 用这条路径
必须同时满足：
1. `local_position_healthy()`（EKF 状态有效）
2. 已 arm、**不是 failsafe**
3. `land_detected` 话题新鲜、且 `landed/ground_contact/maybe_landed` **全为 false**
   （即确认在空中）
4. 速度 < `takeover_velocity_tolerance_mps` (0.20)
5. 连续稳定 `max(1.0, hover_settle_sec)` 秒
6. `require_takeover_permission` 为真时，还需要 `/eight/takeover_allowed == true`

满足后：抓住当前位置作为悬停点，yaw 对齐当前航向，进入 hover。

**`update_state_machine()`（L913）**
```
waiting_for_command
   ├─ airborne_takeover 模式 → begin_airborne_takeover()
   └─ 收到过指令(airborne 模式) → begin_takeoff()
takeoff
   → 满足 (到达目标高度 OR 至少爬升 minimum_takeoff_altitude)
     AND 速度 < hover_velocity_tolerance
     AND 已过 takeoff_duration_sec
   → hover
hover
   → 速度超限则重置计时；连续稳定 hover_settle_sec 后
     + 有新鲜指令 → tracking
   → 指令超时 → 回到 hover（这是"轨迹没了就原地保持"）
tracking
   → 指令超时 → hover
```

> **airborne_takeover 模式有个特别之处**：L948–L951 允许它在 hover 状态下
> **即使还没进 offboard** 就切到 tracking（因为此时 QGC 还在飞，不能抢）。
> 一旦切到 tracking 就会发 setpoint，进而触发 offboard 请求。

### 5.8 `timer_callback`：每 10 ms 的主循环（L969–L1121）

顺序（**任何一步不满足就 return，不发 setpoint**）：

| 顺序 | 检查 | 不满足时 |
|---|---|---|
| 1 | `safety_latched_` | 直接返回，只报状态 |
| 2 | `control_suspended_` | 返回 |
| 3 | `vehicle_status_fresh()` (≤1.5 s) | 挂起控制 `px4_status_timeout` |
| 4 | `vehicle_status_->failsafe` | 挂起 `px4_failsafe` |
| 5 | `offboard_seen_ && !vehicle_offboard()` | 挂起 `offboard_lost`（**Offboard 掉了就停**） |
| 6 | airborne 模式下掉 arm | 挂起 `vehicle_disarmed` |
| 7 | `state_fresh()` | 等待；若已 arm 超过 `local_position_fault_delay_sec` → 安全故障 |
| 8 | `local_position_healthy()` | 同上 |
| 9 | `align_frames_if_stationary()` | 等待静止对齐 |
| 10 | `update_state_machine()` | 推进状态机 |
| 11 | 还在 waiting | 返回，不发任何 setpoint |

然后进入正常输出：
```cpp
mode.timestamp = current_px4_timestamp();
mode.attitude = true;                       // 姿态 Offboard
mode_pub_->publish(mode);

if (hover/takeoff) { desired_ = 保持点（先转到规划系再算，见 L1055–L1065） }
if (tracking)     { desired_ = pending_desired_; }

output = calculate_output();
setpoint.q_d        = output.attitude;
setpoint.thrust_body = {0, 0, −output.thrust};   // 注意负号：body z 朝下
setpoint.yaw_sp_move_rate = output.yaw_rate;
setpoint_pub_->publish(setpoint);
```

**最后的"切 Offboard / 解锁"逻辑（L1090–L1118）**（仅 `auto_takeoff` 模式）
```cpp
if (!command_seen_ || !start_offboard_ || !vehicle_status_) return;
if (elapsed < prestream_seconds_) return;              // 先"预热"1秒发 setpoint

if (nav_state != OFFBOARD) {
   发送 VEHICLE_CMD_DO_SET_MODE，param1=1.0, param2=6.0   // 见下方说明
}
if (未 arm && arm_on_first_ && pre_flight_checks_pass) {
   发送 VEHICLE_CMD_COMPONENT_ARM_DISARM (param1=1)
}
```
> **为什么 `prestream_seconds`？** PX4 要求进入 Offboard 前就已经在发 setpoint，
> 否则会因为"没有 setpoint"而拒绝。必须先刷 1 秒的 setpoint 再请求切模式。
> `retry_due()` 保证同一命令最多 1 秒重发一次。

> **关于 `param1=1, param2=6`**：这是 PX4 官方 ROS 2 offboard 示例里的固定写法
> （`param1=1` 表示"自定义模式"，`param2=6` 是 PX4 的 Offboard 自定义模式号）。
> 本仓库的 `eight` 包（L382）用的是完全一样的参数组合，所以两边行为一致。
> **如果你换 PX4 版本，请以该版本的 `vehicle_command.cpp` / 官方示例为准再核对一次。**

### 5.9 工具函数（尾部）

- `control_state_name()`（L1123）：状态枚举 → 字符串
- `retry_due()`（L1135）：1 秒节流
- `publish_vehicle_command()`（L1140）：填 `VehicleCommand`，`from_external = true`
- `publish_status()`（L1156）
- `publish_debug()`（L1163）：填 `ControlDebug`，`frame_id = "px4_local_ned"`

---

## 6. 数学总结（一张图）

```
规划器参考 (p_d, v_d, a_d, yaw_d)        ← EGO B 样条，lidar 系
        │
        │ ① 坐标系变换  p_ned = R·p_lidar + t
        │ ② 交接/混合平滑
        │ ③ 最低高度钳位
        ▼
   (p_d, v_d, a_d)  in NED
        │
        │ ④ 串级 PID
        │    v_ref = v_d + (kp/kv)·(p_d − p)
        │    e_v   = v_ref − v
        │    a_cmd = a_d + kv·e_v + ∫ki·e_v dt
        ▼
   加速度指令 a_cmd  ──(限幅 + 投影到推力/倾角可行域)──▶  a_feasible
        │
        │ ⑤ force = a_feasible − [0,0,g]
        │ ⑥ body_z = limitTilt(−force, max_tilt)
        │ ⑦ thrust = hover_thrust · force_down / (g · body_z.z)
        │ ⑧ 四元数 = body_z + yaw
        ▼
   VehicleAttitudeSetpoint{q_d, thrust_body=(0,0,−thrust), yaw_sp_move_rate}
        │
        ▼
      PX4 (姿态环 → 角速度 → 电机)
```

---

## 7. 状态机状态字符串（`/px4ctrl/status`）

| 字符串 | 含义 |
|---|---|
| `waiting_for_fastlio_odom_and_planner_command` | 启动时 |
| `waiting_for_armed_stationary_airborne_state` | airborne 模式，等起飞+稳定 |
| `waiting_for_stationary_frame_alignment` | 等两个里程计都静止，做坐标系对齐 |
| `state_takeoff` | 起飞爬升中 |
| `state_hover` / `state_hover_minimum_altitude` | 悬停稳定 |
| `state_tracking` | 跟踪规划器轨迹 |
| `state_hover_command_timeout` | 轨迹断了，回悬停 |
| `tracking_attitude_thrust_tracking` | 正常跟踪中（含相位前缀） |
| `planner_emergency_hold` | 规划器急停 → 原地保持 |
| `safety_fault_<reason>` | **锁存的安全故障，不再输出** |
| `control_suspended_<reason>` | 挂起（offboard 丢失 / failsafe / 掉 arm） |
| `hover_airborne_takeover_prestream` / `_ready` / `_offboard_ready` | 空中接管各阶段 |

---

## 8. 参数完全手册

### 8.1 话题类

| 参数 | 代码默认 | sitl.yaml | x500.yaml | 说明 |
|---|---|---|---|---|
| `planner_frame` | `lidar` | `lidar` | `px4_local_ned` | 规划器参考所在的坐标系名。必须与 EGO `planning_frame` 和 FAST-LIO 的 `header.frame_id` 一致 |
| `desired_state_topic` | `/px4ctrl/desired_state` | 同 | 同 | 轨迹参考输入 |
| `debug_topic` | `/px4ctrl/debug` | *(未设→默认)* | `/px4ctrl/debug` | 调试输出 |
| `odom_topic` | `/fastlio2/lio_odom` | 同 | *(未设→默认)* | FAST-LIO 里程计（**仅用于对齐**） |
| `px4_status_topic` | `/fmu/out/vehicle_status_v4` | 同 | 同 | PX4 状态 |
| `px4_attitude_topic` | `/fmu/out/vehicle_attitude` | 同 | 同 | 只为刷新时间戳 |
| `px4_local_position_topic` | `/fmu/out/vehicle_local_position_v1` | 同 | 同 | **主反馈** |
| `px4_land_detected_topic` | `/fmu/out/vehicle_land_detected` | *(未设→默认)* | *(未设→默认)* | 空中接管判定落地 |
| `offboard_control_mode_topic` | `/fmu/in/offboard_control_mode` | 同 | 同 | Offboard 模式声明 |
| `attitude_setpoint_topic` | `/fmu/in/vehicle_attitude_setpoint_v1` | 同 | 同 | **核心输出**（注意 `_v1`） |
| `vehicle_command_topic` | `/fmu/in/vehicle_command` | 同 | 同 | 模式/解锁/降落指令 |
| `status_topic` | `/px4ctrl/status` | 同 | 同 | 状态字符串 |
| `manual_yaw_topic` | `/px4ctrl/manual_yaw` | 同 | *(未设→默认)* | 人工偏航覆盖 |
| `clear_manual_yaw_topic` | `/px4ctrl/clear_manual_yaw` | 同 | *(未设→默认)* | 取消人工偏航 |
| `takeover_permission_topic` | `/eight/takeover_allowed` | *(未设→默认)* | *(未设→默认)* | 人工接管许可 |

### 8.2 时序 / 超时类

| 参数 | 默认 | sitl | x500 | 说明 |
|---|---|---|---|---|
| `publish_rate_hz` | 100.0 | 100 | 100 | 控制环频率。SITL 用 Gazebo `/clock`，仿真变慢会一起变慢 |
| `state_timeout_sec` | 0.30 | *(未设)* | *(未设)* | FAST-LIO 里程计超时阈值（**仅在对齐前有效**） |
| `command_timeout_sec` | 0.50 | 0.50 | 0.50 | 规划器指令超时 → 回悬停 |
| `px4_status_timeout_sec` | 1.50 | 1.50 | 1.50 | VehicleStatus 超时（该流在 SITL 频率低） |
| `local_position_timeout_sec` | 0.50 | 0.50 | 0.50 | PX4 位置反馈超时 → 安全故障 |
| `land_detected_timeout_sec` | 2.0 | *(未设)* | 2.0 | 落地检测话题超时（1 Hz 发布） |
| `prestream_seconds` | 1.0 | 1.0 | 1.0 | 切 Offboard 前预发 setpoint 的时长 |
| `local_position_fault_delay_sec` | 1.00 | 1.00 | 1.0 | 持续异常多久才判安全故障（防抖） |
| `command_blend_sec` | 0.25 | 0.25 | 0.25 | 轨迹跳变时的平滑时长 |
| `tracking_handoff_sec` | 0.75 | 0.75 | 0.75 | hover→tracking 交接平滑时长 |
| `takeover_velocity_tolerance_mps` | 0.20 | *(未设)* | 0.20 | 空中接管速度门限 |
| `frame_alignment_velocity_tolerance_mps` | 0.20 | 0.20 | *(未设)* | 对齐要求的静止门限 |

### 8.3 开关类（bool）

| 参数 | 默认 | sitl | x500 | 说明 |
|---|---|---|---|---|
| `entry_mode` | `auto_takeoff` | *(未设→默认)* | `airborne_takeover` | `auto_takeoff`=自己起飞；`airborne_takeover`=接管已在空中的机子 |
| `start_offboard_on_first_command` | true | true | false | 收到第一条指令后是否请求切 Offboard |
| `arm_on_first_command` | true | true | false | 是否自动解锁 |
| `require_takeover_permission` | false | *(未设→默认)* | true | 是否需要 `/eight/takeover_allowed` 许可 |
| `require_local_position_valid` | true | true | true | 是否强制要求 EKF 位置有效 |
| `align_to_px4_local_origin` | true | true | false | 是否自动把规划系原点对齐到 PX4 原点 |
| `land_on_safety_fault` | true | true | true | 安全故障时是否下发 LAND |

### 8.4 物理 / 推力类

| 参数 | 默认 | sitl | x500 | 说明 |
|---|---|---|---|---|
| `gravity_mps2` | 9.81 | 9.81 | 9.81 | 重力加速度 |
| `hover_thrust` | 0.864 | **0.864** | **0.60** | **悬停时的归一化总推力**（1.0 = 满油门）。必须实测标定！ |
| `min_thrust` | 0.08 | 0.08 | 0.08 | 最小推力（下限，防熄火） |
| `max_thrust` | 0.95 | 0.95 | 0.95 | 最大推力（上限，留余量） |
| `max_tilt_deg` | 30.0 | 30 | 30 | 最大倾角 |
| `max_acceleration_mps2` | 4.0 | 4.0 | 4.0 | 最大加速度指令 |
| `max_ekf_reset_delta_m` | 1.0 | **3.0** | *(未设→1.0)* | EKF 平移重置补偿上限，超过 → 安全故障 |
| `max_ekf_reset_heading_rad` | 0.35 | **3.20** | *(未设→0.35)* | EKF 航向重置补偿上限 |
| `max_local_position_m` | 100.0 | 100 | 100 | 位置合理性上限 |
| `max_local_velocity_mps` | 15.0 | 15 | 15 | 速度合理性上限 |

### 8.5 起飞 / 高度类

| 参数 | 默认 | sitl | x500 | 说明 |
|---|---|---|---|---|
| `auto_takeoff_altitude_m` | 0.50 | 0.50 | 0.0 | 起飞目标高度（0 = 不起飞，直接 hover） |
| `minimum_takeoff_altitude_m` | 0.25 | 0.25 | 0.0 | 判定"已离地"的最小爬升量（补偿标定误差） |
| `minimum_tracking_altitude_m` | 0.25 | 0.25 | 0.0 | 跟踪阶段的最低飞行高度（相对起飞原点） |
| `takeoff_duration_sec` | 3.0 | 3.0 | *(未设→默认)* | 起飞最短持续时间 |
| `takeoff_tolerance_m` | 0.08 | 0.08 | *(未设→默认)* | 到达目标高度的位置容差 |
| `hover_settle_sec` | 0.80 | 0.80 | 0.8 | 悬停稳定判定时间 |
| `hover_velocity_tolerance_mps` | 0.15 | 0.15 | 0.15 | 悬停稳定的速度门限 |

### 8.6 轨迹合理性 / 偏航类

| 参数 | 默认 | sitl | x500 | 说明 |
|---|---|---|---|---|
| `max_command_position_jump_m` | 0.35 | 0.35 | 0.35 | 单帧位置跳变上限，超过 → 触发 blend |
| `max_command_velocity_jump_mps` | 1.5 | 1.5 | 1.5 | 单帧速度跳变上限 |
| `max_yaw_rate_radps` | 0.6 | 0.6 | 0.6 | 偏航角速度上限 |
| `max_yaw_acceleration_radps2` | 0.8 | 0.8 | 0.8 | 偏航角加速度上限 |

### 8.7 PID 类

| 参数 | 默认 | sitl | x500 | 含义 |
|---|---|---|---|---|
| `kp` | `[1.5,1.5,1.8]` | 同 | 同 | **加速度 / 位置误差** 增益 (m/s² per m) |
| `kv` | `[1.2,1.2,1.4]` | 同 | 同 | **加速度 / 速度误差** 增益 (m/s² per m/s) |
| `ki` | `[0.10,0.10,0.15]` | 同 | 同 | 积分增益 (m/s² per m/s per s) |
| `integral_limit_mps2` | `[0.40,0.40,0.50]` | 同 | 同 | 积分项限幅 (m/s²) |
| `integral_enable_velocity_error_mps` | 0.60 | 同 | 同 | 速度误差超过此值就不积分 |

### 8.8 坐标变换类

| 参数 | 默认 | sitl | x500 | 说明 |
|---|---|---|---|---|
| `transform_rotation_row_major` | `[0,1,0, 1,0,0, 0,0,-1]` | 同 | 单位阵 | 规划系 → PX4 NED 的旋转，行优先 9 元素。必须是真旋转矩阵 |

---

## 9. 两个 YAML 的定位与差异

### `config/px4ctrl_sitl.yaml`（**默认，主用**）
- 场景：X500 + 云台 + MID-360，**带 EGO 规划器**，从地面起飞
- `planner_frame: lidar`，`align_to_px4_local_origin: true`（启动时自动对齐）
- `entry_mode` 未设 → 默认 `auto_takeoff`
- `hover_thrust: 0.864`（带载荷实测值）
- `use_sim_time: true`
- 由 `bringup.launch.py` 默认加载

### `config/px4ctrl_x500.yaml`
- 场景：**裸 X500**，已被 QGC 飞在空中，只要接管
- `planner_frame: px4_local_ned` → **规划器直接用 NED**，不需要旋转
- `align_to_px4_local_origin: false`（已知两系重合）
- `entry_mode: airborne_takeover`
- `auto_takeoff_altitude_m: 0` → 不自己起飞
- `start_offboard_on_first_command: false`、`arm_on_first_command: false`
  （**不抢 QGC 的控制权**）
- `require_takeover_permission: true`（等 `eight` 包批准）
- `hover_thrust: 0.60`（裸机初值，需实测）
- **不订阅 odom**（因为不需要对齐）
- 由 `controller_only.launch.py` 默认加载

> **选哪个？** 跑 EGO 仿真 → sitl.yaml。接真实机 / 接管飞行 → x500.yaml。

---

## 10. 外部包如何咬合

### `ego`（EGO planner）
- `ego_replan_fsm` + `bspline_optimizer` → B 样条 → `/drone_0_planning/bspline`
- `traj_server`（`src/ego/ego_planner/src/traj_server.cpp`）采样 B 样条 →
  `/drone_0_planning/pos_cmd`（`quadrotor_msgs/PositionCommand`，100 Hz）
  - `position/velocity/acceleration` = B 样条的 0/1/2 阶导数
  - `yaw` = 看向轨迹前方 1.0 s 处的方向角
  - `yaw_dot` = 偏航角速度（限幅 π rad/s）
  - `frame_id` = `planning_frame`（"lidar"）
  - `trajectory_flag = TRAJECTORY_STATUS_READY` 才算有效

### `scripts/ego_position_command_adapter.py`（**在本包内**）
翻译层：只放行 `TRAJECTORY_STATUS_READY` 且 frame 匹配的指令，
检查所有数值 finite，然后转成 `px4ctrl/DesiredState` 发到 `/px4ctrl/desired_state`。

> **为什么要单独一层？** `quadrotor_msgs/PositionCommand` 里还有 `kx/kv/kf` 增益字段，
> 是给"直接用位置环的飞控"用的。本控制器自己算 PID，不使用它们，
> 所以需要一个最小化、语义明确的中间消息，避免误用。

### `fastlio2_px4_odom_bridge`
把 `/fastlio2/lio_odom` → `/fmu/in/vehicle_visual_odometry`，
喂给 PX4 的 EKF2 做外部定位融合。
同时估计 ROS 时钟与 PX4 时钟的偏移量，保存在 `timestamp_sample` 里。
**它和 px4ctrl 是两条独立的链路**：
- 桥 → PX4 EKF2 → `vehicle_local_position` → **px4ctrl 的反馈**
- FAST-LIO → px4ctrl（直接）→ **仅用于坐标系对齐**

### `eight`
高层任务编排：起飞 → 等 px4ctrl 报告 `hover_airborne_takeover_offboard_ready`
→ 发 `/eight/takeover_allowed` → 跟踪评估。

---

## 11. 调参手册（按顺序）

### 11.1 第一步：标定 `hover_thrust`（最重要）
```bash
ros2 run px4ctrl offboard_thrust_probe
```
在悬停时读 `/px4ctrl/debug` 的 `thrust` 字段（这就是 `hover_thrust` 的实测量）。
把值填回 YAML。注意：**装了云台/激光雷达后这个值会变大**。

### 11.2 第二步：确认坐标系
```bash
ros2 topic echo /fastlio2/lio_odom --once     # frame_id 是不是 "lidar"
ros2 topic echo /px4ctrl/debug --once          # position_error 量级合理？
```
如果 `position_error` 恒定很大且不收敛 → `transform_rotation_row_major` 错了。
常见错误：x/y 没交换（应该 `NED_x = lidar_y`）。

### 11.3 第三步：调 PID
| 现象 | 调整 |
|---|---|
| 跟踪太慢、误差大 | 增 `kp` |
| 振荡 | 减 `kp` 或增 `kv` |
| 稳态有固定偏差 | 增 `ki`（但注意积分只在 \|e_v\|<0.6 时生效） |
| 撞饱和标志频繁 | 降 `max_acceleration_mps2` 或增 `hover_thrust`（留推力余量） |

### 11.4 第四步：安全阈值
| 参数 | 什么时候调 |
|---|---|
| `max_ekf_reset_delta_m` | SITL 常见 EKF 复位幅度较大（sitl.yaml 用 3.0），实机可能需要收紧 |
| `max_ekf_reset_heading_rad` | 同上（sitl.yaml 用 3.20 ≈ 183°） |
| `local_position_fault_delay_sec` | 定位抖动大 → 加大，容忍更久的异常 |

---

## 12. 排障速查

| 现象 | 检查 |
|---|---|
| 完全没 setpoint | `ros2 topic echo /px4ctrl/status --once`。若停在 `waiting_for_stationary_frame_alignment` → 两个里程计没同时静止，或 FAST-LIO 没数据 |
| PX4 不响应 setpoint | `ros2 topic info -v /fmu/in/vehicle_attitude_setpoint_v1` 确认有订阅者；topic 名是否带 `_v1` |
| 一直不切 Offboard | `prestream_seconds` 没满足；或 `VehicleStatus.nav_state` 没变；查 `/px4ctrl/status` 里的 `attitude_offboard_requested` |
| 一动就掉 Offboard | `control_suspended_offboard_lost` → PX4 拒绝姿态命令，通常是姿态指令不可达或 failsafe |
| `safety_fault_px4_local_position_unhealthy` | EKF2 没融合外部视觉/激光。查 `param show EKF2_EV_CTRL`，查桥是否在跑 |
| `safety_fault_px4_local_position_reset_too_large` | EKF 复位幅度超阈值，调大 `max_ekf_reset_delta_m` 或先修 EKF |
| 起飞爬升不到位就转 hover | `minimum_takeoff_altitude_m` 的"最低爬升量"分支先满足了（`state_hover_minimum_altitude`），是设计行为 |
| 跟踪有阶跃 | `command_blend_sec` 会平滑；但要看 `/px4ctrl/debug` 的 `saturation_flags` |

---

## 13. 建议的阅读顺序

1. 本文档 §1–§2（坐标系）
2. `msg/DesiredState.msg` + `msg/ControlDebug.msg`
3. `src/px4ctrl_node.cpp` L360–L440（两个输入回调）
4. `src/px4ctrl_node.cpp` L656–L854（`calculate_output`，**核心**）
5. `src/px4ctrl_node.cpp` L913–L967（状态机）
6. `src/px4ctrl_node.cpp` L969–L1121（主循环）
7. `config/px4ctrl_sitl.yaml` 逐行对照 §8 的表
8. `scripts/ego_position_command_adapter.py` + `src/ego/ego_planner/src/traj_server.cpp`

**动手练习（按难度）**
1. 在 `/px4ctrl/debug` 里加一个 `position_error.norm()` 字段，观察跟踪收敛
2. 改 `kp` 为 `[3,3,3]`，看 `saturation_flags` 变化
3. 把 `transform_rotation_row_major` 改成单位阵，观察 `position_error` 爆炸
4. 在 `calculate_output` 里加一行打印 `velocity_reference`，理解串级结构
5. 把 `max_acceleration_mps2` 调到 0.5，观察推力投影逻辑的效果

---

## 14. 术语速查

| 缩写 | 全称 | 中文 |
|---|---|---|
| PX4 | Pixhawk 4 开源飞控 | |
| EKF2 | Extended Kalman Filter 2 | 状态估计器 |
| LIO / FAST-LIO | Fast LiDAR-Inertial Odometry | 激光惯性里程计 |
| EGO | Energy/geometric... EGO Planner | 梯度场局部规划器（这篇代码里的来源） |
| NED / ENU / FLU / FRD | 导航/机体系 | 北-东-地 / 东-北-天 / 前-左-上 / 前-右-下 |
| B 样条 | B-spline | EGO 输出的轨迹数学表示 |
| Offboard | PX4 外部控制模式 | 允许 ROS 直接给 PX4 发指令 |
| setpoint | 设定点 | 期望值 |
| feedforward | 前馈 | 直接把规划器的加速度/偏航率加进输出 |
| cascade | 串级 | 位置环外环、速度环内环 |
| anti-windup | 抗积分饱和 | 饱和时冻结/衰减积分 |
| prestream | 预流 | 进 Offboard 前先发一会儿 setpoint |
| latch | 锁存 | 故障一旦发生不再自动恢复 |
| SITL | Software-In-The-Loop | 软件在环仿真 |

---

*文档基于 `src/px4ctrl_node.cpp`（1328 行）编写，行号对应该文件。*
