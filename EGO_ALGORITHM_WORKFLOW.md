# px4ctrl_ws2 中的 EGO 算法与运行流程

本文对应工作空间 `/home/thunder/GRA_1/px4ctrl_ws2` 当前的单机 SITL 配置。内容分成两层：EGO-Planner 原论文中的规划思想，以及本工作空间把 FAST-LIO、EGO、坐标适配和 PX4 姿态/推力控制连起来的实际实现。

## 1. 一句话概览

```text
MID-360 点云和 IMU
        |
        v
FAST-LIO 局部里程计和点云 -----> PGO 回环优化及 map->lidar TF
        |                                  (全局可视化/回环信息)
        +--> /fastlio2/lio_odom ----+
        +--> /fastlio2/world_cloud -+--> EGO 局部地图、目标检查、重规划
                                           |
                                B-spline /drone_0_planning/bspline
                                           |
                              traj_server 轨迹采样和自动 yaw
                                           |
                     PositionCommand /drone_0_planning/pos_cmd
                                           |
                          lidar -> PX4 local NED 适配器
                                           |
                              /controller/desired_state
                                           |
                px4ctrl 位置/速度反馈 -> 姿态 + 归一化推力
                                           |
                           PX4 Offboard 姿态内环
```

EGO 负责“在本地障碍地图中生成、优化和重规划轨迹”。`traj_server` 负责连续求值，`px4ctrl` 负责按 PX4 NED 位置/速度反馈转换出姿态与推力。EGO 本身不输出电机量。

## 2. EGO-Planner 的算法思想

EGO-Planner 是 ESDF-free 的梯度式局部轨迹规划器。它不要求为整个地图预先构建欧氏符号距离场，而是在轨迹优化实际碰到障碍物时收集碰撞信息，结合无碰撞引导路径构造碰撞代价的方向信息，再优化 B-spline 控制点把轨迹从障碍物中“弹开”。轨迹通过碰撞检查后，再检查动力学可行性并在需要时重新分配时间。

原论文的核心优化可以概括为：

```text
J(Q) = lambda_s * J_smooth
     + lambda_c * J_collision
     + lambda_f * J_feasibility
     + lambda_fit * J_fitness
```

其中 `Q` 是 B-spline 控制点。平滑项抑制高阶导数，碰撞项推动轨迹离开障碍，动力学项惩罚超限速度/加速度，fitness 项使局部轨迹贴合引导轨迹或目标。论文算法的优点是只对当前优化覆盖到的局部空间提取碰撞信息，避免维护大范围高开销 ESDF；论文链接列在文末参考资料。

本仓库代码保留了 EGO 的 rebound 优化和 B-spline 结构，但地图输入是本项目的**膨胀占据栅格**：优化器查询 `getInflateOccupancy()`，根据碰撞段生成引导/反弹信息，再调用 L-BFGS 优化。不要把这里说成“EGO 全局 ESDF 地图”或“先用全局 A* 搜出最终路点后直接飞”。A* 是局部初始化/避障辅助，最终送给轨迹服务器的是经过优化和校验的 B-spline。实现可与文末上游仓库对照。

## 3. 传感器、定位与地图

### FAST-LIO 输入

MID-360 插件向 ROS 发布 `/livox/lidar` 和 `/livox/imu`。FAST-LIO 用 IMU 预积分/状态估计，并用点云对局部地图进行匹配，输出：

| 输出 | 用途 | 坐标约定 |
| --- | --- | --- |
| `/fastlio2/lio_odom` | EGO 当前位姿/速度；PX4 外部视觉桥也读取它 | `header.frame_id=lidar`；速度在 `child_frame_id=body` |
| `/fastlio2/world_cloud` | EGO 障碍地图输入 | FAST-LIO 的局部 `lidar` 世界坐标 |
| `/fastlio2/body_cloud` | PGO 回环输入 | 机体/点云局部坐标 |

EGO 的 odometry 回调会按 Odometry 约定把 `body` 系线速度用姿态旋转到规划世界系，再作为轨迹初始速度；如果帧名或时钟不一致，FSM 会拒绝或判 stale odometry，后续规划会暂停/急停。

### PGO 在当前接线中的位置

PGO 读取 `/fastlio2/body_cloud` 和 `/fastlio2/lio_odom`，维护回环约束，发布 `map->lidar` TF、原始/优化路径和回环可视化。当前 bringup 中 EGO 与 PX4 里程计桥仍直接订阅 `/fastlio2/lio_odom` 与 `/fastlio2/world_cloud`，所以 PGO 发布的全局 TF/优化路径不能简单等同于 EGO 或 `px4ctrl` 已改用的位姿参考。

### EGO 局部占据地图

`GridMap` 以 FAST-LIO odometry 更新局部中心，以 world cloud 更新障碍命中体素，并将近期点云命中保留约 4 s，减少稀疏扫描导致墙面瞬间消失。当前 bringup 参数为：

- 规划坐标：`lidar`；地图约 `80 x 80 x 10.5 m`，分辨率 `0.1 m`。
- 局部更新范围约 `10 x 10 x 3 m`；障碍点保留 `4 s`。
- 障碍膨胀 `0.45 m`；优化清距参数 `0.65 m`；最低飞行高度 `0.25 m`。
- 当前栅格膨胀代码在 X/Y 方向按 `ceil(0.45/0.1)` 扩张，在 Z 方向固定扩张上下各 1 个 voxel；因此不是 `0.45 m` 各向同性球体膨胀。
- 地图 z 原点/地面基准由 `ground_height=-0.15 m` 配置；地图外位置按占据处理/拒绝。

实现采用滚动局部地图，不等于整个 80 m 地图范围都已观测。地图没看到的区域、点云匹配失效、消息时延或不一致帧名都会影响“占据/自由”的判断。地图构建与膨胀实现见 [`grid_map.cpp`](src/ego/plan_env/src/library/grid_map.cpp) 和 [`grid_map.h`](src/ego/plan_env/include/plan_env/grid_map.h)。

## 4. 目标到 B-spline 的规划步骤

1. **接收目标。** `manual_goal` 发布 `geometry_msgs/PoseStamped` 到 `/goal_pose`，frame 必须是 `lidar`。目标 z 为规划坐标中的向上高度增量语义；适配 PX4 时会反号映射到 NED Down。错误 frame、NaN、地图外目标或膨胀占据目标会被拒绝。

   当前 FSM 只读取目标位置 XYZ，不把 `PoseStamped` 的 orientation 当作最终航向约束；自动航向由轨迹服务器按路径方向生成。固定 yaw 则由 `manual_goal` 另外发布给 `px4ctrl`。

2. **等待飞行与定位就绪。** `bringup.launch.py` 默认启动 takeoff。EGO 的 `wait_for_stable_px4` 门控要求 `/px4ctrl/status` 进入 `tracking_attitude_thrust_tracking`；目标会暂存。EGO 同时要求新鲜、合理速度且位于地图内的 FAST-LIO odometry。两个条件满足后才对目标调用规划。

3. **构造全局引导轨迹。** `planGlobalTraj()` 以当前位姿/速度为起点、目标为终点构造多项式引导轨迹，长距离时插入中间点并按速度上限分配段时间。它用于给局部规划提供方向，不是最终避障轨迹。

4. **确定局部目标和初值。** FSM 根据规划视距从全局引导轨迹上选局部目标。`reboundReplan()` 用多项式初值，或带随机中间点的初值生成 B-spline 控制点；A* 会在需要时为碰撞段寻找安全引导段。

5. **优化与动力学校验。** 优化器组合平滑、碰撞、可行性和目标贴合代价，对控制点做 L-BFGS 优化；碰撞代价通过碰撞轨迹与自由引导段构造 rebound 方向。结果还要检查膨胀栅格碰撞以及速度、加速度、jerk 等物理上限；超限时会放慢时间或 refine，而不是直接把未通过的轨迹送去飞。

6. **发布 B-spline。** 优化成功后发布 `traj_utils/Bspline` 到 `/drone_0_planning/bspline`，包含阶数、开始时间、轨迹 id、节点向量和控制点。

7. **执行时检查与重规划。** FSM 运行 `EXEC_TRAJ` 时监测轨迹进度、odometry 新鲜度和前向碰撞。接近轨迹段结束、偏离目标或前向预测碰撞时进入 `REPLAN_TRAJ`。如果新轨迹校验失败，不会把失败轨迹当作安全轨迹继续执行，而是发布 hold；保护条件触发时进入 `EMERGENCY_STOP`。

状态流可简写为：

```text
INIT -> WAIT_TARGET -> SEQUENTIAL_START -> EXEC_TRAJ
                                  ^            |
                                  |            +-- 轨迹进度/障碍检查
                                  |                         |
                                  +----- REPLAN_TRAJ <-----+
                                               |
                           无安全替代轨迹 -> HOLD / EMERGENCY_STOP
```

## 5. 轨迹服务器与 PX4 控制接口

`traj_server` 从 B-spline 按 ROS 仿真时间求值位置、速度、加速度，估算沿路径的 yaw/yaw rate，约 100 Hz 发布 `quadrotor_msgs/PositionCommand` 到 `/drone_0_planning/pos_cmd`。轨迹结束后服务器保持末端位置、速度和加速度置零。

`ego_position_command_adapter` 把 `lidar` 参考转成 PX4 本地 NED。当前旋转关系为：

```text
delta_NED = [delta_lidar_y, delta_lidar_x, -delta_lidar_z]
position_NED = delta_NED + startup_origin_translation
yaw_NED = pi/2 - yaw_lidar     (随后归一化到 [-pi, pi])
yaw_rate_NED = -yaw_rate_lidar
```

启动时适配器等待 FAST-LIO 和 PX4 都静止且数据新鲜，再锁存原点平移；PX4 的位置 reset 会更新平移。输出 `/controller/desired_state`，frame 是 `px4_local_ned`。

`px4ctrl` 用 PX4 的 NED 实测位置/速度计算跟踪加速度，施加推力、倾角、yaw 速率限制，再发布 `VehicleAttitudeSetpoint`。EGO 规划的是位置轨迹，`px4ctrl` 控制的是姿态+推力；两者之间有适配层，EGO 不直接控制 PX4 电机。

## 6. 当前接口与参数

| 环节 | 输入 | 输出 |
| --- | --- | --- |
| 手动目标 | — | `/goal_pose`，`PoseStamped`，frame `lidar` |
| 地图/状态 | `/fastlio2/lio_odom`、`/fastlio2/world_cloud` | `/drone_0_grid/grid_map/occupancy_inflate` |
| EGO 轨迹 | `/goal_pose`、FAST-LIO 状态、inflated occupancy | `/drone_0_planning/bspline` |
| 轨迹采样 | `/drone_0_planning/bspline` | `/drone_0_planning/pos_cmd` |
| 坐标适配 | `pos_cmd`、FAST-LIO/PX4 原点状态 | `/controller/desired_state`，`px4_local_ned` |
| 飞行控制 | NED reference、PX4 local position/velocity | PX4 attitude/thrust Offboard |

当前 bringup 默认 EGO 速度/加速度/jerk 上限为 `0.5 m/s`、`0.4 m/s^2`、`1.4 m/s^3`；平滑权重 `2.0`，规划视距 `7.5 m`；障碍膨胀 `0.45 m`、优化清距 `0.65 m`。这些是配置约束，不是保证整条实际轨迹严格等于该峰值的实测结果。参数入口在 [`bringup.launch.py`](src/controller/launch/bringup.launch.py) 和 [`advanced_param.launch.py`](src/ego/ego_planner/launch/advanced_param.launch.py)。

## 7. 当前流程的边界

- 用户输入目标坐标在 FAST-LIO `lidar` 局部系；PX4 feedback/reference 在 PX4 local NED。不能把同一组 `(x,y,z)` 数字直接视为相同物理轴向。
- EGO 用原始 FAST-LIO 局部 odometry/world cloud。PGO 在当前 launch 中发布 `map->lidar` TF 和优化路径，但 EGO 与 PX4 bridge 订阅的仍是 `/fastlio2/lio_odom`，不能假设 PGO 回环修正自动进入这两条消费链。
- 地图范围是配置边界，局部占据只由传感器已观测和近期保留的点构成；“目标在地图内”不等于沿途全部空间已观测为空。
- 优化器无法找到经校验的无碰撞轨迹时会失败或悬停；这是规划/地图可行性判断，不等于 PX4 控制器应该强行穿障继续。
- `quadrotor_msgs/PositionCommand`、`traj_utils/Bspline` 和 `px4ctrl/DesiredState` 是三个不同阶段的消息，检查时要确认各自 frame、时间戳、轨迹 id 和目标是否一致。

## 8. 本项目启动方式

PX4/Gazebo 与 FAST-LIO/PGO 分别先启动；`px4ctrl` bringup 会启动 EGO、桥接/适配器、控制器和默认 takeoff。bringup 自身已包含 EGO，不需再单独启动另一份 EGO 实例。

```bash
cd ~/GRA_1
source /opt/ros/jazzy/setup.bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/mid360_ws/install/setup.bash
source ~/GRA_1/fastlio2_pgo_ws/install/setup.bash
source ~/GRA_1/px4ctrl_ws2/install/setup.bash
export PX4_DIR=~/PX4-Autopilot
export GRA_WORKSPACE=~/GRA_1
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

正常运行链路后再发目标，例如：

```bash
ros2 run controller manual_goal 4.0 -4.0 0.5 --auto-yaw
```

代码导航：[`bringup.launch.py`](src/controller/launch/bringup.launch.py)、[`ego_replan_fsm.cpp`](src/ego/ego_planner/src/ego_replan_fsm.cpp)、[`planner_manager.cpp`](src/ego/ego_planner/src/planner_manager.cpp)、[`bspline_optimizer.cpp`](src/ego/bspline_opt/src/bspline_optimizer.cpp)、[`grid_map.cpp`](src/ego/plan_env/src/library/grid_map.cpp)、[`traj_server.cpp`](src/ego/ego_planner/src/traj_server.cpp)、[`ego_position_command_adapter.py`](src/controller/scripts/ego_position_command_adapter.py)。

## 参考

- Zhou et al., *EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors*, IEEE RA-L 2021, [论文](https://arxiv.org/abs/2008.08835)。论文解释了 ESDF-free 碰撞引导与 B-spline rebound 规划思路。
- [ZJU-FAST-Lab EGO-Planner 上游仓库](https://github.com/ZJU-FAST-Lab/ego-planner)。
