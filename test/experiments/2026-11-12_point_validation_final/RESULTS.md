# px4ctrl_ws2 调优参数多点性能验证

## 测试配置

本报告验证当前推荐配置：EGO `max_vel=1.0 m/s`、`max_acc=1.0 m/s^2`、`max_jerk=3.0 m/s^3`、`lambda_smooth=2.0`、障碍膨胀 `0.45 m`、优化 clearance `0.65 m`；px4ctrl 使用 C1 `kp=[1.8,1.8,2.1]`、`kv=[1.4,1.4,1.6]`，其余控制器与 PX4 参数保持不变。每点使用 Gazebo factory world、MID-360、FAST-LIO/PGO、完整 bringup，先独立起飞 `0.5 m`，再以自动 yaw 发一个 lidar-frame 目标。每轮结束后请求降落、确认落地/解锁，再关闭整套进程后进入下一轮。

点集覆盖短轴向、反向、横向、斜向、中距离、长距离横向及用户常用的 `(4,-4,0.5)`。目标坐标均相对 EGO `lidar` 规划系。

## 指标定义

到达定义为 FAST-LIO 实际位置距目标 `<=0.20 m` 且速度 `<=0.20 m/s`，连续保持 `1.0 s`。位置跟踪误差为 EGO `PositionCommand.position` 与 FAST-LIO `lio_odom.pose.position` 的 3D 距离。加速度峰值为 EGO 参考加速度向量范数峰值。膨胀图 clearance 是 FAST-LIO 位置到 EGO `/grid_map/occupancy_inflate` 最近点的距离；它是占据图余量代理值，不等价于 Gazebo 碰撞传感器或连续轨迹扫掠碰撞证明。

本轮建议门槛：clearance min `>=0.50 m`；到达点还需 PX4 无 failsafe、地图数据足够新鲜、无里程计越界/超时/重规划故障，并最终确认降落。目标被 EGO 拒绝或没有生成 spline 的点仍保留记录，但不伪造到达时间和跟踪误差。

## 多点结果

| 点 | 目标 `(x,y,z)` m | 到达时间 | 位置 RMSE / P95 / 最大值 m | 最终误差 m | 速度峰值 m/s | 加速度峰值 m/s² | 膨胀图最小距离 m | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| P01 短 X | `(1,0,0.5)` | `4.679 s` | `0.204 / 0.410 / 0.454` | `0.151` | `1.022` | `1.242` | `0.981` | 到达，clearance 通过；无 failsafe |
| P02 反向 X | `(-2,0,0.5)` | `4.737 s` | `0.189 / 0.297 / 0.335` | `0.070` | `1.225` | `1.039` | `0.927` | 到达，clearance 通过；无 failsafe |
| P03 横向 Y | `(0,2,0.5)` | `8.268 s` | `0.215 / 0.313 / 0.354` | `0.054` | `1.324` | `1.048` | `0.342` | 到达，但安全余量失败；不得作为安全通过点 |
| P04 斜向 | `(2,-2,0.5)` | N/A | N/A | N/A | `0.034`（悬停峰值） | N/A | N/A | EGO 明确拒绝：目标在 occupied/inflated voxel；无 spline，不起飞执行该目标 |
| P05 长 X | `(4,0,0.5)` | `8.094 s` | `0.208 / 0.321 / 0.370` | `0.091` | `1.330` | `0.897` | `0.645` | 到达，clearance 通过但余量较小；无 failsafe |
| P06 长负 Y | `(0,-4,0.5)` | 未到达，`120 s` 超时 | `0.091 / 0.142 / 0.659` | `3.715` | `0.919` | `0.995` | `0.204` | 生成 spline 并多次重规划，但未到达且 clearance 失败；PX4 无 failsafe |
| P07 常用目标 | `(4,-4,0.5)` | `11.595 s` | `0.271 / 0.510 / 0.528` | `0.111` | `1.330` | `1.038` | `0.356` | 到达，但安全余量及 P95 跟踪误差失败；不得判为安全通过 |

原始记录分别见 [P01](P01_x1_short/result.json)、[P02](P02_xminus2_reverse/result.json)、[P03](P03_y2_lateral/result.json)、[P04 占据拒绝日志](../2026-11-12_point_validation_retry/P04_diagonal_2m/bringup.log)、[P05](P05_x4_long_axis/result.json)、[P06](../2026-11-12_point_validation_final_retry/P06_yminus4_long_axis/result.json)、[P07](../2026-11-12_point_validation_final_retry/P07_regular_4_minus4/result.json)。各试验目录保留对应 CSV、JSON 和 PX4/Gazebo、bringup、FAST-LIO/PGO 日志。

## 结论

7 个点中，3 个到达且通过 `0.50 m` 地图 clearance 门槛（P01、P02、P05）；P03 与 P07 虽到达，但观测 clearance 分别只有 `0.342 m`、`0.356 m`；P06 未到达，最小 clearance `0.204 m`；P04 被规划器正确拒绝。全组未记录 PX4 failsafe，但**这不等于所有目标避障通过**。对 P06/P07 这类长目标，当前环境观测到的局部点云/膨胀地图没有足够余量，不能仅凭轨迹到达宣称安全。

保留当前 C1 控制参数是合理的短、中距离折中，但这组多点测试不支持把同一调参结论外推到长路线。P03/P06/P07 应作为地图余量不合格点处理；不能通过缩小 `obstacles_inflation`、`collision_clearance` 或关闭 fail-safe 让它们变成“通过”。若实际操作以 `(4,-4,0.5)` 为常用点，应先改善/核验该方向点云可见性、地图占据更新与规划通道，再重新做安全测试。

## 测试器修正

运行器先前把 observer 自身的 `/goal_pose` 订阅计为“EGO 已准备”，可能造成目标发布竞态。最终运行器现在等待 `/drone_0_ego_planner_node` 本身订阅 `/goal_pose`，且在未收到 spline 时有限重发；明确收到目标占据拒绝后立即分类。落地脚本的 near-ground disarm 使用 PX4 标准 force magic `21196`，但仍先要求 near-ground/低速连续稳定，不对空中状态强制解锁。

## 验证状态

`colcon build --packages-up-to px4ctrl ego_planner --symlink-install` 成功；测试脚本 `py_compile` 与 `git diff --check` 成功。多点实验后 PX4/Gazebo、FAST-LIO/PGO、EGO/px4ctrl 全部关闭。验证仅覆盖本表中的 factory SITL 场景和路线；它不是实机认证，也不证明未观测区域或动态障碍下的避障安全。
