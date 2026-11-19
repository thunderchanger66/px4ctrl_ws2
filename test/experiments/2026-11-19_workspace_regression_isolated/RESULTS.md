# px4ctrl_ws2 工作空间审计与中间回归

本目录记录中间排查。交付版的完整五点结果见[最终仿真回归](../2026-11-19_workspace_acceptance/RESULTS.md)。

环境：ROS 2 Jazzy、PX4 SITL（固件 `c61b778e2d`）、Gazebo 工厂场景、MID-360、FAST-LIO/PGO。工作空间包含九个包。保留原有目标点、`--auto-yaw` 用法、话题和消息格式、C1 控制增益及 EGO 速度、加速度和障碍膨胀参数。实验只连接本机 SITL。

## 查到的问题与修复

| 范围 | 问题与改法 |
| --- | --- |
| `px4ctrl` 控制器 | 预发送原从收到首条命令计时，期间不一定发出了设定点；改为从实际发布开始计时，长中断重新计时。旧命令不会触发起飞或解锁。未开始控制且未解锁时的 EKF 重置可建立新初始坐标；飞行中仍校验重置量。连续重规划时以最近的混合参考作为下次混合起点。 |
| `px4ctrl` 起飞、适配和 launch | 起飞脚本原本从位置回调直接锁定目标，绕过了 `--require-ev-fusion`；把锁定放回融合检查之后，同时修正 Jazzy 时间差 API 和过期位置判断。适配器过滤非有限的对齐输入；航向重置由控制器处理一次，位置平移仍由适配器补偿。单控制器 launch 现在有实际安装的空中接管配置，默认等外部授权。bringup 的 `use_sim_time` 传给起飞、桥接和控制器。 |
| `fastlio2_px4_odom_bridge` | 先筛掉无时间戳、过期及乱序样本，再对齐原点；同一条消息重复到达不会锁死桥接。禁用自动对齐后使用配置中的平移；PX4 重置计数只在实际发布新样本时递增。超出运动或位置边界仍保留原保护。 |
| `ego_planner` | 最新有效目标遇到定位暂不可用时进入队列，待 EGO 定位与 PX4 门槛满足再处理。轨迹服务器检查三次样条的加速度导数节点跨度，拒绝无法求导或无有效起始时间的消息。 |
| `path_searching` | A* 不再原地修改堆的排序键：以不可变优先级入堆，出堆跳过旧条目。修正负方向索引取整与浮点越界转换，失败后清空旧路径，使用单调时钟判断搜索超时。 |
| `plan_env` | 过滤非有限及地图外的里程计；首帧点云前不读未初始化的地图窗口；检查虚拟天花板索引；水平膨胀为零时，仍清理原有垂向膨胀层。 |
| `traj_utils` | 单段多项式进入正确的边界条件求解器，拒绝非法段数、维度及时间；已有多段求解不变。 |
| `bspline_opt`、`quadrotor_msgs`、`px4_msgs` | 检查与上述包的调用、构建依赖与消息边界，不替换优化器或裁剪消息 API。工作空间与本机固件的 `VehicleLocalPosition`、`VehicleStatus`、`VehicleAttitudeSetpoint`、`VehicleOdometry` 定义逐字节一致。 |
| 测试工具 | 单独 DDS 域和本机发现；输入无效时重置稳定计时；用同时间戳的 EGO `PositionCommand` 与适配后的 `DesiredState` 还原目标 NED 位置，避免中途停车被误判到达。 |

## 验证方式

```bash
cd /home/thunder/GRA_1/px4ctrl_ws2
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=3 colcon build --symlink-install --parallel-workers 2
colcon test --packages-select px4ctrl traj_utils
colcon test-result
python3 test/experiments/run_px4_ned_hover_suite.py --ros-domain-id 81
```

九个包完成构建。四项功能回归通过：外部视觉融合前禁止起飞参考、无效本地数据不得继续发起飞参考（同一用例）；适配器对齐及 EKF 重置；单段轨迹起终点的位姿导数边界条件；零时长拒绝。全量生成代码 lint 在 `px4_msgs` 的 Fast DDS 生成代码项挂起，已停止；没有把它算作通过。CMake 仍有 PCL/FLANN 提示，LBFGS 有现存未用参数警告。

参考：PX4 官方 Offboard 文档与本机 `rcS` / `dds_topics.yaml`（预发送、姿态和推力坐标、DDS 域）；ROS 2 Jazzy `nav_msgs/Odometry.msg`（位姿使用父坐标系，速度使用子坐标系）；ZJU FAST Lab 官方 EGO-Planner 与 EGO-Swarm ROS 2 上游说明。应用的是这些接口约定，没有引入另一套规划器。

## 仿真测量

每点重新启动整套 SITL，先起飞并采集实测悬停点。只用 PX4 本地 NED 实测位置与适配器的 NED 参考计算三维轨迹误差，约 20 Hz，排除起飞和等待阶段。每个样本算 `‖p_ref − p_px4‖₂`，汇总 RMSE、P95、最大值。真实目标误差另外算 `‖p_goal,NED − p_px4‖₂`。到达要求参考距指定目标不超过 `0.08 m`，实测误差不超过 `0.20 m` 且速度不超过 `0.20 m/s`，并连续稳定 `1 s`。定位误差、实机表现和物理碰撞距离不在本指标内。

中间批次保留为故障证据：有一轮 P02 只飞到原点附近的停止参考，旧观测器却判为到达；另有几轮收到了时间域不一致或 DDS 长度异常的消息并停在起飞前。那些批次不进入本报告的五点验收表。原始轨迹和各节点日志与本文件同目录。

本目录这轮已使用独立 DDS 域和校正后的到达判据。P02、P03 到达；P01 末段补追求解报舍入错误后进入 `SAFE_HOLD`，最终超时；P04、P05 没观察到移动轨迹参考。五轮均确认落地并退出进程组。随后新增目标发送确认与补追备用初始化，最终验收结果单独保存。
