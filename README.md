# GRA_1 PX4 + FAST-LIO2 + EGO 工作空间

此工作空间中的唯一 ROS 控制器包是 `controller`。当前 `controller/px4ctrl_node` 使用 filtered-trend PD-MRAC 控制律，接收 EGO 轨迹转换后的 PX4 本地 NED 期望状态，向 PX4 发布姿态和推力设定值。旧的固定增益控制器包已移除。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
cd ~/GRA_1/px4ctrl_ws2
colcon build --packages-up-to controller --symlink-install
source install/setup.bash
```

## 运行

按部署流程启动 PX4/Gazebo 和 FAST-LIO2。PGO 当前只发布 `map->lidar` TF/优化路径，EGO 与 PX4 bridge 仍消费 FAST-LIO 局部 `lidar` odometry；除非需要查看回环结果，否则无需为了 P03 单点跟踪启动 PGO。然后启动完整 EGO + 控制链路：

```bash
source /opt/ros/jazzy/setup.bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/mid360_ws/install/setup.bash
source ~/GRA_1/fastlio2_pgo_ws/install/setup.bash
source ~/GRA_1/px4ctrl_ws2/install/setup.bash
ros2 launch controller bringup.launch.py start_rviz:=true
```

稳定起飞后，在 EGO 的 `lidar` 规划坐标中发送目标：

```bash
ros2 run controller manual_goal 4.0 -4.0 0.5 --auto-yaw
```

控制器参数位于 `src/controller/config/px4ctrl_gra_mid360.yaml`。当前完整 bringup 的 EGO 参考上限为 `2.0 m/s`、`1.5 m/s²`、`4.0 m/s³`，适配器使用经 P03 重复验证的 `0.15 s` 有界加速度预览。P03 指标及调参过程见 [P03 调参报告](test/experiments/2026-11-26_p03_rmse_jerk_tuning/RESULTS.md)。
