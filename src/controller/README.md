# controller：EGO + PX4 PD-MRAC 控制

当前默认 `px4ctrl_node` 已替换为 PX4Ctrl 原目录中的 filtered-trend PD-MRAC 外环。包名、`controller/msg/DesiredState` 输入、`/controller/desired_state`、`/px4ctrl/status`、`/px4ctrl/debug`、PX4 姿态/推力输出、EGO 适配器和 `manual_goal` 命令保持现有接线。

## 启动当前 MID-360 + EGO 流程

PX4/Gazebo 和 FAST-LIO2/PGO 依原流程启动后：

```bash
cd ~/GRA_1/px4ctrl_ws2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch controller bringup.launch.py
```

目标仍在 EGO 的 `lidar` 规划帧：

```bash
ros2 run controller manual_goal 4.0 -4.0 0.5 --auto-yaw
```

当前完整 bringup 使用 `config/px4ctrl_gra_mid360.yaml`，结合 MID-360 hover-thrust 标定和 filtered-trend PD-MRAC。EGO 默认参考上限为 `2.0 m/s`、`1.5 m/s²`、`4.0 m/s³`；controller 最大倾角为 `30°`、最大加速度为 `6.0 m/s²`、水平 jerk 上限为 `6.0 m/s³`。EGO 加速度预览由适配器限制在 `1.8 m/s²` 和 `5.0 m/s³`，默认预览时间为 `0.15 s`。单点 P03 的重复指标见 `test/experiments/2026-11-26_p03_rmse_jerk_tuning/RESULTS.md`。


`/controller/pd_mrac_state` 发布 MRAC 扰动估计、位置/速度模型误差、自适应矩阵范数和滤波误差范数。
