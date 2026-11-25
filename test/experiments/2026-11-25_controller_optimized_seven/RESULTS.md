# PX4Ctrl/EGO 低跟踪误差七轮复测

测试日期：2026-11-25  
仿真：PX4 SITL + Gazebo 工厂世界 + FAST-LIO2/PGO + EGO + 独立 `controller` 包  
参考坐标：EGO 输入 `lidar`；统计仅用 PX4 local NED  
起飞：每轮从地面重新启动，自动起飞 0.5 m，确认悬停后才发送目标  
控制器文件：`src/controller/config/px4ctrl_gra_mid360.yaml`  
EGO 限值：`max_vel=0.5 m/s`、`max_acc=0.4 m/s²`、`max_jerk=1.4 m/s³`；障碍膨胀、规划时域等其余参数保持原值。

## 结果

六个单点实验各自独立重启完整仿真；第七轮独立重启并连续飞完六个点。每轮均确认到达、降落、解除解锁和整栈关闭。

| 实验 / 目标（lidar，m） | 到达时间 s | NED 跟踪 RMSE m | P95 m | 最大误差 m | PX4 峰值速度 m/s | PX4 峰值加速度 m/s² | 末端目标误差 m |
|---|---:|---:|---:|---:|---:|---:|---:|
| P01 `(2, 0, 0.5)` | 11.15 | 0.086 | 0.139 | 0.145 | 0.447 | 0.470 | 0.046 |
| P02 `(2, -0.5, 0.5)` | 11.25 | 0.073 | 0.121 | 0.131 | 0.432 | 0.416 | 0.041 |
| P03 `(4, -4, 0.5)` | 24.10 | 0.058 | 0.095 | 0.113 | 0.438 | 0.418 | 0.024 |
| P04 `(3, 0, 0.5)` | 14.90 | 0.087 | 0.123 | 0.134 | 0.466 | 0.609 | 0.012 |
| P05 `(2, 0, 1.5)` | 11.55 | 0.087 | 0.127 | 0.144 | 0.460 | 0.386 | 0.044 |
| P06 `(10, 0, 0.5)` | 34.95 | 0.053 | 0.098 | 0.130 | 0.402 | 0.363 | 0.023 |
| P07 连续 P01→P02→P03→P04→P05→P06 | 104.80 | 0.051 | 0.113 | 0.150 | 0.639 | 1.449 | 0.015 |

连续航线从悬停开始的逐点到达时间：P01 `11.15 s`、P02 `15.40 s`、P03 `38.10 s`、P04 `58.80 s`、P05 `69.90 s`、P06 `104.80 s`。

七轮合并 RMSE 平均为 `0.071 m`。与前一批相同七个目标的测试记录相比，RMSE 分别降低约 `43% / 57% / 37% / 46% / 29% / 30% / 48%`。这次是**组合调优**：控制器增益恢复到原目录 `controller` 基线 `[kp]=[1.5,1.5,1.8]`、`[kv]=[1.2,1.2,1.4]`、`[ki]=[0.10,0.10,0.15]`，同时将 EGO 限值从 `1.0/1.0/3.0` 降到 `0.5/0.4/1.4`；因此不能把改善单独归因于某一个参数。

## 指标定义与限制

- 轨迹起点是 PX4 NED 实测状态已稳定悬停后，NED 参考首次移动的时刻；起飞阶段不计入跟踪统计。
- 跟踪 RMSE 是 20 Hz 采样的 `controller/DesiredState` NED 参考位置与 PX4 `VehicleLocalPosition` NED 实测位置间三维欧氏误差的均方根。
- 峰值速度取 PX4 NED 的 `vx, vy, vz` 向量模最大值。峰值加速度取 PX4 `VehicleLocalPosition.ax, ay, az` 的 NED 向量模最大值，不是由稀疏位置差分估算。
- 到达判据要求参考稳定、参考目标偏差不超过 `0.08 m`、实测目标误差不超过 `0.20 m`、实测速度不超过 `0.20 m/s`，连续满足 1 秒。
- 最大样本跟踪误差为 `0.150 m`（连续路线），没有 failsafe 样本或无效状态跳过。测试验证这七个模拟目标的跟踪结果；未测实机，也未统计障碍物最小间距，不能据此宣称避障裕度已验证。

## 关于原 PX4Ctrl 目录的 RMSE

`/home/thunder/下载/PX4Ctrl(1)/PX4Ctrl/src/controller/src/px4ctrl_node.cpp` 与本工作空间 `src/controller/src/px4ctrl_node.cpp` 逐字一致。该原目录提到的小 RMSE 圆轨迹演示实际启动的是另一个 `controller1` PD-MRAC 控制器，且采用平滑两圈圆轨迹和独立 tracking/terminal-hover 统计；它不能直接作为本次 EGO 点到点 RMSE 的同口径基线。本次优化没有换成 `controller1`，仍是原 `controller` 控制算法，只调回该控制器原有基线增益并限制 EGO 参考速度/加速度/jerk。

## 完整记录与复跑

每个 P01–P07 目录保存 `samples.csv`、`samples.json`、`result.json`、航点进度和各启动/降落日志；整批状态在 [suite_results.json](suite_results.json)。测试运行器和 NED 观察器为 [run_controller_px4_ned_hover_suite.py](../run_controller_px4_ned_hover_suite.py) 与 [px4_ned_hover_observer_controller.py](../px4_ned_hover_observer_controller.py)。

```bash
cd ~/GRA_1/px4ctrl_ws2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 test/experiments/run_controller_px4_ned_hover_suite.py \
  --output test/experiments/2026-11-25_controller_optimized_seven_rerun \
  --ros-domain-id 88
```
