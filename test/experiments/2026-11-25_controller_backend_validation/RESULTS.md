# 独立 `controller` 包接入 PX4Ctrl 七轮 SITL 验证

日期：2026-11-25  
工作空间：`px4ctrl_ws2`  
后端：`src/controller`（独立 bringup、EGO 轨迹适配、起飞和目标发布）  
仿真：PX4 SITL + Gazebo 工厂世界 + FAST-LIO2/PGO + EGO  
配置：`src/controller/config/px4ctrl_gra_mid360.yaml`（沿用现有 MID-360 调参）

## 结果

七轮全部完成。六个单点各自重新启动完整仿真，从稳定悬停点开始；第七轮重新启动后，从悬停点连续飞完六个目标。每轮均确认落地、解锁清除，并确认启动的整栈进程关闭。

| 实验 | 目标（EGO lidar，m） | 到达时间（s） | NED 跟踪 RMSE / P95 / 最大（m） | 最终跟踪误差（m） | 最终目标误差（m） | 样本 | 结果 |
|---|---:|---:|---:|---:|---:|---:|---|
| P01 | `(2, 0, 0.5)` | 10.40 | `0.153 / 0.292 / 0.336` | 0.034 | 0.037 | 208 | 到达、落地、关闭 |
| P02 | `(2, -0.5, 0.5)` | 20.15 | `0.170 / 0.318 / 0.373` | 0.129 | 0.128 | 403 | 到达、落地、关闭 |
| P03 | `(4, -4, 0.5)` | 11.55 | `0.092 / 0.180 / 0.201` | 0.079 | 0.103 | 231 | 到达、落地、关闭 |
| P04 | `(3, 0, 0.5)` | 17.85 | `0.162 / 0.317 / 0.435` | 0.065 | 0.068 | 357 | 到达、落地、关闭 |
| P05 | `(2, 0, 1.5)` | 7.30 | `0.124 / 0.179 / 0.184` | 0.027 | 0.026 | 146 | 到达、落地、关闭 |
| P06 | `(10, 0, 0.5)` | 16.75 | `0.076 / 0.151 / 0.200` | 0.024 | 0.048 | 335 | 到达、落地、关闭 |
| P07 连续 | P01 → P02 → P03 → P04 → P05 → P06 | **91.75** | `0.098 / 0.202 / 0.392` | 0.024 | 0.025（末点） | 1835 | 六点全到达、落地、关闭 |

P07 从悬停点开始的逐点到达时间：P01 `12.45 s`、P02 `27.70 s`、P03 `53.95 s`、P04 `64.00 s`、P05 `74.55 s`、P06 `91.75 s`。连续实验中的目标监视器有 3 次超时重发；重发均成功，EGO 将已在执行中的重复目标忽略，六个航点都按序到达。

本次有效飞行样本内没有 failsafe 样本，也没有因 PX4 定位无效、状态过期或 Offboard 退出而丢弃的统计样本。P02 的最终目标误差最大，为 `0.128 m`，仍满足本测试 `0.20 m` 的到达阈值。

## 统计口径

- 起点是起飞并稳定悬停后捕获的 PX4 `VehicleLocalPosition` 本地 NED 位置；开始时间取参考开始移动的时刻，不包含起飞和等待目标的阶段。
- 跟踪误差是控制器 `/controller/desired_state` 参考位置与 PX4 `/fmu/out/vehicle_local_position_v1` 实测位置之间的三维欧氏距离；只在 PX4 本地定位有效、状态新鲜、已解锁、处于 Offboard 且无 failsafe 时采样，频率约 20 Hz。
- 最终目标误差是 PX4 本地 NED 实测位置与目标点经 EGO 适配器变换得到的 PX4 NED 目标位置之间的距离。终点判据要求目标参考接近目标、速度足够低、实测位置进入 `0.20 m` 半径并持续稳定 1 秒。
- 本批验证了新控制器、起飞流程、EGO 轨迹适配、目标输入和 PX4 NED 跟踪链路；没有单独测量最小障碍物距离或碰撞裕度，因此这些结果不能替代专项避障安全测试。

## 如何复跑

运行器会对每个实验单独启动和关闭 PX4/Gazebo、FAST-LIO2/PGO、EGO 与 `controller`。确认当前没有另一个仿真或 Agent 占用后执行；`--output` 必须是新建或空目录。

```bash
cd ~/GRA_1/px4ctrl_ws2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 test/experiments/run_controller_px4_ned_hover_suite.py \
  --output test/experiments/2026-11-25_controller_backend_validation_rerun \
  --ros-domain-id 82
```

测试运行器使用 [run_controller_px4_ned_hover_suite.py](../run_controller_px4_ned_hover_suite.py)；观察器订阅新控制器的 `/controller/desired_state`，定义在 [px4_ned_hover_observer_controller.py](../px4_ned_hover_observer_controller.py)。每个实验目录均包含逐样本 CSV、汇总 JSON、航点进度、起飞/目标/落地日志、PX4/Gazebo 与 FAST-LIO/PGO 启动日志。

总表及完成标志见 [suite_results.json](suite_results.json)。
