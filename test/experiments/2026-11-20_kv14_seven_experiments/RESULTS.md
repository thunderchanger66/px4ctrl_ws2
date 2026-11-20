# `kv=[1.4,1.4,1.6]` 七轮实验记录

本次使用 `kv=[1.4,1.4,1.6]`；`kp=[1.6,1.6,2.0]`、`ki=[0.08,0.08,0.13]` 保持原工作区值。六个单点实验各自重启完整仿真，从有效悬停点发目标。第七轮要求按 P01 至 P06 连续飞行；用户中途要求停止，所以本文件是**部分结果，不能视为七轮全部完成**。suite 总表 `complete=false`。

跟踪误差是 PX4 local NED 中 `DesiredState.position` 与 `VehicleLocalPosition.position` 的三维距离，约 20 Hz。计时从稳定悬停后 NED 参考首次开始移动起算。单点终点误差按指定目标 NED 位置计算。连续路线要求每个航点通过参考稳定、参考到目标偏差 `<=0.08 m`、实际到目标误差 `<=0.20 m` 和速度 `<=0.20 m/s` 连续 1 s，记录从第一个航点移动至最后航点的全程统计及逐点到达时间。

| 实验 | 目标 / 路线 | 用时 s | NED 跟踪 RMSE / P95 / 最大值 m | 最终目标误差 m | 状态 |
| --- | --- | ---: | ---: | ---: | --- |
| P01 | `(2,0,0.5)` | 22.40 | `0.188 / 0.354 / 0.479` | 0.094 | 到达、落地确认 |
| P02 | `(2,-0.5,0.5)` | 未到达 | `1.097 / 3.300 / 4.334` | 2.972 | PX4 状态无效/失效，未到达；落地确认 |
| P03 | `(4,-4,0.5)` | 未到达 | `1.521 / 1.579 / 13.836` | 11.887 | PX4 退出 Offboard，出现大幅本地高度 reset 与安全停机；落地确认 |
| P04 | `(3,0,0.5)` | 9.15 | `0.143 / 0.227 / 0.239` | 0.075 | 到达；落地 helper 未确认，整栈已关闭 |
| P05 | `(2,0,1.5)` | 5.15 | `0.140 / 0.183 / 0.191` | 0.170 | 到达、落地确认 |
| P06 | `(10,0,0.5)` | 18.05 | `0.087 / 0.158 / 0.209` | 0.020 | 到达、落地确认 |
| P07 | P01 -> P02 -> P03 -> P04 -> P05 -> P06 | 中断前 54.90 | `0.234 / 0.468 / 0.656`（部分） | 不适用 | 用户中断；只完成 P01 航点 |

P07 在 44.00 s 稳定到达 P01，NED 目标误差 `0.106 m`，随后开始去 P02；约 54.90 s 时测试被中断。中断时已经记录到一段包含 PX4 Offboard 状态丢失的部分轨迹，因此该统计不代表完整连续路线性能，也没有连续路线最终误差。

P02/P03 的失败不能据此归因于 `kv`：P02 记录到状态无效和 failsafe；P03 日志记录 PX4 本地 `z` reset 约 `1.14 m`、控制器因较大的后续 reset 锁存保护、EGO 进入 emergency stop。它们是本次测试结果中需要单独解决的定位/Offboard链路问题。P04 的 landing 确认也需复查，尽管隔离脚本确认整组进程已退出。

原始文件：每个点的 `samples.csv`、`samples.json`、`result.json` 和启动/目标/落地日志；P07 `waypoint_progress.json` 保留中断时的逐点进度。汇总见 [`suite_results.json`](suite_results.json)。

## 手动重跑七轮

请确认没有其他 PX4/Gazebo/FAST-LIO/PGO 实例后，在终端执行。runner 会依次执行六个单点和一个连续六航点实验，每轮独立启动、起飞到悬停、测试、降落并关闭整栈。连续路线顺序固定为 P01、P02、P03、P04、P05、P06。输出目录必须是新的或空目录。

```bash
cd ~/GRA_1/px4ctrl_ws2
source /opt/ros/jazzy/setup.bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/mid360_ws/install/setup.bash
source ~/GRA_1/fastlio2_pgo_ws/install/setup.bash
source ~/GRA_1/px4ctrl_ws2/install/setup.bash
export PX4_DIR=~/PX4-Autopilot
export GRA_WORKSPACE=~/GRA_1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 test/experiments/run_px4_ned_hover_suite.py \
  --output test/experiments/2026-11-20_kv14_seven_experiments_rerun \
  --ros-domain-id 82
```

`kv` 已保存在 `src/px4ctrl/config/px4ctrl_bringup.yaml`，runner 的 bringup 会加载该文件。运行过程中按 Ctrl-C 会先尝试降落并关闭进程组，记录已完成航点，并将 suite 标为未完成。
