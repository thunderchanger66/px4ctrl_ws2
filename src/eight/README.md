# eight：裸 X500 分步八字试验

本包分为两个手动启动的步骤。`takeoff.launch.py` 先按 PX4 官方 ROS 2 示例
预流位置 Offboard 设定值，切换 Offboard 并解锁，爬升到 2 m 后把控制权交给
`px4ctrl` 的姿态设定值；该终端必须保持运行。
看到 `Offboard ready; keep this launch running` 后，再在另一终端启动
`eight.launch.py` 发布两圈八字轨迹，结束后自动请求 PX4 LAND。不需要 QGC，
也不需要预先指定悬停点。

先构建一次：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zone/px4ctrl_ws-main
colcon build --packages-up-to eight --symlink-install --executor sequential
source install/setup.bash
```

然后按顺序使用四个终端。ROS 终端都需先执行
`source /opt/ros/jazzy/setup.bash` 和
`source /home/zone/px4ctrl_ws-main/install/setup.bash`。

```bash
# 终端 1：裸 X500 仿真
cd /home/zone/PX4-Autopilot
make px4_sitl gz_x500

# 终端 2：DDS Agent（由你单独管理，以下两条择环境执行）
MicroXRCEAgent udp4 -p 8888
# 如果命令找不到，先 source /home/zone/ros2_ws/install/setup.bash

# 终端 3：起飞和 Offboard 接管，保持运行直到降落结束
ros2 launch eight takeoff.launch.py

# 终端 4：等终端 3 打印 Offboard ready 后执行
ros2 launch eight eight.launch.py output_dir:=/tmp/eight_runs
```

`takeoff.launch.py` 会启动 Gazebo `/clock` 桥接、`px4ctrl` 和 SITL GCS 心跳，
不会启动 Agent；`eight.launch.py` 只启动轨迹发布和评估节点，不会再次起飞、
切模式或启动控制器。若已有时钟桥接，可对第一步传入
`start_clock_bridge:=false`。飞行结束并确认 PX4 已落地后再关闭终端 3。

默认轨迹为 `x=x0+2sin(phi)`、`y=y0+sin(2phi)`、高度和航向保持，50 Hz
发布；默认每圈 8.9 秒，峰值指令速度约 2.0 m/s，两圈含平滑起停约 20.8 秒。
如需复测原来的约 0.6 m/s，可在第四个终端使用
`ros2 launch eight eight.launch.py period_sec:=30.0`。输出 CSV/JSON
到 `/tmp/eight_runs`；JSON 的 `passed` 给出 RMSE、P95、最大误差、末段稳定
及 Offboard/failsafe 的综合判定。完整飞行效果仍需在仿真中验收。

本包的 `sitl_gcs` 辅助进程提供本机 MAVLink 地面站心跳，并临时给 PX4 的
`COM_RCL_EXCEPT` 增加 Offboard 的无遥控输入例外；进程正常退出时恢复原值。
这只用于**无遥控器的裸 X500 SITL**，不应当用于实机。若另一个程序已经在
使用 MAVLink UDP 14550，可对第一步传入 `start_sitl_gcs:=false`，并自行保证
地面站心跳和无遥控配置。

起飞只在 PX4 预检通过、地面未解锁且位置有效时发起；不会绕过
PX4 预检。起飞节点以 PX4 本地 NED 坐标持续发送位置目标；到达 2 m 且低速后，
`px4ctrl` 才被允许捕获当前位置并接替姿态控制。无需人为设置悬停点。第一步
每 2 秒打印 PX4 模式、落地检测、速度和 `px4ctrl` 状态。上次日志表明旧流程
虽然进入自动起飞模式并解锁，但 `landed` 一直为真、NED 高度约为 0，随后
自动上锁；因此本轮改用位置 Offboard 设定值起飞。

若 PX4 已停在约 2 m、`permission=False` 且控制器仍在等待，先检查日志中的
`land_age_s`。PX4 的落地检测在状态不变时仅以 1 Hz 发布，接管逻辑现在允许
2 秒的落地检测消息间隔。正常交接应看到 `permission=True`，随后出现
`Offboard ready`。若 PX4
命令被拒绝、状态超时、定位失效或跟踪误差持续超过 1.5 m，试验中止并在
仍由本包控制且无 PX4 failsafe 时请求 LAND。发生 PX4 failsafe 时尊重 PX4
自身的故障动作。试验结束后自动请求 LAND，不再要求手动切模式。
