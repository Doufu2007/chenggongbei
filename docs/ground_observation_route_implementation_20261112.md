# 独立首点外观察路线实施补充

本版为主代理作出的工程裁定，并非用户指定的参数：取消 `.20m/s` 档，请求只用 `.03/.10m/s`；初次 `.03` 必须连续 fresh 满至少 1 wall/sim 秒才可 `.10`。首段实际前进 `.05m` 立即请求零速，以之后真实 odom 累计路径增量 `≤.01m` 且静稳 1 wall/sim 秒验证停车，之后才继续路线；末段最后 `.10m` 降至 `.03`。

冻结运行时上限：全局 300s、单次 TURN（含 TURN_SETTLE 及该阶段 PAUSE）45s、累计 TURN 100s、累计 DRIVE（含停车验证、DRIVE_SETTLE 及该阶段 PAUSE）190s、初始 WAIT 5s、CAPTURE 10s。planner 的距离、永久原点峰值与累计角度预算按本局真实首 ID/初始 pose 冻结，跨 leg 不重置。

最小实施计划：

1. 先增加失败测试，覆盖单/双段路线真实投影、首段停车验证、末次转向后直接采图、暂停不重置预算、四帧 scan 与独立 mode。
2. 新增 `ground_observation_route.py`，继承已验证 `GroundSynchronizedApproachGuard`；保留原传感校验、先前 odom 配对、fresh/PAUSE、局部净空、真实图像三帧与 ROS 适配。仅覆盖路线调度及动态预算，不改旧文件。
3. 新增独立启动脚本；专项测试与字节码编译通过后冻结 SHA256。不开正式 profile，不联网，不提交平台，不把观察完成写成计分成功。

每段在转向静稳后以当前真实位置为局部投影 origin；航向取当前真实位置指向冻结航点，投影前向距离用于终止，直行结束再检查到航点欧氏误差 `≤.02m`。最后一次 TURN 静稳直接进入 CAPTURE，不产生额外直行。每次转向前清除上一段直行局部 origin，永久原点和全程路长/转角不动。

专项测试发现旧短行程 `.025rad` 转向接受窗会在 `1.8m` 长段单凭固定偏角突破 `.02m` 横偏门槛。本版新增裁定：每腿接受窗收紧为 `min(.025, atan2(.01, 腿长))`，误差 `≤.08rad` 时允许最低 `.03rad/s` 角速度请求以收敛；大误差仍沿用 `.35..5rad/s`。最终朝向接受窗仍 `.025rad`，原 `.05rad` 航向硬停及 `.55rad/s` 观测角速度硬停保留。

独立审查补充：转向静稳结束再次验证本腿窄接受窗；漂移超窗回同一次 TURN 重新校准，单次起始时刻、局部转向 origin、累计 TURN 时间均不重置。显式配置更紧的线速上限也约束初/末低档。最终 TURN 静稳前后与 CAPTURE 持续保留观察位 `.02m` 和航向 `.05rad` 硬停，避免清除最后直行局部 origin 后漏掉最终约束。

只保留四个完整真实配对 scan：`start`、`first_leg_end`、`observation_arrival`、`final_heading`（单段路线的两个到达阶段可使用同一真实 scan，但不伪造帧）。其余证据靠 3200 条 10Hz 轨迹与 14000 条实际命令，丢行显式报告。`mode` 和 `survey.approach.mode` 均为 `ground_observation_route`。初始三图不设硬门槛；末尾三张不同原 stamp 的 UAV 图须满足继承的完整有效性与先前真实 `/uav/odom` 配对。

本地验证：控制器最初 12 项因模块缺失失败，launcher 3 项因脚本缺失失败；新增早停车案例又先发现“将零速静稳计入低档连续运动”的错误，已先红后绿修复。独立审查与自主复核增加窄窗静稳漂移、紧线速配置、最终位置/朝向硬停，均先红后绿。最终 route 专项 29 通过；与旧 approach、sync、planner、route 分析专项共 **107 passed**，字节码编译通过。未跑全套，未验证平台控制增益/执行窗，未上传或改旧 v15/v16 包与正式 profile。

冻结 SHA256：

- `src/inspection_mission/inspection_mission/ground_observation_route.py`：`2fb7dacabc0bef12a240fdf93b3783e748dd4d0893ffc3b882638b059996e37f`
- `scripts/run_ground_observation_route.sh`：`34b742da61d20bfe2d97798cdc87ad878ba3c7b9edd2e784336fb95c8a7ea26a`
- `tests/test_ground_observation_route.py`：`a4cfe7135c99378713dfaebd18a2c2d952173cd6fd960f5b54e7d1dcbcad2820`
