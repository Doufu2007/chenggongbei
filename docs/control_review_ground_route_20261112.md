# 首点外观察路线：独立只读审查

日期：2026-11-12。审查新 `ground_observation_route.py` 与独立启动脚本；不改生产源码、测试、旧包或平台证据，不重复全套。修复由实现代理完成。本报告不放行平台长路线提交。

## 两项实际发现及处置

**1. 长腿转向静稳后漂移可突破对准合同，之后中途横偏锁停。** 原版只在进入 TURN_SETTLE 时检查 `min(.025, atan2(.01,腿长))`。独立重放 ground_2 的第二段 1.8m 航程：进入静稳后真实 yaw 漂移到 `target+.018rad`，仍低于 `.05rad` 硬限且可满足静稳驻留；原版随即输出 `.10` 直行，到前向投影约 `1.114819m` 时横偏 `.0200689m`，被硬停。理想无惯性的原测试未覆盖此路径。

实现代理先加回归复现，再在静稳出口重验实际 yaw。超窄窗时返回 TURN，先输出一帧零速，重新记录局部初始角误差；`turn_started_wall`、`turn_origin`、累计组的 `phase_started_wall` 均不重置。修后回归既验证这些不变量，也续跑同一局到 CAPTURE；定点复核通过。原有 `.05rad` 航向及过转硬限仍在。

**2. 显式更紧线速上限未约束初/末慢档。** 原版接受 `RouteLimits(linear_command_mps=.01)`，初档和末段慢档仍固定返回 `.03`；独立重放首 DRIVE 即可看到配置 `.01`、请求 `.03`。默认 `.10` 不受此问题影响，但违反可选配置合同。

实现代理将慢档改为 `min(.03, configured)`，档位证据按真实请求记录，不增加新档位。完整低速单段回归与本次定点检查通过，默认仍只用 `.03/.10`。

## 关键状态与预算结论

- 真实首点与初始 pose 冻结候选路线；跨 leg 只清上一段直行局部 origin/yaw，永久起点、累计路径、累计转角、恢复次数保留。每段按当前真实位置指向冻结航点，实际投影结束后再检查航点误差 `<=.02m`。
- PAUSE 保持原 phase，各阶段墙钟继续累计。TURN→TURN_SETTLE及 DRIVE→停车验证→DRIVE_SETTLE属于同组，不重启累计预算；只有组转换才结算上一组。重对准不重启单次45秒或累计100秒转向预算。
- 首段实际投影达到 `.05m` 时进入零速停车验证；累计路径增量 `>.01m` 硬停，之后需静稳满1墙钟/仿真秒。初始慢档的连续1秒不能借停车驻留或PAUSE累积；末段最后`.10m`回慢档。
- 最后一段静稳后递增 leg_index、清 drive_origin、进入纯 TURN；最后转向静稳通过后直接 CAPTURE，不再创建 DRIVE。三张有效原始 UAV 帧与 prior pose 约束继承已审查的同步 guard。
- 全局300秒、累计直行190秒、单TURN45秒/累计TURN100秒、CAPTURE10秒分别存在；规划器 metadata 中较早240/165/60秒仍标 provisional，实际控制合同在 RouteLimits 和 route runtime status 中明确。不能将规划元数据的预分配与当前运行上限混为一组已测耗时。
- 新 route mode 与观察记录器容量3200/14000一致，仅发布地面 `/cmd_vel`；其余任务、飞行、灯光接口保持静默。launcher 从上传源码路径执行模块，保留 ROS setup 与主程序非零退出码。

## 验证的实际范围

必要专项覆盖跨leg、PAUSE、真实反馈硬停、末次纯转向/采图及launcher；两项发现的修后定点回归均通过。独立脚本直接用新 controller 和真实 guard 解析器驱动 ground_2 合成反馈到最终三图，并记录每次请求，事件只写临时目录、来源标注 `synthetic_guard_fixture_not_platform`。

该合成合同生成12个事件，最大26,424B，显式 mode为 `ground_observation_route`；新离线入口解析无 parse/assembly error，三张 capture 与四个 scan stage 原值保留。结果 partial，因为该临时合同没有构造 ROS adapter 的完整 snapshot/lifecycle；没有伪装为 collected 或真实平台 run，没有写比赛台账。

这证明当前代码与传输合同能在受控本地反馈下执行预期分支。它不证明 `.10` 请求的真实平台响应、提高档位后的停车包络、紧窄转向请求的可达性、300秒平台执行窗口或正式识别/裁判计分。v16b仍须先提供真实同步修补反馈，长路线是否上传由主代理根据新证据另行决定。

## 最终冻结复核

实现代理另补了最终 TURN 后位置与 CAPTURE 持续合同：进 CAPTURE 前重验冻结观察位误差 `<=.02m`，CAPTURE 持续检查同一位置及最终目标航向 `.05rad`。这补足最后纯转向清除 drive_origin 后不再有上一腿直线航向检查的边界。最终三项新增检查与本审查两项修复，共5项定点回归 **passed in 2.30s**；未为此重复完整测试。实现代理的107项必要合并回归为另一份验证记录。

最终逐文件哈希已与实现代理重冻值一致：

- `ground_observation_route.py`：`2fb7dacabc0bef12a240fdf93b3783e748dd4d0893ffc3b882638b059996e37f`。
- `run_ground_observation_route.sh`：`34b742da61d20bfe2d97798cdc87ad878ba3c7b9edd2e784336fb95c8a7ea26a`。
- `test_ground_observation_route.py`：`a4cfe7135c99378713dfaebd18a2c2d952173cd6fd960f5b54e7d1dcbcad2820`。

冻结版本未发现其他阻塞本地执行的源码问题；平台档位响应、停车及时间窗口限制仍按上节保留。本报告不替代真实练习证据。
