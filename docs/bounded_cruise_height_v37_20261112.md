# v37 固定初始柱巡航高度：作者交付与证据边界

2026-11-12。设计已由主代理批准；此交付只新增独立文件，尚未上传或进行官方运行。v34 实际响应分析另见 `height_runtime_timing_v34_platform_analysis_20261112.md`。

## 实现合同

`bounded_cruise_height.py` 实现独立 `CruiseHeightLimits` 和 `CruiseHeightGuard`。绝对目标 z=0.85m，绝对硬界1.00m、相对升高硬界0.965m；运动90s、全局110s、累计三维路径2m。上升25s、保持8s、下降45s、全零回支撑8s。保持要求误差±0.005m、|vz|≤0.01m/s，源stamp与墙钟均连续3s。下降到origin+0.025m立刻零输出，|vz|≤0.02m/s才通过下降；原墙钟1s静稳支撑门保留。暂停后不恢复非零，origin与预算不重锚。

其余源/arrival 0.20s、clock lag0.12s、横漂0.04m、tilt0.05rad、总速度0.15m/s、角速0.10rad/s、有限水平scan净距0.641m均保留。固定公开world假设下的静态柱半径0.515296542099m、绝对顶端1.208340688664m，低于公开插件真实max_z=1.8m。几何核验保存为initial_column_validation；不声称已经在平台认证地图一致性。

新scan语义仅在本实验使用：合法完整环内的+inf作为原始无回波保留，不伪造成range_max；NaN、-inf、近有限、布局异常、重复stamp变内容、倒退、陈旧仍拒绝。它依赖固定初始静态柱和无额外动态障碍假设，不能用于未知空间自由判断。旧v31/v34全有限门字节不变。

工厂复用冻结发布链：`/cmd_vel`只发全零，`/uav/cmd_vel`只发Z请求；保留实际DDS最终查询及查询返回后的全部重检、异常归零和独立watchdog。上升远场请求0.05；保持/下降沿用v34局部trim公式，证据明确标注其在0.85m高度尚未实测。没有水平航段、穿窗、灯光、识别、分数或任务完成消息。

## 点云与时序

`cruise_cloud_capture.py` 只订阅公开前向 `/uav/lidar/points` 的PointCloud2；WAIT、ASCEND_CRUISE、HOLD_CRUISE、DESCEND_SUPPORT各最多一完整帧，2MiB/帧、8MiB总量。严格验证row_step*height，保存全bytes及行padding、完整布局/header、实际原始prior odom和实际TF；超限整帧拒绝。只关联odom_stamp≤cloud_stamp、skew≤0.05s的最近有效prior，不用未来位姿或候选外参。缺TF链如实记录；static TF优先保留，所有截限有计数。缺云、采集异常、缺pose/TF均不改变控制放行条件。

回调只复制有界bytes及元数据。hash/base64由显式finalize在STOP/FINISHED、清理零输出并关闭watchdog后调用，编码不持有guard或command锁。新schema `bounded_cruise_height_v1`、section `bounded_cruise_height_*`、mode `bounded_cruise_height`与旧实验隔离；完整gzip分片带压缩/解压双长度和双SHA。

`runtime_timing`保留原v34核心观测器的真实revision `v34_timing`。新增 `capture_runtime_timing.revision=v37_capture` 单列外层全部receive（包括原始元数据工作）、cloud_capture、终态cloud_encode。两套span重叠，禁止相加；记录器仍有2048条trace、32条慢span上限及丢弃计数。未分离DDS readiness/反序列化、源发布墙钟、调度或guard锁等待。

## 作者验证

所有结果均为本机离线合成输入，不是平台实测。

- 初始模块缺失RED：height 29项、collector 34项；后续边界RED与修复前日志均保留于 `runs/review/cruise_height_v37_20261112/`。
- collector最终专项：42 passed，含全8MiB容量、完整尾哨兵、坏布局、真实prior、50ms边界、静态TF容量、坏TF不生成幻影链、元数据有界及锁外编码。
- height最终专项：34 passed，含独立数值、静态包络、+inf/坏值、双3s保持、下降同tick联合门、预算/硬界、DDS迟返/异常零、完整云穿过真实工厂、gzip双长度/双SHA恢复、采集异常隔离，以及新cloud回调阻塞时watchdog继续全零。
- 必要组合回归：289 passed（13.52s），覆盖当时33项height、42项collector、旧height/timing/vertical/trim/ground/launcher。随后只增加反馈证据字段及一项watchdog测试，最终34项height全部通过；不重复宽测。
- 新模块及测试py_compile、独立launcher的bash -n通过；launcher实际子进程退出码0/17/setup29均验证。
- 完整合成动态响应轨迹可完成上升0.85、全inf高处scan、保持、下降和静稳，t<90s、path<2m；只证明本地协议在该模型可执行，不证明真实高处trim响应。

作者联调另捕获并修复三处有实证问题：新元数据工作缺少独立计时；意外collector异常传播；工厂deque轨迹与collector列表接口不匹配。每处都有修复前失败日志与修复后通过日志。

## 冻结与审查

manifest列出全部交付文件的SHA256、六份冻结父文件的期望/实测SHA、测试命令与日志路径。非作者审查结论将在独立review文件落地；此报告不先行宣称审查通过。

下一次真实运行仍需验证0.85附近误差/速度/双3s保持、全高度trim响应、scan由有限到无回波的变化、下视TF/返回、真实累计路径/横漂/姿态、下降与支撑，以及四阶段完整云实际取得情况。没有官方结果或完整证据时，不宣称通过、得分或任务完成。
