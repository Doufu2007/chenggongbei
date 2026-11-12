# 低高度 Z trim 辨识实现

2026-11-12；依据 `docs/superpowers/specs/2026-11-12-vertical-trim-response-design.md` 与对应 plan 实现。仅新建模块 `bounded_vertical_trim.py`、启动脚本 `run_bounded_vertical_trim.sh` 与专项测试；未打包、未上传、未运行新的平台实验。

## 固定协议

先保留初始候选起飞柱、公开水平 scan、真实双 clock／odom、TF、DDS 和静稳门。确认实际相对上升 `0.10m` 后依次请求：`+.025m/s` 最多 1s、`+.030m/s` 最多 1s、`-.010m/s` 最多 0.30s，随后全零观察。非零辨识期间相对高度低于或等于 `+.035m` 立即终止后续非零阶段。没有新鲜且源 stamp 前进的实际 odom／成功发送的本阶段非零请求，就不推进下一阶段；未取得的数据明确为未知，不重试。

硬上升界 `+.15m`、横漂 `.04m`、倾角 `.05rad`、速度 `.15m/s`、角速度 `.10rad/s`、总路径 `.30m`。首次运动起点只建立一次；**10s 永久运动墙钟涵盖上升、PAUSE、全部 trim、阶段间归零以及回到支撑高度的观察**，不随阶段切换清零。总墙钟45s。参数与协议在新模块固定，CLI 不接受放大动作的参数。

零输出后必须用新鲜实际 odom 确认到达原支撑高度的 `±.005m` 内、实际速度 `≤.005m/s`、角速度 `≤.01rad/s`、位置驻留半径 `.002m`、连续静稳至少 1s，才设置 `support_return_verified=true`。这表示“回到起始支撑高度附近并静稳”，并非独立触地传感器证据或受控降落；不把零命令当悬停。继承的 `observe_s=5` 仅是原 dataclass 中保留字段，本实验回程使用 `stable_s=1` 与永久10s预算，完整证据的 `return_policy.inherited_observe_s_used=false` 明确区分。

## 复用与适配接缝

`VerticalTrimGuard` 继承冻结的 `VerticalResponseGuard`。原来的位姿／四元数校验、逐条危险 scan 检查、重复消息检查、时效、静态柱、路径、初始地面 odom 配对等仍由原实现执行。新 guard 覆盖协议 step、永久运动预算、trim 中断行为、有符号命令记录及诊断证据。

薄适配层先调用原 `create_vertical_node` 建立原来的两个 publisher、七个公开输入、图查询与独立 watchdog，再**显式替换**实例的 `guard` 和 `_publish`，补 `uav_negative_z` 计数。没有修改冻结模块或其全局对象；旧 factory 创建的 guard 尚未运行或输出任何控制就被新 guard 替换。原订阅闭包通过实例的 `self.guard`／`self.receive` 动态访问新 guard，因此复用原回调路径。

非零协议状态保留 `ASCEND`，另用 `phase` 表示 `ASCEND_TO_010`、`TRIM_POS_025`、`TRIM_POS_030`、`TRIM_NEG_010`。这样原回调观察到硬故障或低高度退出 `ASCEND` 后，仍立即请求零。定时阶段达到持续上界时进入 `PHASE_GATE`，先归零；tick 只有在完整门和本阶段反馈／请求记录均成立时才能启动下一阶段。该零间隙也拒绝非法 odom／scan，不能靠下一帧覆盖非法输入后继续。

上升期间 PAUSE 可以重新静稳后继续，但原点与首次运动时刻不变。**任何 trim／PHASE_GATE 中断永久进入 ZERO_RETURN，`protocol_complete=false`，不会重新补跑非零阶段。** 缺数据或缺匹配时仍可能转 STOP；STOP／FINISHED 不会复活非零阶段。正常执行完三个 trim 只标记辨识序列完整；最终仍需实际支撑高度附近静稳才能正常结束。

每次实际发布在原 command lock 及 guard lock 内再次核对实时源龄、阶段期限与允许的符号／幅度。正／负／零分别计数；地面始终全零。地面 publisher 异常独立锁停并把当次 UAV 请求改为零；UAV 非零发布异常后继续单独尝试 UAV 零，不将失败非零发布计为成功。图查询仍在 command lock 外，原独立 watchdog 可归零。

这些是软件检查与归零请求，非硬实时执行器保证；回调／tick 调度延迟和实际停止响应必须在平台证据中观察。已知 v27 全零可能回落，故障归零不代表已经悬停或触地。

## 诊断证据

完整轨迹、命令、水平／下视 scan、TF、原先的长度／双 SHA／gzip分片合同均复用。新命令记录保存真实有符号 Z、状态和 phase；额外保存每个阶段进入的墙钟与 odom stamp、原因、实际推进的 odom 数与首末 stamp、成功非零请求数、`protocol_complete`、中止原因及支撑高度静稳结果。

诊断 section 为 `bounded_vertical_trim_core`／`bounded_vertical_trim_gzip_part`，避免与 v27 混淆。局部 FINISHED、exit0、网络回传成功均非官方成绩；不发布识别、到点、灯光、穿窗或任务完成声明。

## 验证与已修缺口

初始 RED：模块与脚本不存在，23 项失败。实现固定协议后这23项通过。补充覆盖发布边界、负命令发布失败、回程重复 odom 的检查后，曾有一项测试把已过 `.20s` 的输入当作新鲜输入，期望 PHASE_GATE 而实际正确转入零观察；已补上真实新输入以分别测试到期与过期路径，此错误未导致放宽生产门。

自查发现原 guard 的非法输入锁停分支不认识新增 `PHASE_GATE`。两项新回归先复现 NaN odom／scan 在该零间隙未立即锁 STOP，再在新子类中补齐即时锁停，原冻结实现保持不变。

首次实现的历史检查（下述独审修复前）：

```text
python -m pytest -q tests/test_bounded_vertical_trim.py tests/test_bounded_vertical_response.py tests/test_diagnostic_receiver.py tests/test_probe_telemetry.py --junitxml=runs/review/bounded_vertical_trim_20261112/final_affected.xml
# 149 passed（28项trim、49项v27、68项receiver、4项telemetry）
python -m py_compile src/inspection_mission/inspection_mission/bounded_vertical_trim.py tests/test_bounded_vertical_trim.py
# exit 0
Git Bash: bash -n scripts/run_bounded_vertical_trim.sh
# exit 0
```

本地检查证明门禁、协议顺序、记录、适配路径和拒绝行为；不证明 `.025/.030/-.010` 在当前平台的真实响应。本局若两段正 trim 均继续爬升，可能接近 `.15m` 硬界，仍应按原上限停机、保留未完成结果，不通过放大门限强行完成辨识。

冻结源码应继续保持：v27 `1ee83ca7f52a5e1e60de63f778e7caae939b9d4645f510d064bfc57f3016fd78`；`ground_single_point.py` `902424ca5ba3ec208a0028645d377f4ce92ff77e5ee56205c87fac95a9b349cb`；`ground_observation_route.py` `5ab108c4128db8b32c18cee084fce636e8ce80700f18dd37e468006a2a58b945`。新文件最终 SHA 和测试记录见 `runs/review/bounded_vertical_trim_20261112/manifest.json`，独立审查前不打包上传。

## 独审 P1：地面发布成功但延迟返回

独审原反例保留在 `runs/review/vertical_trim_independent_20261112/delayed_ground_probe.py` 及原 metrics 中。负阶段 `3.15s` 开始、`3.45s` 到期；tick 在 `3.16s` 进入地面零发布，地面调用在 `3.51s` 成功返回时，旧适配层仍把 `-.010m/s` 发给 UAV。发布前最初的 wall／阶段／源龄检查不能覆盖后续阻塞；这是软件发布边界缺口。修前源码、测试、报告、manifest 和原反例 metrics 同时快照在 `before_delayed_ground_fix/`，未覆盖历史证据。

新增 7 个回归先全部失败：地面调用期间阶段到期且传感器变旧、仅传感器变旧、永久运动预算到期、实测高度降到禁非零阈值、DDS match 丢失、重新查询 DDS 抛异常，以及重新查询 DDS 自身耗时导致输入过期。最小修复仅在新 trim 适配层中增加后置边界：地面发布返回后重新读取 UAV 实际 matched count，更新 guard，再读取最新 wall，检查 `_wall`、`_condition` 与 `command_allowed`；任何无效结果均把待发 UAV 请求改为零。DDS 查询异常先锁停，仍继续单独尝试 UAV 零。真实成功发送时刻继续由发送返回后的 `wall_clock()` 记录，成功计数保持不变。

修后定向检查为 `7 passed, 28 deselected`（`delayed_ground_green.xml`）；完整 trim 专项为 **35 passed**（`after_delayed_ground_fix.xml`）；新源码及测试 `py_compile` exit0。修前 149 项仅保留为历史检查，不当作本次修后的结果；本次未改冻结 v27、地面代码或脚本，所以未重复整仓。冻结的三份源码 SHA 与上文一致。

用原独审 Transport 重跑同一延迟条件，另存 `delayed_ground_after_fix_probe.py`／`delayed_ground_after_fix_metrics.json`，未修改原独审反例：负阶段原期限仍为 `3.45s`，地面调用 `3.51s` 返回后只发送全零，命令实发记录为 `wall_elapsed_s=3.51, linear_z=0`；状态进入 `ZERO_RETURN/OBSERVE`，原因 `trim_interrupted:clock_stale`。这证明**阻塞返回后不再发送已失效非零请求**；不证明阻塞期间能硬实时归零，也不证明实际 UAV 已经停止、悬停或触地。新源码 SHA 为 `54744be026c23d43bfe8dba5686ffd7d25d9028a8d4eaf06e667501b5984d89c`，等待独审重新确认，仍未打包／上传／取得新平台成绩。
