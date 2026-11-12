# v31 有限高度反馈实现记录

2026-11-12。root 明确批准的设计／计划：`docs/superpowers/specs/2026-11-12-bounded-height-hold-design.md`、`docs/superpowers/plans/2026-11-12-bounded-height-hold.md`。仅新增模块`bounded_height_hold.py`、脚本`run_bounded_height_hold.sh`与专项测试；未打包／上传／运行新平台实验，v27、v29及地面控制源码不改。

## 来自实测的起点与新增假设

已完成独审的v29报告为`docs/vertical_trim_v29_response_analysis_20261112.md`。内部观察窗确认在本局初始近水平低高度条件中，`u=.025`对应vz中位`-.0039m/s`、`u=.030`对应`+.0061m/s`；两点局部插值得到候选trim`.02695`与局部增益2。v31固定P式`clip(.02695+.5*clip(.5*(target_z-z),-.03,.03),.01,.05)`是root事先指定的**待实测反馈**，不是v29已经验证的悬停，也不是可外推任意姿态的全域模型。没有积分、自动调参或负Z精细下降。

## 软件阶段和边界

新`HeightHoldGuard`继承冻结v27的位姿、scan、静态柱、DDS、TF及双时间检查；`HeightHoldLimits`在新模块独立批准永久25s，先用旧类验证其余原限制，旧`VerticalLimits`仍拒绝25s。始终以首次原点和首次运动时刻计预算，累计路径不重置，硬相对升高`.15m`、横漂`.04m`、倾角`.05rad`、速度`.15m/s`、角速`.10rad/s`、累计路径`.30m`、fresh`.20s`和总45s原值不变。

上升`+.05`至实际`+.10m`后先归零，下一tick通过当前门与真实本阶段odom推进／成功请求记录才进入HOLD。HOLD目标仍`+.10m`，z误差≤`.005m`与`abs(vz)≤.01`必须连续至少3s；驻留同时要求wall和实际odom源stamp跨度≥3s，不以重复stamp凑时间。每条odom若高度／vz越界即时复位驻留，不能由下一帧覆盖坏帧。HOLD本阶段最多8s，未达到归零并永久中止。

HOLD完成后先归零阶段门，再进入目标`+.02m`的同式反馈下降。实际相对高度≤`.025m`即终止非零；只有同时`abs(vz)≤.02`才标下降段通过，速度未满足记未完成并归零，不继续压低。阶段gate进入下降的同一tick也检查此联合门，不能先发一条低高度非零。最终仅全零观察，实际支撑高度±`.005m`、全速度≤`.005m/s`、角速≤`.01rad/s`、位置半径`.002m`、实际至少1s静稳才正常本地结束。非零阶段的任何pause永久转ZERO_RETURN，不自动恢复；本地状态、识别／得分均分开，不发布任务声明。

## ROS接缝与发送证据

复用冻结factory生成两个publisher、七输入、图查询和watchdog，随后显式替换guard及`_publish`。地面恒全零；UAV仅正Z或全零。发送前和可能阻塞的ground发布返回后重新检查最新时间、phase期限、源龄、DDS匹配与反馈允许值。若阻塞期间收到新odom使当前请求变化，当次归零且永久终止，返回decision与真实输出一致。DDS查询异常、ground发布异常锁停仍独立尝试UAV零；UAV非零发布失败后尝试零且失败请求不计成功。

成功请求在publish返回后用真实wall记录，附phase、target_z、实际z/vz、误差及odom stamp。存在阻塞时，这只能保证返回后拒绝失效非零，不能保证阻塞期间硬实时归零，不能由零请求推断实际悬停或触地。原命令cap1200、轨迹cap4096在本实验25s/50Hz范围足够，任何drop仍显式记录。

完整gzip分片和双SHA合同沿用，section独立为`bounded_height_hold_core`／`bounded_height_hold_gzip_part`，记录HOLD驻留证据／复位、phase推进及中止。当前`forward_cloud_collection=not_attached`、`forward_cloud_control_used=false`：独立collector由另一代理后挂，当前候选不宣称已采齐完整云。控制依赖的场地仍是明确声明的初始固定静态柱，不做前飞或穿窗；FOV缺口按独立报告后续补证据。

## RED、修复与验证

- 首轮39项因新模块／脚本缺失而RED（`red.xml`），最小实现后37过；其中两项测试误写初始运动时刻1.1s，实际初始静稳在1.0s首次满足，已纠正测试并保持永久起点未变。
- 两项新增反例先RED：HOLD坏odom在tick前被好帧覆盖会假完成；下降gate切入时低于阈值仍首发非零。已分别在逐odom复位和gate后的联合门修复（`continuous_and_gate_red.xml`）。随后41项GREEN（`green.xml`）。
- 新增发送边界反例：ground返回期间odom改变使反馈值不同，实际发零但原factory返回旧非零decision；先RED（`changed_feedback_red.xml`），新适配层即时中止反馈并让返回状态与零一致。

最终必要相关检查：

```text
python -m pytest -q tests/test_bounded_height_hold.py tests/test_bounded_vertical_trim.py tests/test_bounded_vertical_response.py tests/test_diagnostic_receiver.py tests/test_probe_telemetry.py --junitxml=runs/review/bounded_height_hold_20261112/affected_green.xml
198 passed（height42、trim35、v27 49、receiver68、telemetry4）
python -m py_compile src/inspection_mission/inspection_mission/bounded_height_hold.py tests/test_bounded_height_hold.py
exit0
Git Bash: bash -n scripts/run_bounded_height_hold.sh
exit0
```

仅主协议和其依赖检查，不把本地软件逻辑称为平台高度保持成功。原源码SHA核验见manifest；当前独立审查待完成，未打包上传。正式平台完成标准是原始轨迹证实3s HOLD误差带、下降联合门、支撑高度静稳以及预算没有越界；即便全部通过，也只是有限实验而非巡检计分成功。
