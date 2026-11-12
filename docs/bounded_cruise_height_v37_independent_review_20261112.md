# v37 固定初始柱巡航高度独立审查

2026-11-12，非作者审查者 `/root/axis_final_review`。最终结论：通过本轮定点软件审查，无剩余阻断。此前复现的一处点云阶段归属问题已由作者修复并独立复验；本审查未修改生产代码、未上传、未启动仿真，未改 v38。

最终审查源码为 `bounded_cruise_height.py` SHA256 `65408ea8d17f2cb1629eef1a1509b68e547ed1392c2c2c086fbd47ca2ee1b96d`、`cruise_cloud_capture.py` SHA256 `73f7e51481880309f9ec416ca7cf60009ab37e3d91770e8d40c42a7262c5c902`、启动器 SHA256 `68a243a8919cb6d84982dddb9fb95c7313f64b7ed772f3212bb01edd71b412b8`。新交付8项与冻结父6项分别重算SHA吻合；最终作者manifest仅更新文档审查状态，生产与测试保持不变。

## 已关闭的问题

原 `b992a5dd...dc2b01` 版本的 cloud callback 仅用当前 `phase` 分槽。真实工厂在 HOLD 入口 source=11.3s / wall=1.3s 后，可复现两种误标：STOP 后 wall=1.32 收到 source=11.32 的云仍占 HOLD 槽；上一阶段延迟云 source=11.22 / 真实prior=11.2 也占 HOLD 槽。控制没有越界，但“四阶段采样”证据会错误补齐，因此作为同一阶段归属问题阻断了初审。修复前独立证据保存在 `runs/review/cruise_height_v37_20261112/independent_probe_results.json`。

作者修复后在 guard 锁内仅取 state、phase、阶段入口源stamp和入口墙钟标量；点云解析/复制仍在控制锁外。WAIT只接受WAIT状态，活动阶段只接受ASCEND状态，终态/门阶段/非法source/早于阶段入口source均拒绝，不占槽；固定原因计数与32条有界明细进入终态证据，每个合法帧附实际 `stage_context`。

## 独立定点复验

复验4组全部通过，见 `independent_recheck_results.json`，未重复作者36项height、42项collector及289项关联宽测。

1. 重算8份新交付与6份冻结父SHA，全部一致。最终只更新作者报告与计划文档状态后另做最终manifest核验，不重复行为测试。
2. STOP后的云拒绝，`terminal_state=1`，控制状态保持STOP，无槽被占用。
3. source=11.22的上一阶段云拒绝，`source_before_phase_entry=1`；随后合法HOLD云11.32仍被采入，记录state=ASCEND、入口source=11.3、真实prior=11.3、skew=.02。终态编码时guard及command锁均未持有；gzip分片重组后双长度、双SHA、24字节完整原帧及stage context/拒绝计数保持一致。
4. 真实factory进入下降后，在地面零发布回调中将新鲜odom更新到origin+.024m、vz=-.019m/s；最终两次UAV请求均为0，进入ZERO_RETURN且descent_verified=True。发布前后交接未把旧非零决策带出。

复现命令为 `python runs/review/cruise_height_v37_20261112/independent_recheck.py`。初审复现脚本保留作历史记录，不应将其原始finding结果解释为最终版本仍失败。

## 源码审阅范围

独立固定limits为绝对目标.85、绝对硬界1.00、相对硬界.965、运动90s/全局110s/path2m、阶段25/8/45/8s；origin与预算不在暂停或阶段间重置。HOLD以源钟和墙钟均3s、误差±.005及|vz|≤.01判定，坏odom观测即时清dwell；下降rise≤.025立即零，速度≤.02才记通过；原支撑墙钟1s静稳门保留。暂停后进入永久全零回支撑，不恢复非零。

静态柱/完整机体包络、原fresh/tilt/drift/speed与有限scan>.641门保留；+inf仅按已批准固定静态柱无回波语义保存，不伪造maxrange。工厂继承链仍通过动态 `self.guard` 执行最终DDS、ground发布返回后的重新检查、失败归零和独立watchdog；新回调不会在复制云或编码时持guard/command锁。原v34核心时序与v37外层receive/cloud计时分列且标重叠，不相加。

collector保存完整row_step×height与行padding，每帧2MiB/总8MiB，超限整帧拒绝；真实prior仅选stamp≤cloud且skew≤.05，缺pose/TF如实标记，TF来源/头/外参保留，不以候选TF补链。四阶段云与计时均仅为观测，不成为控制许可。终态后编码及独立schema/section隔离、gzip双长度/双SHA路径已审。启动器无仿真/裁判启动，原shell错误退出码经trap保留；作者报告退出0/17/setup29专项通过，本轮已查对应测试与shell实现，未重复执行该无疑点专项。

这些证据只支持本地软件合同。0.85m处trim响应、实际HOLD双3s、地图与TF对应、真实累计路径/横漂、下降与支撑及四阶段点云是否取得，仍须由实际运行完整回传验证。本报告不声称平台通过、官方计分或任务完成。
