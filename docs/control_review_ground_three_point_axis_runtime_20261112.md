# 轴向三地面点运行器独立审查

结论：**当前候选不放行**，两项真实发布边界反例已复现；需要在新 axis 适配器修复后复审，保持冻结父源不变。

审查源码 SHA256：`5ece68ce5b1460e0bb419bdc87b18b9212be749ebfa6ea35a368003ee7f24454`。启动脚本 SHA256：`3b5251e1f4b5dcc8d48d23c59c6ce102b422af1e2c031fa5ae05f0e0032bdfd4`。读取了规格、实现报告、限值/轴向 planner、三点/单点/原始传输继承接缝；没有修改生产代码、上传或运行作者的 172 项检查。

独立探针及完整结果：`runs/review/ground_three_point_axis_independent_20261112/independent_probe.py` 与 `independent_metrics.json`。共 5 个独立探针，3 个通过、2 个反例。

## 阻断 1：图查询后实际 DDS 匹配丢失仍发布负 X

真实 factory→callbacks→tick 路径中，tick 首次轻量匹配读取为 1，刷新图谱时 `/cmd_vel` 端点也读取为 1。随后查询其他输出 topic 时，实验将实际 `/cmd_vel` 匹配降为 0；此时原适配器不再读取实际匹配，guard 保留缓存 1，仍发布 `linear.x=-0.03`。

探针观察到 `actual_match_at_publish=0`、`guard_cached_match=1`、`state=DRIVE`，实际非零 publish 已发生。应在最终非零发布前重新读取实际匹配，且该查询返回后重新核对当前预算、phase 与传感器新鲜度，防止新的查询耗时跨越边界。不要依赖描述性图谱缓存。

## 阻断 2：负 X publish 异常逃逸且本轮不尝试归零

探针令真实 `/cmd_vel` publisher 对负 X 抛 `RuntimeError`。异常从继承的 `original_tick()` 逃逸，外层视觉 coordinator 的 try 块在其后，不能捕获该异常。观察到的唯一 publish 尝试为 `(-0.03,0)`，guard 仍处于 `DRIVE`，本 tick 没有独立零命令尝试。

应在新适配器中捕获发布异常，记录真实失败、吸收锁停，并独立尝试 `/cmd_vel` 零命令；持续归零尝试与 watchdog 不得因一次异常中断。此反例的独立 tick 夹具关闭了异步 watchdog，以检验当前回路本身的保证；它不证明平台会发生这种异常，也不允许依赖 cleanup 偶然补救。

## 已通过的独立核查

- 实际 factory 明确替换 axis guard；订阅回调与实际 tick 使用新 guard，视觉 coordinator 也指向新 guard。真实 publish 为 -0.03，证据记录同一负值；向南 1mm 位移在机体朝北时得到正 1mm 进度，drive_yaw 保持 +pi/2。
- TURN_SETTLE 在 X 漂移 8mm 后重新瞄准 body 朝向 `1.5618078048292405rad`，转向误差取负并重新进入 TURN；开始下一 forward 操作先清 reverse flag，forward 请求恢复正值。
- 使用额外顺序 `ground_1→ground_2→ground_4` 完成一轮本地三点合成反馈。首点、当前视觉后的 entry 和点间 handoff 都得到轴向 operation；旧 epoch 匹配拒绝，epoch 为 2、5、8；识别/到点声明逐点对应目标。永久起点与创建墙钟不重置，累计路径/旋转/分组时间单调累计。最终路径约 9.943m、旋转约 25.623762rad，28rad 合同保持固定。
- 只读检查确认 `_plan_point`、`_begin_entry`、`_handoff` 三处使用新 planner，`DRIVE_REVERSE_TO_XY` 的目标距离保持正值，heading 检查仍对机体方向，测量投影单独取负；未改飞行、灯和全任务完成输出。

以上仅是本地软件接线、反馈约束与内部视觉合同证据。反向平台标定仍须单独通过，当前候选地图和几何余量仍待真实验证；即使独审修复通过，也不能代替负 X 实测或官方计分证据。

按主控要求，本轮先结束并释放代理槽；只读点云 collector 延后，不会与修复作者争用生产文件。
