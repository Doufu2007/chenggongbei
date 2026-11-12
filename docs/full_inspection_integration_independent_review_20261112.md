# 完整任务整合定点独立审查

最终定点组合 19/19 通过（1.76 s），日志 `runs/analysis/20261112_v38_visual_root_cause/integration_latest_heartbeat_review.txt`。本报告覆盖地面返航交接、完整参考库和完整视觉结果记录；不代替飞行控制模块的完整独立审查或平台实测。

## 地面零发布失败必须阻止 flight factory

独立脚本 `independent_integration_review.py` 用真实 `create_return_three_point_node` 先完成 `RETURN_SETTLE`、真实成功零返回与 `return_completion`，再让清理阶段的 publisher 抛异常。真实 axis adapter 将异常转为 `False`，初版 `_dispose` 忽略该值仍创建 flight，初审复现失败见 `integration_initial_review.txt`。

父代理修复后，每次清零必须显式返回 `True`；存在 `axis_publish_attempts` 时还核对本轮新增尝试、零值、成功返回及无 error。清理仍完成 watchdog/节点释放，任一次零失败最终抛错并禁止 flight。真实 factory 反例复验通过。真实 v40 `return_completion` 同时通过飞行 `validate_ground_return` 正控，返航 proof 与 handoff 相容。

最新 `_spin_with_progress` 包装原 tick，决策完成后最多每 5 秒发一次旁路 heartbeat；发送异常不修改原决策，finally 恢复 tick。只读复查及最终定点组合通过。

## 完整十模板目录

目录只追加两张已校验公开图，旧八图保留。缺图、损坏与错误 class manifest 均拒绝；独立错误 class 反例通过。原几何门三张真实 JPEG 的证据见 `v38_visual_root_cause_20261112.md`。

## 189 KB 结果的记录修复

实际首帧十模板 worker 结果默认 JSON 为 189,780 B，超过 `DiagnosticReporter` 单事件 64 KB；原 recorder 吞掉编码异常后只计数，未留下完整对应。初审失败后，父代理明确授权本审查代理修改 `ground_match_result_recorder.py` 与专项。因此该项是本人作者实现，由父代理非作者阅读审查；不是本人对本人实现的独立审查结论。

owner 的 `poll()` 返回原列表和原元素对象，仅 deepcopy 稳定快照入有界队列。后台 daemon 完成 JSON、gzip、双 SHA256、base64 分片与发送；每片 32,000 字符，包含 recording ID、索引/片数及压缩前后长度和 SHA256。最多 18 个结果，pending queue 2，后台序列化 raw 上限 4 MiB；阻塞/失败/超限均诚实计数且不改变控制返回。关闭与 flush 有界，worker close 仍委托。

专项经真实 `DiagnosticReporter._encode` 和接收校验器验证分片，并解包恢复全部 JSON；真实 189 KB 回放结果亦完整还原。阻塞 sink 时 `poll` 八次小于 0.1 秒、队列不超 2、drop 计数可见，原对象不变。后台 4 MiB 检查不能声明任意恶意对象 deepcopy 的内存硬上限。

最终冻结校验值由分析目录 `final_review_manifest.json` 给出。初审 RED、第一次 GREEN、最终 heartbeat 版本日志分别保留，未覆盖失败证据。
