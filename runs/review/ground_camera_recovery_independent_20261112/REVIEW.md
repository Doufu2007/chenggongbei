# v36 相机恢复独立定点审查

2026-11-12；审查者 `/root/axis_final_review`。结论：本轮未发现阻断问题，可继续 v38 本地软件审查。未改生产源码、未上传、未声称平台验收。

冻结对象为 `ground_three_point_camera_recovery.py`，SHA256 `2c7addc67afda66e4aab53b1796dc0537756fb6ffd907647eb998a6699b809ee`。独立重算冻结清单中的 14 项及 v38 模块/启动脚本共 16 项，全部吻合。v36 factory 构造完成后的 20 个顶层 AST 语句与冻结 axis factory 完全一致，最终 DDS 检查、锁、零发布回退及 transport 计数未发生语义漂移。

独立窄探针 5/5 通过，见 `v36_probe_results.json`。未重跑作者 22/22、108/108 全套。

- 相机墙龄精确 0.500s 仍接受，0.50001s 转 PAUSE，输出零。
- 同时墙龄旧但 clock-image < -0.12s、未来 arrival、缺失图像均 STOP，不冒充可恢复正向缺口。
- 相机恢复后墙钟推进 1.2s、源钟仅推进约 0.23s，仍为 PAUSE；两钟均满足原 1s 停稳后才建立新 epoch，清除旧 captures/jobs/matches。
- 测试直接填充真实 factory 构造出的 guard，不在运行时替换 `node.guard`。最终 DDS 查询期间制造 0.530038013s 相机缺口后仅发布零，实例保持不变。
- 注入工厂若改变 started_wall 或 frozen_limits 对象身份，会被拒绝。

源码复核确认：仅 CAPTURE/RECOGNIZE_WAIT、recognition 尚未 attempted、有效方向的正向相机超龄进入恢复；REPORT_WAIT/attempt 后、非法消息/反序继续硬停；移动阶段新增 header 校验不以 clock-image 负差新增停止。每点一次 recapture、原默认全局三次恢复、原 control 0.20s/相机0.50s/exposure-prior0.05s门均保留。默认值与 v38 额外资源策略分别存在独立模式中。

边界：探针使用真实 guard 与本地 ROS transport stub，证明软件分支与最终发布决策；不证明官网调度、实际传感器恢复概率、碰撞情况或计分接受。源码摘要与实际图像匹配结果也不由本轮探针重新验证。

复现：`python runs/review/ground_camera_recovery_independent_20261112/independent_probe.py`。该脚本仅写本轮 review 目录。
