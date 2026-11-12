# 地面观察路线：独立证据入口

日期：2026-11-12。仅完成新模式的离线传输兼容；不构成路线控制验收、平台运行、识别、得分或上传许可。v16/v16b 真实回传仍使用原 `ground_bounded_approach` 入口。

与路线实现代理核对的合同：

- 外层 `mode`、`survey.mode`、`survey.approach.mode` 均为真实 `ground_observation_route`。路线额外上下文在 `survey.approach.route`，原值保留。
- 仅该 mode 允许 trajectory_10hz=3200、actual_commands=14000；每传输分片最多 300 行。旧 survey/short-drive 的 500/1200 及 approach 的 1200/5000 保持原限值。
- `capture_0/1/2` 保留原 JPEG 与 SHA256；按真实 `approach.capture_count` 要求完整重组的 UAV native，整组事件全失仍输出 `missing_capture_phases` 并记 partial。count=0 不凭空要求三图；任何诊断状态都不表示任务完成。
- `survey.scan_records` 原样保留最多四个源 stage：`start`、`first_leg_end`、`observation_arrival`、`final_heading`。单段路线的两个到达 stage 可能引用同一真实 stamp，不编造独立采样。测试覆盖四帧完整 360 束 LaserScan payload 与各自 prior 元数据的字节/数值保真；分析器不据此自行认证扫描几何或关联准确性。

入口：

```powershell
python tools/analyze_ground_observation_route_evidence.py INPUT.jsonl --output NEW_OUTPUT_DIRECTORY
```

共享实现通过显式 `expected_mode`，不改 raw/mode、不猴补全局默认。`kind=ground_observation_route_analysis`；score=null、task_completion/official_scoring=unverified。旧入口不会将 route 数据冒充 approach。

先完成 11 个失败复现，再实现入口与最小共享 mode 分支。五组分析专项 **92 passed in 5.62s**；随后将合成 scan fixture 补为完整 LaserScan 各字段，新路线 11 项再次 **passed in 2.28s**；CLI `--help` 正常。未运行全套，未修改控制源或已有平台证据。合成 fixture 只写 pytest 临时目录，不进入平台 runs 台账。

SHA256：

- `tools/analyze_ground_observation_route_evidence.py`：`4d8e235927b27948a1e8679999c4d5fb45db9986463ba7f22347927a4b4aa600`。
- 共享 `tools/analyze_ground_survey_evidence.py`：`91ed96f2cf5c27f4b8c5f1771dbb35b43a69259b331905a59afb724317ece8ed`。

本轮仍须等路线控制器冻结后，用其实际生成事件作本地合同复核；是否可上传路线、采用何档速度和预算，由真实 v16b 反馈与后续独立审查决定。
