# 本地诊断证据分析

运行以下命令读取接收器 JSONL，并输出摘要和真实图像文件；命令不联网、不控制机器人、不修改输入：

```powershell
python tools/analyze_probe_evidence.py runs/evidence/20261111/callback.jsonl --output runs/analysis/20261111_v4
```

v5 运行后，把输入替换为对应的新 JSONL，使用新的输出目录。输出为 `summary.json`，以及每条有效 `image_sample` 的 `image_L行号_哈希前缀.jpg`。文件名独立于 topic，不会把 topic 中的斜杠当目录。图片仅严格解码验证后保存 base64 中的原始 JPEG 字节；不会重新编码、补图或生成占位图。摘要记录完整 SHA256、字节数、topic、stamp、frame、尺寸和输入行号。

摘要中的 `launcher_starts/exits`、`probe_starts/finishes`、`samples`、`diagnostic_errors` 均保留输入行号、payload 时间、接收时间和 source。`draw_order_payloads` 保存实际 `/referee/ground_tasks` 原始样本，既不补全也不猜测抽签或坐标。`unknown_fields/unknown_events` 保留未识别字段及事件，便于以后核查版本差异；图像中的未知字段不能覆盖真实来源行号或接收器 source。

供文档生成器复用的稳定 Python API 为 `analyze_evidence(input_path, output_dir) -> dict`，`analyze_file` 为同一实现。除上述明细外，摘要直接提供 `event_counts`、`sample_topics`（实际 sample 或 image_sample 出现的话题）、`launcher_exit_codes`、`startup` 两个启动布尔值、`missing_events`、`missing_snapshot_fields` 和 `source_evidence.sha256`。这些字段只描述已收到的证据。

工具支持完整 `probe_snapshot`，以及 metadata、整节 value、按 topic 拆分的 value 和 samples/topic/sample 分片。合并结果在 `snapshots.assembled`，原分片及其行号保留在 `snapshots.parts`。分片没有完整性清单，所以即使部分字段已经合并，仍标为部分证据。

数量分为两类：`reported_sample_counts` 为各 topic 最近收到的计数报告，不将不同快照相加；全部原报告在 `sample_count_reports`。`sample_record_counts` 为本文件中该 topic 的样本记录数，可能含独立 sample 与 snapshot 对同一 ROS 样本的重复记录，因此不等于 ROS 接收总数或频率。

| status | 含义 | CLI 退出码 |
| --- | --- | ---: |
| `pending` | 文件尚未生成、为空或只有空行，正在等待证据 | 0 |
| `partial` | 有有效证据，但只有分片或缺少关键启动/快照/退出事件 | 0 |
| `collected` | 已收到 launcher_started、probe_started、包含必要字段的完整 snapshot、launcher_exit | 0 |
| `invalid` | 非法 JSON、重复键、非有限 JSON 数值、结构错误或非法 JPEG | 2 |

`collected` 仅描述诊断采集，**不代表比赛通过**。空 snapshot 不算完整快照，缺少的字段在 `missing_snapshot_fields` 中列出。`missing_events` 明列未收到的生命周期事件（包括可能未通过网络发送的 `probe_finished`），`event_presence` 还列各类已知事件是否出现。`official_scoring` 始终为 `unverified`。启动退出码、完整性、裁判接受和成绩分别判断，不能相互替代。

非法内容在 `parse_errors` 标原始行号，已合法解析的其他记录仍保留。非法图像不会落盘；JPEG 必须能解码且尺寸与 metadata 一致。输出目录中的有效图像只以本次 `summary.json` 的 `images` 清单为准；建议每次采集使用独立目录，避免与旧采集文件混淆。

本机图像校验依赖 Pillow；若缺少，可安装 `python -m pip install Pillow`。测试命令：

```powershell
python -m pytest tests/test_analyze_probe_evidence.py -q
```

测试内小图仅为合成测试夹具，不是比赛图像，也不会进入实际采集报告。
