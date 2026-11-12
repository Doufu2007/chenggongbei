# 三地面点证据压缩和独立解码审查

2026-11-12。结论：当前生产 `GroundThreePointGuard.additional_events()` 的压缩输出，经真实发送端编码、接收端验证和独立解码能够往返。压缩流、长度、哈希、分片和 session 边界检查通过；本次发现的来源路径碰撞与 JSON 指数溢出已由主控修复，独立负例复核通过。没有修改生产控制、解析器或既有平台证据。

本审查是本地合成传输验证，不证明平台三点得分、真实 ROS 发布、完整比赛完成或网络传输时限。唯一已有官方首点 10 分仍以 v25 官网证据为准，不移作本次三点成绩。

## 审查身份与复现材料

- 只读审查源码：`tools/analyze_ground_three_point_evidence.py`、`tests/test_ground_three_point_evidence.py`，并对照生产压缩函数和 `DiagnosticReporter._encode`、`tools.diagnostic_receiver.validate_event`。
- 最终解码器 SHA256：`e4e0c95ee7485b218d81d708f9e93df34788e475d351625d2ab65d01d0f64acc`。
- 最终生产三点源码 SHA256：`304a0674e35d4815410924e3edc1620901dc719e14901e57ad127ef98a55d1ba`。
- 接收器 SHA256：`1621108f3b1688a8e177240970dd2620525a683aac05ed665119697622211d6a`。
- 发送端 telemetry SHA256：`47a3080096ec0337cd563c6ffd2810dc0b84d42c9d9908d24a3000d255bbeec5`。
- 独立探针：`runs/review/20261112_three_point_evidence/independent_probe.py`，SHA256 `80a8d94e156a74865311cd560dab1e526914e0842bc622b75c60e574d1bd7d2f`。
- 成功探针目录：`runs/review/20261112_three_point_evidence/probe_20261111T224720171738Z/`。目录名使用 UTC，报告日期使用本地时区。
- 结果 `results.json` SHA256：`9cb530257ccaeed76d461a34bce2c7f1e640bc8eb9f761c79cfd680d229e6250`。

最终探针开始和结束时四个受审源码的 SHA 全部相同。现有 9 项测试没有由本审查重复运行；主控随后新增到 12 项的测试结果另由主控保存。本审查独立执行生产路径与额外负例，避免把作者测试通过当作独立复核。

## 真实生产编码往返

普通场景执行一次现有 `ThreeRun` 的合成三点状态机，依次形成三个点的真实生产 Guard 历史；直接调用该 Guard 的 `additional_events()`，没有重写生产 gzip 编码。每一个返回事件都经过实际 `DiagnosticReporter._encode()` 与 `validate_event()`，再由独立 `decode_bundle()` 逆序重组。比较保留 JSON 结构与值，按 JSON 语义统一 tuple/list。

| 检查 | 普通合成三点 | 声明容量压力场景 |
| --- | ---: | ---: |
| 轨迹行 | 4,586 | 12,500 |
| 命令行 | 0 | 30,000 |
| CAPTURE JPEG 图 | 9 张，8×8 | 9 张，800×800 |
| 图像内层分片事件 | 9 | 153 |
| 完整 JSON 字节 | 1,545,287 | 13,080,229 |
| gzip 字节 | 161,880 | 5,356,814 |
| 压缩传输分片 | 5 | 149 |
| 外层总事件 | 9 | 153 |
| 最大实际 UTF-8 信封 | 48,662 B | 48,668 B |
| receiver 验证 | 全部通过 | 全部通过 |
| 独立 decoder | 完整 | 完整 |

普通 fixture 没有真实 ROS publisher，因此 `actual_commands` 为 0；不能称为完整命令实测。压力场景在独立内存对象中显式合成上限行和随机 800px 像素，测的是容量与编码兼容性，不是运动或识别准确率。所有分片低于 65,536B 发送/接收信封限制，完整 JSON 和 gzip 也分别低于解码器 32MiB 与 8MiB 上限。

解码后的 survey 与生产 Guard 观测输出一致，completed_points 与原记录一致。原始 gzip 解压字节另行重算长度与 SHA；普通完整 JSON SHA 为 `1304b8f2f68a46d51a9ec42a8bbab3369fa3f58327bc186690e1e8942a364bcf`，容量压力为 `dd34374ee2e39e9dec63d888b2eca76fa2f4a6ad51ffe06201c531bce95204d1`。

## JPEG 内层和来源保护

补充探针 `check_inner_images.py` 对上述已生成的新 bundle 分别重组 9 张 JPEG，验证每张分片索引覆盖、数量、宽高和 SHA 元数据一致，严格 Base64 解码后重算 JPEG SHA、检查完整首尾与 384KiB 字节限。两种场景共 18 张全部通过；细目见成功目录 `inner_images.json`。这只是本地合成像素的编码证据，不冒称官方原图或原始 ROS 像素核验。

两份新 callback raw 在分析前后的 SHA 一致；两份已解码 bundle 在内层图像审查前后 SHA 一致。探针仅在新建唯一目录写产物，没有覆盖旧 raw、旧解析或旧 XML。

## 拒绝边界与本次修复

20 个拒绝分支全部实际拒绝：缺片、同索引异内容、压缩长度错误、原文长度错误、双 SHA 错误、part_count 超限、原文超限、压缩超限、分片混 session、正文 session 不一致、重复 JSON key、NaN、指数溢出、非 UTF-8、gzip 多流、gzip 尾随数据、长度谎报下的膨胀、两种来源输出路径碰撞。逐项异常与错误原文保存在 `results.json`。

相同索引相同内容的重传正常接受，`duplicate_parts=1`，不改变恢复正文；逆序输入也恢复正确。解码使用 `decompress(..., declared_uncompressed_bytes+1)` 并验证 eof、unused_data、unconsumed_tail 和精确长度，不以无界 `gzip.decompress()` 处理不可信包。独立探针里的 `gzip.decompress()` 只用于已通过有界独立解码的本地生产流复算。

初读发现的两项缺陷均已反馈主控并修复：

1. `analyze_file(input, output)` 原先允许来源就是 `output/bundle.json` 或 `output/summary.json`，将覆盖来源；当前 resolve 后先拒绝。独立临时目录复现两种冲突均抛 `analysis output would overwrite source`，原始字节完整保留。
2. `parse_constant` 不能拦截标准 JSON 数字 `1e309` 转 Python inf。最初独立复现解码成功且 `math.isinf=True`；当前新增有限 `parse_float` 校验，独立同类数据被拒绝为 `nonfinite JSON float`。

## 计分与适用边界

两份 summary 均保留 `official_score=null`、`full_mission_complete=false`；三个 completed_points、局部停止文本、发送标记和完整字节传输均不升级为官方接受或成绩。点历史中的 `official_acceptance` 均为 false。

本审查未发送网络请求。生产结尾的同步上传受 180s 总预算及每请求至多 5s 限制；容量压力的 153 个外层事件不构成在真实网络中一定上传完的保证。缺片时独立 decoder 会拒绝完整性，必须根据正式新局的 raw 和官网结算继续验证。分析产物应继续使用新目录，避免把复用目录中残留的旧 bundle 当成本局结果。
