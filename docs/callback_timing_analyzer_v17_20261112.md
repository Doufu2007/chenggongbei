# v17 计时重组与独立审查

日期：2026-11-12。仅补离线解析器与专项测试；没有改控制源码、启动脚本、旧上传包、raw或已有分析目录。

`tools/analyze_ground_motion_evidence.py` 将 top-level `callback_timing` 加入可选字段，并要求收到的值为dict。共享 survey/approach/route 入口沿用此合同；完整性必填集仍不包含该字段，旧数据无需补齐。core/full中的对象原样进入 `snapshots.assembled.callback_timing`，嵌套桶、计数、QoS、慢事件、未知扩展字段与原值一并保留；没有推导CPU归因、修改mode或生成得分。

测试先生成真实 `CallbackTimingRecorder.snapshot()`，覆盖core/full保真、startup可选revision、原文件字节不变、core保持partial，以及null/list/string必须invalid。新增5项先按预期失败：2项丢失字段、3项错误类型被误收。最小修补后，motion/survey/short-drive/approach/route五组必要回归 **97 passed in 5.14s**；主代理负责全套，不在本代理重复运行。

结构样例 `runs/analysis/20261112_approach_v17_timing/schema_fixture.json` 标明 `schema_fixture_not_platform_evidence`，仅供结构检查，不进入平台运行台账；没有据此写平台性能或任务成功结论。

## v17控制接线只读结论

对冻结的 `ground_callback_timing.py`、共享 `bounded_ground_probe.py`接线、专项测试逐段审查，没有发现更改运动、QoS、安全阈值或控制时钟的缺陷：receive包装只读header后调用原处理并finally计时；未复制图像像素；固定输入topic、固定桶与最多12个慢事件限制存储；建订阅时复用已查询到的publisher资料，未加graph查询；spin/tick仍维持原20Hz上限、不追赶补发，watchdog保留。

这些是本地代码与合同结论，不能替代平台计时。解释边界：

- receive从Python订阅回调开始，消息此前已反序列化，包含本队转换、guard处理及其间发生的锁等待/抢占；未独立测锁等待。
- spin_once包含wait/take/deserialize/callback，也包含receive；两者总耗时不可相加，spin wall大可能只是正常等待。
- clock-minus-header受消息源、公开桥、DDS、executor及最近clock样本共同影响，不能单凭此值定位某一环节。
- QoS为实际订阅请求与创建时已知publisher配置，不证明消息延迟或丢失原因；Windows CPU量化为0不能解释为无成本。

只读审查核对冻结SHA256：

| 文件 | SHA256 |
| --- | --- |
| src/inspection_mission/inspection_mission/ground_callback_timing.py | eca410cbe4f66c243c2e60ef4641b2b725ab054bef1b47b63b29df18d8240b16 |
| src/inspection_mission/inspection_mission/bounded_ground_probe.py | ddebbbba13f34e62981202e22f31b214600e81d7e74dde017dfb219a0071c6b4 |
| tests/test_ground_callback_timing.py | 4f5b170fcd5bc59e3c79129d4f22a522898940b54f2b316b8177d811d20f18ec |

本次离线变更文件的哈希另由校验输出记录；分析器不会打入上传运行包，无需为此重打包。真实新局继续使用 `tools/analyze_ground_bounded_approach_evidence.py`，写新分析目录并保留原raw。
