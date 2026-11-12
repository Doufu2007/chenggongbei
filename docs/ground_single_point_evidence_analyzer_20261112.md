# 首点证据解析器

2026-11-12。新增显式入口：

```text
python tools/analyze_ground_single_point_evidence.py INPUT.jsonl --output NEW_OUTPUT_DIR
```

只接受原始 `mode=ground_single_point`、`read_only=false`、`official_scoring=unverified`。复用原 native JPEG/各自真实前序 pose、三个 CAPTURE、轨迹和实际 command series 合同；本模式显式沿用 route 的 3200/14000 行容量。旧 route/approach/survey/short-drive 模式仍按各自合同解析，不将旧日志重标为首点。

新增 `single_point_core` 专用分片解析，保留 session、完整候选 plan、epoch、match_results、recognition、attempts 和收到的 recognition_status。专用片的完整原 data 另存 `single_point_observations`，未知字段留 `unknown_fields/raw_parts`。与历史 `survey.approach.single_point` 的副本不一致，或专用 core 之间不一致，均记录 conflict；缺专用片即 `missing_single_point_core=true`，只能 partial，仍保留已有嵌入证据。

`attempted/sent` 是本队派发尝试/返回记录；端点 `actual_publish_attempt_count/actual_publish_return_count` 则记录真正的 DDS publish 调用/返回，预检查失败为零次调用。解析器将收到的 `recognition_status[].data`、原长度、截断标记与原 SHA 原样保留；即使字符串看似 `{"accepted":true,"score":10}` 也不解析为 ACK 或成绩。顶层 `score=null`、`official_scoring=unverified`、`task_completion=unverified` 保持；source 不允许伪称 official acceptance 或 full mission complete。

14 个新增行为用例先实见失败，再实现接线；随后使用真实 guard emitter 增补 2 项，发现本解析器初稿误把 `epoch_invalid` 当 bool，而实际为 `null` 或错误原因字符串，已用两项实红纠正。保留 `analyzer_emitter_initial.xml`，没有将该解析器缺陷说成控制器缺陷。

最终 16 项加旧 motion/survey/approach/route/short-drive 共 **113 passed / 6.55s**。最初红 XML 为 `runs/review/20261112_route_v23/analyzer_single_point_initial.xml`，最终绿 XML 为 `analyzer_single_point_final_v2.xml`。这些是合成传输及真实 emitter 的本地测试，不是平台实测。项目整套回归由主代理在所有单点文件冻结后统一执行。

同日传输独审修订：可选 `matcher_failures` 列表规范保留失败 job、错误与 `job_was_executed=null`；成功 `match_results[].job` 的原 topic/frame/stamp/pose/CameraInfo/raw SHA/encoding/stride 不另作变换。为避免重复大块超限，当前源只在专用 core 发完整单点块，survey 内以 `single_point_transport={section:single_point_core, session_id:...}` 引用。解析器支持此引用并校验 section 与 session：错 section 为 invalid，会话不一致显式 conflict；仍兼容旧嵌入块。

修订后 **21 项单点解析测试，加旧模式共 118 passed / 7.70s**，XML 为 `runs/review/20261112_single_point_transport/analyzers_final.xml`。引用校验的两个初始红例在 `analyzer_reference_initial.xml`；传输源修复的独立证据见 `docs/control_review_ground_single_point_transport_20261112.md`。

v24b 实测后最小补充：大快照 fallback 的 `section=survey, topic=顶层字段, value=原值` 现显式保留为 `survey_fragment_observations/survey_fragment_assembly`，验证 partial 与原 mode，重复冲突不静默覆盖。不把这类片段计作 ground_motion_full，不覆写原 survey_metadata；因此只有片段而没有 full 事件时仍为 partial。`camera_liveness` 可选 timing/null 也规范保留，不用当前相机活性时间改写旧曝光。

新增 5 项 fallback 和 2 项 camera_liveness 用例，分别先红后修；单点现 **28 项**，加旧模式共 **125 passed / 8.34s**。XML `runs/review/20261112_single_point_v24b/analyzers_final.xml`，本局独立复核见 `docs/platform_single_point_v24b_independent_20261112.md`。
