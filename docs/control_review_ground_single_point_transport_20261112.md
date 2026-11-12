# 单点传输、ROS 接线与源绑定独审

2026-11-12。审查最终 `ground_single_point.py` SHA256：

```text
a07efbda404b2596f85cc7e9f9d8e2fd54ab442c16d21319658cf6169c6b16d6
```

本轮独立复核 **10 passed / 7.29s**。原始四项反例均已在生产源修正；当前范围没有新增离线阻断。结论只适用于本地传输、源码绑定和模拟 ROS 适配器，平台实测、裁判接受、单点得分与完整比赛完成均未据此确认。

## 已复现并修复的问题

| 原行为 | 最终行为与验证 |
| --- | --- |
| 专用 core 可发送，但完整 provenance、三个结果及八条有界状态令重复嵌入的 survey_metadata 超过 64 KiB，report_final 只警告并跳过该项 | survey_metadata 只留指向专用 core 的 section/session 引用，保留其余 approach/scan/route。所有真实 hook 事件均通过 DiagnosticReporter._encode；同反例的专用 core 为 46,883 B，survey_metadata 为 25,290 B。 |
| 成功 match_results 只留 JPEG SHA/pose，worker 输入中的原 CameraInfo、frame/topic、raw SHA、encoding、stride 只存在内存 expected_jobs | 追加去 JPEG 字节、去内部 generation 的完整原 job 元数据；独立检查三个曝光的原 stamp、各自 prior pose、CameraInfo、frame/topic、ROS 原字节 SHA、stride 均不改写。 |
| 无订阅者预检查失败，publisher.publish 未调用，但 actual_publish_attempt_count 被记为 1 | 派发锁存 attempted 与真实 DDS 调用计数分开。无订阅者为 0 调用/0 返回；成功为 1 调用/1 返回；publish 抛异常为 1 调用/0 返回，均不重发。 |
| 当前 context 的 worker 错误分支只 stop 并留下 reason，丢失真实 job 与执行未知事实 | 在有界 matcher_failures 留原 job、error、wall、context、dispatched、executed。active 子进程意外退出的执行状态 null 原样保留，不变成 false。 |

初始生产行为复现为 **4 failed / 4 passed**，位于 `runs/review/20261112_single_point_transport/independent_initial_behavior.xml`。更早 `independent_initial.xml` 包含一个审查测试话术预期写错的额外失败，已纠正为调用正式 `recognition_phrase`；该项不是生产缺陷，没有将它纳入四项问题计数。

## 最终检查范围

- 实际 800×800 ROS 图像，bgr8、rgb8 与 mono8，包含带行 padding 输入；JPEG 派生明确为 quality 75、无 resize。
- 三个成功结果使用真实模板 manifest 的完整 provenance，保留真实源 SHA 与原 CameraInfo 绑定。测试的目标类别是显式合成 fixture，不是裁判答案或平台识别结果。
- 将二十条三万字节状态输入生产 guard，验证保留最近八条，每项 UTF-8 4096 B 上限及显式原长度、原 SHA、截断标记；专用 core 和其余 hook 都可编码。
- 经生产 `node.report_final()` 检查大负载，不重新引入会被拒绝的重复 approach 单点块。
- 模拟 ROS publisher 验证完整原话术只发一次、预检查及实际 publish 抛错的真实计数；UAV/灯/cruise 控制策略未在本轮放开。
- 协调器当前 context 的真实 job 失败分支留下源绑定及执行未知 null；不会因此标记 official_acceptance 或 full_mission_complete。

独立脚本及最终 XML：

```text
runs/review/20261112_single_point_transport/review_transport_cases.py
runs/review/20261112_single_point_transport/independent_final.xml
runs/review/20261112_single_point_transport/real_emitter_sizes_a07efbda404b.json
```

## 解析器接线

解析器规范保留新 `matcher_failures` 及 `match_results[].job`；原 `recognition_status.data` 不解析为 ACK。单点专用 core、历史嵌入副本、当前 compact 引用均保持原来源。新引用的 section 错误会 invalid，session 与 core 不一致显式 conflict，不静默合并。

**21 项单点解析测试加旧 motion/survey/approach/route/short-drive，共 118 passed / 7.70s**，结果在 `analyzers_final.xml`。引用校验初稿的两个红反例在 `analyzer_reference_initial.xml`；它们属于本解析器接线修正，不算机器人控制器问题。

当前共享解析器 `tools/analyze_ground_survey_evidence.py` SHA256：

```text
3feb88f6a0a7e872d6e178460f5f502c225808eef6b6820973464326a3486bde
```

本轮没有操作网站或覆盖旧平台 raw；实际新测评由主代理重新打包并在已授权账号中执行。所有平台结论仍须单独引用真实 record/runId、裁判日志和平台成绩。
