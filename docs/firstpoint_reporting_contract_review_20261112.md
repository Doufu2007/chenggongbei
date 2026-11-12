# 首点上报接线合同：独立只读复核

2026-11-12。范围为官方公开判分细则缓存和当前本队源码；不运行机器人、不向裁判发送消息、不读取裁判私有实现或随机答案。

## 可复核的接口与顺序

官方来源为 [判分细则课程](https://www.splenach.com/courses/129d6e75-f19a-4c97-9ec6-0853695a4c48)，本地缓存 `docs/referee_rules_v1.txt`。巡航专用段第 119 行明确：按 `/referee/ground_tasks` 本场顺序，点外识别对应地面图片，经 `/inspection/detections` 报告正确话术，判定正确后进入；进入点位后经 `/referee/current_target` 发布 `ground_X`。第 130—132 行给出正确识别且进入的地面点为 10 分/点；第 154—157 行限定地面导航、指定点及类别匹配和完整进入。此处以巡航专用顺序为准。

`bounded_ground_probe.py:350` 的 `INPUT_TYPES` 已包含 `/referee/recognition_status: std_msgs/msg/String`，不需重复创建同名订阅。第 532 行起 `_receive_impl` 将非 Image 转为字典，交给 `guard.observe_public(topic, payload, wall_s)`；该话题还进入 `public_messages` 的 first/last 保留通路。当前仍没有真实状态 payload 样本，不能假设它使用 JSON、`accepted` 字段、自队事件 UUID 或指定的接受值。

`OUTPUT_TYPES` 已声明 `/inspection/detections` 与 `/referee/current_target` 两个 String 发布者。端点存在和 matched 数只证明传输配对，不证明任何识别、到点或计分已发生。`official_protocol.recognition_phrase("ground", measured_category)` 已对应表 5 的五种地面识别话术；类别必须来自本局真实图像，不能用上一局结果或 `ground_N` 映射替代。

## 新单点节点应补的最小接线

1. 在独立单点节点显式允许 `/cmd_vel`、`/inspection/detections`、`/referee/current_target`；保留旧诊断节点仅允许 `/cmd_vel`。其余 UAV、灯光与 cruise 发布者保持静默。
2. 修正新节点的输出元数据来源。当前 `refresh_endpoints()` 第 475 行的 `silent: topic != "/cmd_vel"` 是旧诊断约束；新节点真正发布两 String 后复用此硬编码会记录错误。应同时记录策略是否允许、实际发布计数及最后发布原文，使“允许但未发”与“已经发布”可区分。
3. 每次实际 String 发布保留 topic、原始 `String.data`、本地 wall/ROS clock、当前抽签首 ID、对应原图 SHA/stamp/前序 pose、实测类别与识别证据，以及进入后的停车 pose。调用 `publish()` 返回只记为本队已调用发送，不标为官方接受。
4. 保留原 `/referee/recognition_status` 载荷和接收时刻；取得真实样本后再建立解释与关联。若暂采用 `transport_only` 推进策略，明确其只证明已发送及等待，不声称满足官方“判定正确”的接受条件。
5. 点外识别报告与入点后声明分开。仅当本局首 ID 与实际板的物理关联、图像类别和到点条件均成立才执行；识别不确定则拒绝上报，不遍历类别或点号试分。
6. 首点有限实验不发送全任务完成、降落或伪造结束标记。以裁判接受/计分记录或真实官方成绩确认首点计分；没有结算时，将本队动作证据与官方成绩缺失并列。

## 当前结论

输入名称和类型已接通；最小新增工作在新单点节点的受限发布、准确元数据及运行时视觉/进入条件。官方状态的接受语义和首点计分仍需实际消息及裁判证据验证，不能靠本地 FINISHED 字样替代。
