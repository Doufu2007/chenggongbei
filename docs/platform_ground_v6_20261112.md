# 平台有界地面练习 v6：真实诊断记录

日期：2026-11-12。平台详情 ID：`930a52f0-a4f6-4483-b88b-813e17dd3807`；runId：`run_20261112_010055_0b9da786`。

本轮是只允许地面低速短脉冲的接口验收，`mode=bounded_ground_motion`、`read_only=false`，不是完整巡检任务。平台终态为 `failed`、`score=null`，平台时长为 **52.948 秒**；失败原因是“流程未检测到完整结束标志（未提取到成绩）”。没有发送任务完成、识别、照射或飞行消息，不能把诊断程序退出或回调完整视为比赛成功。

## 已取得的公开接口证据

创建声明的输出发布端后，本局六个公开输出均观察到 `/ican_inspection_public_bridge_211` 的真实订阅端点及 **1 个匹配订阅**。所见 QoS 为 `RELIABLE/VOLATILE`，类型如下。这是本局图与传输匹配证据，尚不是官方接口语义或控制响应验证。

| 输出话题 | 本局观察的类型 | 本轮发送行为 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 4994 条零速，0 条非零速 |
| `/uav/cmd_vel` | `geometry_msgs/msg/Twist` | 静默 |
| `/referee/current_target` | `std_msgs/msg/String` | 静默 |
| `/inspection/detections` | `std_msgs/msg/String` | 静默 |
| `/referee/cruise_event` | `std_msgs/msg/String` | 静默 |
| `/uav/laser_fire` | `std_msgs/msg/Bool` | 静默 |

v5 快照的零订阅不能推成“平台永久没有控制接收端”；v6 记录说明本局声明公开输出后可以匹配。仅声明其他五个输出，没有在这些话题发布消息。

## 本轮为什么没有启动短脉冲

探针终态为 `FINISHED`，原因 `wait_timeout:odom_stale_or_invalid`，探针自身记录的墙钟时长为 **22.054 秒**。其状态日志在 `start_stability_dwell`、`odom_stale_or_invalid` 等等待条件之间反复变化，未进入 `PULSE`，所以非零发布次数为 0。状态日志保留 64 条，另有 **409 条变更被有界存储省略**；原始计数不会因省略而重置。

本轮收到 `/clock` 216 条、地面 odom 1077 条、无人机 odom 1076 条、`/scan` 172 条、`/tf` 2153 条；双相机各收到 65 帧。不同频率消息与 v6 新鲜度条件的负时间差容忍值存在不适配：odom 可先于下一条较低频率 `/clock` 到达，被旧条件当作无效，进而重置连续 1 秒稳定驻留。这个机制可以解释状态变化，但 v6 没有记录每次决策的传感器年龄，不能声称已逐条回放证明全部拒绝原因。

v6 还把控制 tick 紧随每次 `spin_once` 调用，导致约 22 秒内发送 4994 条零速。后续版本将命令调度限制到独立墙钟频率，并显式记录时钟采样相位差及决策时年龄，再由平台新一局验证。后续修复不追改本轮结果。

地面 odom 首末位置的三维差约 **0.00019045 米**，它覆盖整次采集，不能当作短脉冲响应；本轮根本没有发出非零脉冲。`max_observed_displacement_m=0` 是探针尚未建立脉冲起点时的字段值，也不表示整轮 odom 完全不变。

## 传感器与图像的实际覆盖范围

本局公开抽签数据为 `ground_1,ground_2,ground_0`，仅表示本轮任务顺序，不是图像答案。累积的 TF 只有三条已观察边：`odom → base_footprint`、`odom → uav/base_link`、`uav/base_link → uav/laser_link`；未取得的相机外参不能由此补造。

已保存双相机各首末两张、共 **4 张真实诊断 JPEG**，原字节未重新编码。它们是静态传感器快照，不构成完整演示录像，也不能证明七类识别、穿窗、照射、降落或裁判得分。

## 证据与重建

- 原始回调：`runs/evidence/20261112/callback_ground_v6.jsonl`，81733 字节，SHA256：`560bbf72dd5204823168eede3ee8fb5d6f81894f0aa20d49a57380a67da4f79f`。
- 平台终态：`runs/evidence/20261112/ground_v6_detail_0103.json`。
- 提交及产物记录：`ground_v6_submission.json`、`ground_v6_artifacts.json`，与终态同目录。
- 独立分析摘要：`runs/analysis/20261112_ground_v6/summary.json`，真实 JPEG 同目录。
- v6 上传源码和脚本保持原样；分析工具不在已提交的 `src` 中，无需重建本轮上传包。

重建命令：

```powershell
python tools/analyze_ground_motion_evidence.py runs/evidence/20261112/callback_ground_v6.jsonl --output runs/analysis/20261112_ground_v6
```

分析器输出 `status=collected`，仅表示所要求的生命周期事件及完整诊断快照已收到：共 9 个事件、4 张 JPEG、0 个解析错误。`official_scoring`、`task_completion` 与脉冲响应因果判断均仍为 `unverified`，`score=null`。原始 core/full 分片、未知字段、缺失项和冲突均单独保留；不会把各快照的消息计数重复相加。
