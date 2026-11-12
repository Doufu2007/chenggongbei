# 官方平台只读探针 v4 证据报告

更新日期：2026-11-11。运行号 `run_20261111_155937_44feef8e`，详情 ID `81691f7d-2aee-4b0a-a9a6-537b768677c4`。

本次已取得本队上传 shell 启动、rclpy 探针运行、公开话题发现、真实 `/odom` 与 `/tf` 消息以及脚本退出码 0 的证据，解决了“只能看到裁判日志、无法证明用户程序启动”的部分缺口。**平台最终状态仍为 `failed`，`score:null`；本次只读探针没有执行比赛任务，未获得任务得分或完整完成证据。** 脚本正常退出与平台是否取得成绩是两个不同结果。

## 证据与终态

| 文件 | 核验内容 |
| --- | --- |
| `runs/evidence/20261111/callback.jsonl` | 21 条自有诊断回调，12,322 bytes；SHA256：`0a0b7c33fc1276c29357b67a136339cc6d14487b15cfca7788903b84b3931cc0` |
| `runs/evidence/20261111/probe_files.json` | 上传文件名、大小与哈希记录 |
| `runs/evidence/20261111/probe_detail_sixth.json` | 16:36:30 +08 的中间快照：`running`、`score:null`、服务状态 `down` |
| `runs/evidence/20261111/probe_detail_latest.json` | 16:59:38 +08 的最终快照：`failed`、`score:null`、`durationMs:66337`、服务状态 `up` |
| `docs/platform_probe_v4_20261111.json` | 原始证据的派生汇总，保留中间快照并另列最终快照 |
| `runs/records/run_20261111_155937_44feef8e.json` | 练习接口探针台账；不计入完整任务完成率分母 |

最终原始失败原因是“流程未检测到完整结束标志（未提取到成绩）”。它不能改写成“Python 启动失败”：本次实际启动与收数已由独立诊断证实。服务状态在不同查询时刻会波动，最终为 `serverDown:false`、`serverStatus:up`，不能把中间的 `down` 回推为本次或历史失败根因。

回调接收时间从北京时间 16:35:57.885889 至 16:36:34.159730。事件计数如下：

| 事件 | 数量 | 可证明的范围 |
| --- | ---: | --- |
| `launcher_started` | 1 | 上传 shell 已执行启动上报 |
| `probe_started` | 1 | 本队 rclpy 探针已运行 |
| `public_graph` | 1 | 已发现允许读取的公开话题图 |
| `subscription` | 14 | 已完成对应类型发现和订阅创建；不等于收到消息 |
| `sample` | 3 | 两条 `/odom`、一条 `/tf` 实际消息 |
| `launcher_exit` | 1 | 上传脚本报告真实 `exit_code:0` |

回调记录的 `source` 为 `unverified_callback`，属于本队诊断证据，不是裁判确认。该脚本直接前台运行上传的 Python 源码，因此本次也**没有证明 colcon 编译成功**。

## 已发现的公开接口

下列发布者均报告 `RELIABLE`。`/referee/ground_tasks` 与 `/tf_static` 为 `TRANSIENT_LOCAL`，其余为 `VOLATILE`。

| 话题 | 实际发现类型 | 已收到的消息样本 |
| --- | --- | --- |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 无 |
| `/camera/image` | `sensor_msgs/msg/Image` | 无 |
| `/odom` | `nav_msgs/msg/Odometry` | 2 条 |
| `/referee/events_public` | `std_msgs/msg/String` | 无 |
| `/referee/ground_tasks` | `std_msgs/msg/String` | 无 |
| `/referee/recognition_status` | `std_msgs/msg/String` | 无 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 无 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 1 条 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 无 |
| `/uav/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 无 |
| `/uav/camera/image` | `sensor_msgs/msg/Image` | 无 |
| `/uav/laser/scan` | `sensor_msgs/msg/LaserScan` | 无 |
| `/uav/lidar/points` | `sensor_msgs/msg/PointCloud2` | 无 |
| `/uav/odom` | `nav_msgs/msg/Odometry` | 无 |

图快照中各话题有 1 个发布者、0 个订阅者；这是探针创建订阅前的快照，不是订阅失败证据。`String` 的 `data:string` 字段已确认，但没有收到抽签订单或识别状态的实际内容；不能据此猜测它们的 data schema 或关联语义。

两条 `/odom` 分别为仿真时间 3.18 s 和 3.20 s，`header.frame_id=odom`、`child_frame_id=base_footprint`。位置近 `(0.00003,0.0000082,0.055)` 米、姿态近单位四元数、速度近零。`/tf` 样本为仿真时间 3.20 s 的 `odom → uav/base_link`，数值与第二条 odom 一致。两条 odom 和一条 TF 不足以证明稳定频率、完整 TF 树或两路 odom 可互换。

## 诊断回传积压

`probe_started` 至首条 TF 样本的 payload 时间跨度只有 0.085912 s，对应接收时间跨度却有 28.830619 s；payload 与接收时间差从约 12.15 s 增至 40.90 s。跨机器时钟尚未校验，绝对时间差不能直接称为单向网络延迟，但这一批消息的相对间隔增长支持回传积压解释。

上传 ZIP 内的 reporter 使用单工作线程 FIFO，队列容量 64，单请求超时 1.5 s，工作线程为 daemon，`close()` 只 `join(timeout=3)`。因此仍有诊断排队时进程即可结束；独立 shell 的退出上报时间差约 10.72 s，也与前面的 FIFO 积压不同。没有收到 `ground_tasks`、相机、心跳或 `probe_finished`，不能据此断言 ROS 回调没有发生或相应接口不可用。

下一次只读采集应减少逐条网络事件，把各话题计数、最新允许样本和最终摘要合并回传，并保证最终摘要有独立发送机会。优先取得抽签 CSV、识别状态实际 data、两路 CameraInfo/图像、scan/点云 header 与样本、UAV odom、静态 TF，以及灯光相关的公开合同。随后才能验证地图、视觉与控制；本报告没有填入未知坐标、目标身份、有效照射或裁判分数。

## 尚未完成的验收

尚未证实：源码编译、实际相机帧和标定内外参、抽签内容、识别回执语义、灯光反馈、机器人控制、地面点计分、穿窗、照射、降落、完整任务结算与正式平台录像。当前 `verified:false` 和 `platform_run_verified:false` 应继续保留。默认 `event_ack` 与显式 `transport_only` 的本地适配也不能替代裁判接受证据。

相关缺口见 [protocol_gap_current.md](protocol_gap_current.md)，飞行覆盖与照射几何审计见 [flight_contract_audit_20261111.md](flight_contract_audit_20261111.md)。
