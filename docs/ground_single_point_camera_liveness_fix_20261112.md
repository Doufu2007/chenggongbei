# 单点 v24b：相机存活门限修复

2026-11-12。本轮只修改 `ground_single_point.py` 的静止识别相机存活检查和相关测试；旧观察路线、控制器、odom/scan 新鲜度与真实图像时间戳不变。没有打包或上传测评。

## 真实失败原因

原始证据为 `runs/evidence/20261112/callback_single_point_v24b.jsonl`，636,523 B，SHA256 `40d5905f4c0d749537ba8183617686a9f886cc58d2922d4ebed5d80a76a221e7`。紧凑提取保存在 `runs/review/20261112_single_point_camera/v24b_camera_root_cause.json`。

| 原始位置 | 事实 |
| --- | --- |
| 第 3 行 `ground_motion_core` | 53.083s，FINISHED，原因 `single_point_uav_image_stale`，errors=[]；UAV 图像156帧 |
| 第 3 行 `callback_timing.spans.receive:/uav/camera/image` | 155个实际到达间隔：最小 .313796517s、最大 .384845019s、总52.498606919s；曝光header间隔 .331–.336s，总51.666s |
| 同一计时记录 | 图像回调最大 .000257510s；tick最大 .004625331s。没有显示主循环被SIFT长时间阻塞 |
| 第 5 行 `survey_metadata` 的 `capture_2` | 原曝光53.075s，到达墙钟相对起点50.821462692s；真实前序UAV odom53.06s，差 .015s |
| 第 11 行 `actual_commands`，全局行1104 | 首个 RECOGNIZE_WAIT命令：elapsed50.835714s，原clock53.1s，零速 |
| 第 11 行同序列，全局行1112 | 首个STOP命令：elapsed51.038446s，原clock53.2s，零速 |

第一条STOP命令在最后捕获图像到达后约 `.216983308s`，已越过新增的 `.20s` 相机墙钟门；此时原clock−图像曝光仅 `.125s`。命令序列的墙钟舍入至6位小数，因此该差值是首个STOP命令的采样差，不是精确停止锁存、publish或执行器确认时间。

停止前最后一条实测scan配对：scan53.251s，到达elapsed50.997577510s，prior odom53.24s，到达50.986918038s，skew .011s；首STOP命令与该scan到达仅约 .04086849s。最终汇总中的scan年龄超过2秒，是STOP后的持零退出期数据，不能倒过来作为这次停止的起因。

真实送达的曝光频率约3Hz，墙钟送达频率约2.95Hz，因此 `.20s` 的相机持续存活门在每个正常帧周期中必然出现空窗。公开候选 `inspection_task1.world` 第302行UAV相机 `update_rate=30`；候选模型的设置不是平台实际送达频率保证，不能据此假设每33ms有图，也不能仅从本证据定位节流发生在相机、桥接或传输哪一层。

## 最小修改

单点节点新增固定 `CURRENT_CAMERA_MAX_AGE_S=.50`，只用于 RECOGNIZE_WAIT / REPORT_WAIT 的当前图像存活：

- 当前墙钟−真实最新递增曝光的到达墙钟 ≤.50s。
- 当前clock−真实最新图像原曝光位于 [-.12,.50]s。
- 重复或倒退的图像header不能刷新最新图像到达时间；旧图像stamp不改写为当前clock。
- `camera_liveness` 证据保存检查时墙钟、真实上次到达时间、原曝光、两种源龄和本次门限。

.50s是工程门限：本次最大到达间隔 .384845019s 加一个20Hz命令间隔 .05s 为 .434845019s，仍小于.50s。它不是对未来平台相机频率的保证。超过门限继续零速停止，不无限接受旧图。

以下合同全部保持：clock/odom/scan/UAV odom 当前年龄 .20s、clock负采样容差 .12s、每帧曝光对应真实前序UAV odom≤.05s、同局首ID/静稳epoch、无非零命令、位置/偏航漂移门、匹配20s、原路径/累计转角/全局运行预算。历史三帧匹配仍使用原header、pose和hash，不要求SIFT在.50s内返回，也不借完成时间刷新历史曝光。

## 验证

新增相机针对性用例13项，修改前8失败5通过，修改后13通过；完整单点专项43通过/25.05s。XML保存在 `runs/review/20261112_single_point_camera/`：`camera_initial_red.xml`、`camera_after_fix.xml`、`single_point_camera_fix.xml`。

覆盖真实v24b采样间隔、.384845s正常间隔、.50s含边界/.500002s拒绝、原clock源龄两侧边界、连续约3Hz相机且控制数据新鲜、重复曝光不刷新到达、图像门独立而控制数据.201s仍拒绝；原三帧识别、一次发送、路线入区、停稳、传输不等于裁判接受等专项均保持通过。

该结果说明修复了已复现的提前停止条件。是否在线识别成功、合法话术被裁判接受及实际入区得分，仍需下一次平台测评。
