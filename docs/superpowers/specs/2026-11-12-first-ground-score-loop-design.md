# 首个地面任务计分闭环：独立节点设计

日期：2026-11-12。范围：在已真实完成的 v23 观察路线之后，接通本局三帧识别、唯一类别话术、有限轴向入区和一次到点声明。此设计不声称已取得裁判分数，不输出全任务结束消息。

## 接口与复用边界

新增 `ground_single_point.py`、`run_ground_single_point.sh` 和专项测试。`GroundSinglePointGuard(GroundObservationRouteGuard)` 复用原 TURN、TURN_SETTLE、DRIVE、DRIVE_SETTLE、CAPTURE、PAUSE 及全部安全检查，不修改旧 route 默认行为。仍采用两路 odom BEST_EFFORT、20 Hz 命令和独立 watchdog。

只有 `/cmd_vel` 可以非零。额外允许的两个输出均为 `std_msgs/msg/String`，各最多尝试一次：

| 输出 | 原始 data | 触发条件 |
| --- | --- | --- |
| `/inspection/detections` | `official_protocol.recognition_phrase("ground", 实际观测中文类别)` 的完整表 5 话术 | 本局首 ID、三帧同一物理板、唯一地面类别、当前传感器有效 |
| `/referee/current_target` | 本局公开抽签第一 ID，例如 `ground_3` | 识别话术已发送、实际中心进入候选任务矩形、末腿停车并持续静稳 |

识别话术不添加 JSON、ID、分数或 UUID；不轮发类别。只读订阅 `/referee/recognition_status`，保存有界原文、到达时间和当时阶段。已知类型是 String，接受语义尚未实测，不能假设其 JSON 字段或把本地 publish 返回视为裁判接受。

上报后持零等待 1 秒再进行有限物理入区，证据明确标 `transport_only / official_acceptance=false`。最终是否得分仅由网站裁判日志或实际公开反馈证明。没有 ACK 不伪造 ACK，也不把未知 ACK 变成无限等待。

## 最少新增状态

1. `RECOGNIZE_WAIT`：原 CAPTURE 收齐三帧后持零，异步取得匹配结果。
2. `REPORT_WAIT`：一次合法话术尝试后持零，留出公开反馈接收窗口。
3. `DECLARE_WAIT`：实际入区且停稳后一次到点声明，持零收取公开反馈并正常有限退出。

只有在 `phase == CAPTURE`、三帧已经通过原捕获门、且 stop 原因精确为 `approach_observation_complete` 时，子类截获正常观察结束并进入 RECOGNIZE_WAIT。其他 STOP 一律锁存，不恢复、不重置起点。

子类在首次 `_start_turn`、任何非零输出之前冻结观察加未来入区的完整几何预算；不能先按观察路线移动，再在识别后扩张预算。入区复用原两腿执行器；末腿 DRIVE_SETTLE 成功后直接 DECLARE_WAIT，不追加拍照转向。永久起点、累计路径、累计转角、运行时长和 PAUSE 计数均连续保留。

## 异步识别与证据身份

复用生产 `GroundMatcherWorker`：`set_context(session_id, frozen_first_ID, stationary_epoch)`、`submit(job)`、`poll()`、`cancel_context()`、`close(timeout_s=.5)`。单个 spawn 进程进行 SIFT；最多一个正在处理、一个最新待处理、一个结果。上层按三个原始曝光顺序逐个提交，避免 latest-pending 覆盖中间帧。

匹配、JPEG 编码和阻塞 IPC 都不在 guard 锁或 ROS 回调中运行。捕获回调只保留原 ROS 图、原记录和对应的原 CameraInfo；编码在独立准备线程，主循环只轮询有界结果。worker 不调用 ROS、不发官方消息。匹配等待最多 20 秒，包含冷模板初始化及三帧；仍计入原全局 300 秒。

每个 job 保留：session、首 ID、静稳 epoch、全局递增 job_seq、原图 topic/frame/stamp、JPEG SHA256、原 UAV prior 的 stamp/位置/四元数、原 CameraInfo header/K/d、模板来源及 hash。原图只接受 800×800，原 frame 为 `uav/camera_link`，CameraInfo 为同 frame、零畸变且 K 在本局不变的先前标定消息；标定消息原 stamp 不改写成曝光 stamp。每帧 UAV prior 仍须真实位于曝光之前且相差不超过 .05 秒。

两个时间合同分开检查：当前 clock/odom/scan 保留原 .20 秒 freshness、clock progress 和 .12 秒 clock sampling lag；历史曝光事实使用原 stamp，不能刷新时间。worker 的 `matches_current_context` 只证明身份相同，`eligible_for_recognition` 始终由控制层决定。

从第一帧曝光至话术发送期间必须持续没有非零运动命令、抽签不变、clock/pose epoch 不倒退、当前传感器有效，位置漂移不超过 .01m、yaw 不超过 .02rad、每样本速度不超过 .01m/s、角速不超过 .03rad/s。无效结果不发送；硬故障永久停止。可恢复 freshness 故障只允许一次重新静稳并重取三帧，不能沿用失效 epoch。

## 模板和本局唯一关联

公开参考共 12 张、7 类，其中地面 8 张覆盖五类，窗口 4 张覆盖两类。本节点只接受五个地面 class_id：`collapsed_ruins`、`landslide`、`trapped_people`、`damaged_vehicle`、`rescue_camp`，映射官方中文类别由本地固定 UTF-8 字典提供。

地面模板来源：`wbxxmz/ros2-car-first@48b3dac26b6f562ff21dd321db03fd6ffc91b492`，`src/ican_inspection_examples/ican_inspection_examples/templates/`。上传使用相对 assets 路径并逐项核验 SHA256，不使用开发机绝对路径，不把本局匹配类别写入下一局。

| 文件 | SHA256 |
| --- | --- |
| collapsed_ruins_02.jpg | 200732c46709ed2d6fba5017ce7f56c8cc0908d60eed14b6063b84a9c37cba7f |
| damaged_vehicle_01.png | 76feb761210089a1c3885ce09024ea3da6e991c52fc4bd7cc3f13d7e28b1c13b |
| landslide_01.jpg | 2653c12e906773f388fde39fbee3633ede5531460bb1442096452c41fe10a30d |
| landslide_02.jpg | f79dc767cacfd16dc8d3f989ec3ae1743f46c85bafca95223120e19c516cc689 |
| rescue_camp_01.png | 781d7c9e505a5a46c9177f584d5aff220560d28a290d1046a6378bcbb38bd014 |
| rescue_camp_02.jpg | b81c0269dc79776a10edcb54043b3e06a9188d5bbbf4b28ad951e4e998cb864a |
| trapped_people_01.png | e31429ccbf79d3898bc24fe5fb5bd8faefbfc6c29d9fc0ed573f78db1fde90b7 |
| trapped_people_02.jpg | b8aefe526d7d3a3a9045107aae75f0f9cbdcc190c5a61c9d27a9c67646308777 |

复用 GroundPlaneMatcher 的几何及跨类别冲突门。候选 source 四角完整位于图内；通过原 UAV pose 将 body-plane 中心/四角变为世界坐标。只接受唯一关联到当前首 ID 候选板矩形的物理板：中心距离不超过 .08m，三帧中心互差不超过 .03m、同一类别；同类多参考合并，同区域多类别拒绝。上述两阈值是待平台检验的工程选择，不是官方规则。相机外参、板几何及同号 ID 映射仍标 candidate。

## 轴向入区与既定限额

首版入区草案在观测点后增加两腿，右侧板 x 取 1.115（比中心西偏 1cm），最后朝 +x；避免候选 laser 到窗口前沿距离 .640m 小于原 .641m scan 门。即便取 1.115 也只有约 9mm 静态余量，必须由实时 scan 决定。

| 首 ID | 观察位 | 入区两腿终点 | 全程名义路径 | 加原 .30m 裕量 | 首版动作 |
| --- | --- | --- | ---: | ---: | --- |
| ground_0 | (.6,-.45) | (.6,-1.35)→(1.115,-1.35) | 2.465m | 2.765m | 允许有限实验 |
| ground_1 | (.6,-.5) | (0,-.5)→(0,-1.35) | 2.55m | 2.85m | 允许有限实验 |
| ground_2 | (.6,-1.8) | (0,-1.8)→(0,-2.7) | 3.9m | 4.2m | 不入区；超过原 3m 且南墙净空不符 |
| ground_3 | (.6,-1.025) | (.6,-1.925)→(1.115,-1.925) | 3.04m | 3.34m | 不入区；超过原 3m |
| ground_4 | (.35,0) | (.35,-.775)→(1.115,-.775) | 1.89m | 2.19m | 允许有限实验 |

本版不会缩减 .447m 外廓半径、.114m sensor offset、.06m 局部运动、.02m clearance、.03m scan anchor、.03m stop reserve、.641m 全向净空要求；全局 300s、累计 DRIVE 190s、每 TURN 45s、累计 TURN 100s、累计转角至多 6rad、总路径至多 3m 不放宽。普通停车保留 v23 累计 post-trigger 路径 .01m 硬门及提前 .002m 请求零速。

声明前再次验证：首 ID 未变、识别发送成功、当前传感器有效、末航点误差 ≤.02m、实际中心在候选板矩形内且距边界 ≥.02m、双时钟静稳 ≥1s、速度/角速符合原静稳标准。中心入区不等于全机完全入区；证据记录底盘/全外廓与板边界关系，不预设满分。

## 一次性发布和验收

调用顺序为 command_lock→guard 锁复核→先锁存 attempted→调用 publisher→返回后锁存 sent。发布异常或不确定完成不自动重试，避免重复话术或到点声明。每条保存 topic/data、发送时原 clock/odom、wall、首 ID、三帧 hash、类别关联与传感器检查结果；publish 返回只标 transport_sent。

专项先红后绿覆盖：三帧才能产生唯一话术、类别冲突/错区域/旧局/旧 epoch/原 stamp 倒退/相机先验过旧拒绝；异步准备不锁控制；当前 sensor stale 时零输出；启动时冻结 composite budget，识别后预算不涨、累计 origin/path/rotation 不重置；末腿未停稳/未入区不能声明；发布异常最多一次尝试；ground_2/3 禁止入区声明；旧 route 行为保持。

平台逐步验收只依据真实回传：三帧在线几何匹配→唯一话术发送记录→实际末腿静稳→一次 current_target→裁判反馈或网站计分。若没有裁判计分证据，结果仍记“闭环动作已执行、官方计分未验证”。
