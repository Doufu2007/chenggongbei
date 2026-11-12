# 官方平台只读探针 v5 证据（2026-11-12）

v5 已取得完整诊断快照、14 类公开消息样本和两张真实相机诊断 JPEG，证明本队探针节点在官方环境中运行并接收传感器。平台终态仍是 `failed + score:null`；本次只读探针没有执行比赛任务，也没有运动、识别、完成或计分输出，不能作为正式任务通过或得分证据。

## 测评与平台终态

- runId：`run_20261111_222348_ef5c04b0`；详情 ID：`a1485e1e-27a0-45b0-bd2f-d3c313fdef6a`。
- 平台记录创建于 2026-11-11 22:23:48 +08；最终记录更新时间为 2026-11-12 00:17:06 +08。
- `status:failed`、`score:null`、`scoreDetail:null`、`durationMs:112172`。该耗时是平台记录耗时，不是完整任务完成时间。
- 失败原因：`流程未检测到完整结束标志（未提取到成绩）`。只读探针未执行任务和正式收尾，不能把这一结果改写成 Python 未启动，也不能将脚本退出码 0 改写成比赛通过。
- 最终查询快照的 `serverDown:true`、`serverStatus:down` 只描述查询时状态，不能回推为本局失败原因。
- 原始终态：`runs/evidence/20261112/probe_v5_final.json`；SHA256：`c5fabf5e51bc1a47e1c5952468a1ab47f946d1a001034397a29a7890912ac6be`。

## 回调与证据完整性

原始回调 `runs/evidence/20261111/callback_v5.jsonl` 为 42,706 bytes，SHA256：`2bb334edc51f1b9207bedf22323d0b15c03280157e3fca148355e03e8ce9723c`。路径沿用接收器日期目录；事件生成于 2026-11-12 00:13:53–00:15:00 +08。

共保存 5 条网络诊断事件：`launcher_started` 1 条、`probe_snapshot` 1 条、`image_sample` 2 条、`launcher_exit` 1 条（退出码 0）。没有收到 `probe_started`、`probe_finished` 事件，不能声称这两条启动/完成事件已回传；完整快照与真实传感器数据仍能证明节点实际运行。分析器因此保持 `status:partial`、`official_scoring:unverified`；快照必需字段齐全，解析错误和快照 `errors` 均为空。

分析结果：`runs/analysis/20261112_v5/summary.json`，SHA256：`0129af141748f8464cf2fcd7e690444bea259cc3dc0c9ef98067a7861ca14bc5`。快照上报的累计回调数与保存的一条代表样本分开记录，不能相加，也不能直接用 60 秒采样设置推断仿真发布频率。

| 真实接收话题 | 快照上报累计回调数 | 本地保留代表消息数 |
| --- | ---: | ---: |
| `/referee/ground_tasks` | 1 | 1 |
| `/tf`、`/tf_static` | 822、1 | 各 1 |
| `/clock` | 538 | 1 |
| `/odom`、`/uav/odom` | 1297、1294 | 各 1 |
| `/scan` | 422 | 1 |
| `/uav/laser/scan` | 1066 | 1 |
| `/uav/lidar/points` | 644 | 1 |
| `/camera/image`、`/camera/camera_info` | 163、157 | 各 1 |
| `/uav/camera/image`、`/uav/camera/camera_info` | 161、156 | 各 1 |
| `/uav/laser_state` | 1297 | 1 |

`/referee/recognition_status` 的类型和订阅已发现，但本次没有实际消息样本，其接受值和关联语义仍未知。

## 本轮确认的数据合同与边界

- 本轮公开地面抽签原文为 `ground_1,ground_4,ground_0`。这是允许读取的任务顺序，不是图片类别答案；它仅证实本轮出现这三个 ID，不能推出完整五点集合或 ID 到真实坐标的映射。
- 两路源图均为 `800×800`、`rgb8`、`step=2400`；地面图 `frame_id=camera_link`、时间戳 3.058 秒，空中图 `frame_id=uav/camera_link`、时间戳 5.391 秒。名称中没有 optical 后缀，不得因此猜测图像光学轴到机体的旋转。
- 两路 CameraInfo 的 `fx=fy=476.7030836014194`、`cx=cy=400.5`，`distortion_model=plumb_bob`、五项畸变系数均为 0。UAV 投影矩阵的第 4 项为 `-33.36921585209936`，地面为 `-0.0`；不能把投影矩阵的基线项直接当作完整机体外参。
- 两路 Odometry 的父 frame 均为 `odom`，地面 child 为 `base_footprint`，空中 child 为 `uav/base_link`；样本时间戳均为 2.90 秒。它们当时姿态接近，不证明全程可互换，也不构成导航坐标和响应已验证。
- 仅保留的一条静态 TF 为 `uav/base_link → uav/laser_link`，平移 `(0,0,-0.07)`、四元数约对应绕 y 轴 `π/2`。它不能补出缺少的相机、前向点云或全部机体 TF。
- UAV LaserScan 样本为 `frame_id=uav/laser_link`、角域约 `[-0.001,0.001]`、3 束、量程约 `[0.03,5.0] m`，三束均为 `inf`。这不是有效离地高度，也不能证明上方、后方或整机体积净空。
- UAV 点云样本为 `frame_id=uav/lidar_link`、`height=1`、`width=1718`、`point_step=16`，字段为 x/y/z/intensity。快照只保留数据的短前缀，不能据此重建整片点云或证明通道无障碍。
- `/uav/laser_state` 已实收 `std_msgs/msg/Bool`，代表值 `false`，发布端为 RELIABLE/VOLATILE。它确认接口存在且有流式回调，不证明开启命令已交付、目标已命中或持续照射计分。
- 公开端点快照中 `/cmd_vel`、`/uav/cmd_vel`、识别与声明输出话题的订阅者均为 0。该快照未提供这些控制端的消息类型和接收证据；不能推断永久不可用，也不能以此绕过连接门禁。

## 真实 JPEG 的用途

诊断 JPEG 为探针由真实源图生成的 `360×360` 缩小图；分析器保存其原始 JPEG 字节，未再次编码：

| 来源 | 保存文件（位于 `runs/analysis/20261112_v5/`） | bytes | SHA256 |
| --- | --- | ---: | --- |
| `/camera/image` | `image_L000003_7e8a2d30d4881eaa.jpg` | 2760 | `7e8a2d30d4881eaa20979850a1fd3806ac0b7808a7d4dd901c93061a8e70a4dc` |
| `/uav/camera/image` | `image_L000004_25df1e640247efd7.jpg` | 9213 | `25df1e640247efd7efa8b1b8800a1354f5a8904d79dcd242e9af358469f4ca8e` |

两张图仅用于证明相机数据路径和后续视觉诊断，不是完整任务录像、七类识别率、穿窗效果、照射命中或裁判得分证据。地图/图库版本、光学外参、真实控制响应、识别确认和正式结算仍需继续验证。
