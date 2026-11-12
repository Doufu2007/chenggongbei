# v25 首个真实地面点得分独立复核

2026-11-12。本局官方测评 **success，10.00 分，275.179 秒**；官网裁判赛后诊断明确显示 **`ground_0 到达并计分 +10`**。这已证明一个真实地面点的识别、上报、进入和计分闭环。只完成一个地面点，未完成另外两个地面点、穿窗、空中照射、降落或整场比赛。

## 官方结果和证据身份

- 平台记录：`c4202874-407a-4361-844a-0f95ba7b1d7f`；runId `run_20261112_060713_501f412f`；编号 `#86630201`。
- 官方列表：`runs/evidence/20261112/single_point_v25_list_final.json`，相应项为 `status=success`、`score="10.00"`、`durationMs="275179"`。
- 官网截图：`single_point_v25_official_success.png`，独立查看可见上述测评号、测评通过、成绩 10 分及裁判原文 `ground_0 到达并计分 +10`。截图 SHA256 `6e90188878b5893ccead06147097b4178044e79fcd0b823e192f688e7ac45e14`。
- 本局控制源码 SHA：`902424ca5ba3ec208a0028645d377f4ce92ff77e5ee56205c87fac95a9b349cb`，与本次官方上传 artifact manifest 一致。ZIP SHA 为 `6c3f39642065ee14b07775a37fa1b9eac6dce91a37977382b337ad27858bbf1e`。
- 原始回传：`callback_single_point_v25.jsonl`，885,764 B，SHA256 `042ebd40a121aa8f6c54b55afb720b29d8d0076d1f567ee02e54aa5f0a62610a`；62 个事件，有 `launcher_exit=0`。
- 新独立解析：`runs/analysis/20261112_single_point_v25_independent/summary.json`，SHA256 `0c0cfe2e0599c31725b7100930447d046acc61a01355376d8b3c2441cf18ab11`。原 raw 和已有解析目录均未修改。
- 独立复算：`runs/review/20261112_single_point_v25/review_v25.py`、`independent_metrics.json`；源码绑定、JPEG 字节、相机与 prior、单次发布计数均有可执行断言。

## 三帧识别的真实来源

抽签为 `ground_0,ground_3,ground_2`。只对第一项 ground_0 进行本局有限实验。三次在线 worker 结果均为 `trapped_people / 被困人员`，各自 job_seq 为 0、1、2，session 和 stationary_epoch 与专用 core 一致。

| 曝光 | 原图 stamp | 各自前序 `/uav/odom` stamp | 图像减 prior | 原 CameraInfo stamp | 原 JPEG SHA 前缀 |
| --- | --- | --- | --- | --- | --- |
| capture_0 / job 0 | 92.122s | 92.120s | 2ms | 92.122s | d137503246f05f0d |
| capture_1 / job 1 | 92.457s | 92.440s | 17ms | 92.122s | aafd594934f4cf62 |
| capture_2 / job 2 | 92.790s | 92.780s | 10ms | 92.457s | ea4a5782000bee0c |

三个原 JPEG 均为 800×800，源 `/uav/camera/image`、frame `uav/camera_link`。每个保存文件的 SHA、native transport SHA、对应 `match_results.source_sha256` 与 `job.source_sha256` 四者相同；三张图的 SHA 各异。每个 `job.pose` 与该图原 exposure 的 prior odom 完全一致，也与结果外层 pose 一致，没有插值、借用别帧 pose 或替换时间戳。

原图文件在独立解析目录：

```text
native_L000028_d137503246f05f0d.jpg
native_L000030_aafd594934f4cf62.jpg
native_L000032_ea4a5782000bee0c.jpg
```

job 保留原编码 `rgb8`、stride `2400`、原 raw ROS SHA、完整 CameraInfo header/K/D/R/P/ROI；`K` 与 CameraInfo.k 逐值相同，所有 CameraInfo 源时戳不晚于各自曝光。第二、三帧使用真实收到的前序 CameraInfo；没有为了对齐而重写为图像时戳。原 JPEG 派生说明明确为原 ROS 像素、quality 75、无 resize。

原始 ROS 像素字节没有通过回传保存，因此 **raw ROS SHA 仅是保留的源记录，无法独立从 raw 像素重算**。JPEG 则有完整字节，可以且已经逐帧独立重算。不能将二者的核验强度混同。

三个结果使用同一个真实模板 `trapped_people_01.png`，SHA `e31429ccbf79d3898bc24fe5fb5bd8faefbfc6c29d9fc0ed573f78db1fde90b7`，与本地打包模板字节及 manifest 一致，保留公开来源 commit。三帧投影中心的最大相互距离为 `0.000308593m`；对候选 ground_0 中心的距离分别为约 `0.012323m / 0.012416m / 0.012237m`。这表明本局三个输入和结果相互一致，不构成其他点或新场景准确率的统计证明。

worker PID 均为 628，三次 match 用时约 `1363.172ms / 1303.943ms / 1334.336ms`；结果保留各自实际 worker 起止时间。没有把离线 matcher benchmark 代替本局实际 worker 记录。

## 一次识别、一次声明及公开反馈

识别 `/inspection/detections` 的真实 `actual_publish_attempt_count=1`、`actual_publish_return_count=1`；唯一记录发送原文：

> 发现被困人员，地面巡航任务点确认，将巡航信息发送回指挥中心。

发送时 clock `97.0s`，机器人位置约 `(0.589153,-0.450977)`，仍在任务点外。随后一次公开 `/referee/recognition_status` 的 `String.data` 为原文 **`ground_0`**，其接收记录较本队发布记录晚 `0.001219456s`，phase `REPORT_WAIT`。该差值是本队本地记录间隔，不是独立网络延迟或精确裁判处理耗时。

入区并停车后，`/referee/current_target` 的真实 publish 调用/返回也均为 1，内容 **`ground_0`**；此声明较识别发送记录晚 `95.817770163s`。声明时 clock `191.1s`、odom `191.18s`，位置 `(1.113135334,-1.355610228)`，离候选中心约 `0.013124213m`。本队输出仍明确 `body_fully_inside=not_claimed_center_entry_only`；不从车体中心位置推断整个机体足迹均在框内。

`recognition_status.data` 在通用解析器里保持原始 String，不被擅自解析为 accepted 或成绩。本局 ground_0 的得分判断来自额外的官方赛后诊断及总分；不是把 `sent=true` 或 `ground_0` 字符串本身变成分数。

## 实际轨迹和有限完成范围

实际记录包含 1,904 行轨迹、4,316 行实际命令，两个 series 都完整。四腿依次为外观察 `(0.6,0) → (0.6,-0.45)`、进入 `(0.6,-1.35) → (1.115,-1.35)`；四腿静稳航点误差分别为：

```text
0.001088825m / 0.005662308m / 0.012129886m / 0.005911991m
```

各腿请求停车锚后的原 guard 累计 path 增量为约 `0.581mm / 0.059mm / 0.583mm / 0.121mm`；它们包含请求至实际发布前的运动，不是执行器 ACK 或精确连续制动最坏界。

总累计 path `2.509538296m`，小于冻结预算 `2.764958005m`；累计旋转 `4.070505806rad`，小于冻结预算 `4.597735802rad`。累计转向 wall time `93.364650108s`，累计直行 wall time `90.357921150s`，本局有一次恢复。末 odom 约 `(1.113308413,-1.355615365)`。

本队核心 `FINISHED / single_point_transport_complete_official_score_unverified / 195.336s / errors=[]`，官方平台结算耗时为 `275.179s`，两者是不同计量范围，不能相互替代。UAV、灯光和 cruise 仍静默，没有完整结束发布；另外两个 ground、穿窗、空中照射和降落均未执行。

## 平台可对本次有限任务自然结算

本局代码没有发布整场完整完成消息，但平台在此局得到了合法 ground_0 分数，并自然给出 success/10。**因此，“缺少完整结束消息就必然 failed”的先前推论已被本局反证。** 之前各局 failed/null 是它们各自的实际平台结果，不能由此推广出统一结算必要条件。

本局只证明这个有限首点运行可以成功结算，不保证任意提前退出、任意单点、所有失败路径或完整任务都能如此结算。后续以实际裁判结果继续验证。

独立 parser 仍为 `partial`，保留 `score=null`、`official_scoring=unverified`：它仅解析本队回传，不读取或替代官网成绩。本局没有 ground_motion_full，末尾 fallback 缺少 `image_records/motion_log/tf_edges` 三项，但三次 CAPTURE、完整 series、单点 core、来源和发布记录齐全，parse/assembly/conflict/diagnostic 均为空。此取证打包状态与官网 success/10 应并列呈现。

## 台账中的真实点号

台账原 `scores.items.ground_1/ground_2/ground_3` 是三任务槽位，不强行把物理 ground_0 改名为 ground_1。新增可选 `scores.ground_points` 保留真实物理点号：本局 `ground_0=10`，其他四点均为 null；旧 items 槽位仍全部 null，总分 10，人工分 null。两种视图不相加，不从点分推造总分。

该兼容接线保留所有旧记录。新增真实点号合法、未知 ID/非法分数拒绝、总分一致及不重复计分用例；`tests/test_run_records.py` 共 **50 passed / 2.23s**。本局仍标 `kind=interface_probe`、`full_mission_completed=false`，不进入完整比赛完成率统计。文件 `runs/records/single_point_v25_c4202874.json`。
