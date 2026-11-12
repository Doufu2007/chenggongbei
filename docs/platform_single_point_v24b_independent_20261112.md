# v24b 平台首点实验独立复核

2026-11-12。本轮完成首点外观察位和三次真实曝光，随后被相机活性年龄门停止；没有产生识别结果、ROS 识别上报、进入或到点声明。平台结果仍为 **failed / score=null**。本队 `FINISHED` 是停止归零完成，不是正式测评成功。

## 身份和证据

- 平台记录：`e51c52ce-2605-4a4b-acb2-ea9c4a8bbfee`；runId `run_20261112_055257_72484c3b`；编号 `#29764869`；练习 `single-point-v24b`。
- 权威缓存：`runs/evidence/20261112/single_point_v24b_list_after_run.json`。平台耗时 `128.875s`，失败原文“流程未检测到完整结束标志（未提取到成绩）”。
- 本局源码：`a07efbda404b2596f85cc7e9f9d8e2fd54ab442c16d21319658cf6169c6b16d6`；ZIP 与启动脚本身份见同目录 `single_point_v24b_artifacts.json`，不将后来修补的源码归给此局。
- 原始回传：`callback_single_point_v24b.jsonl`，636,523 B，SHA256 `40d5905f4c0d749537ba8183617686a9f886cc58d2922d4ebed5d80a76a221e7`，65 个事件，末尾 `launcher_exit=0`。
- 独立解析：`runs/analysis/20261112_single_point_v24b_independent/summary.json`，SHA256 `a24af1b9ed78c3008fdaebbb0b7a4f35b0250bc3f9fa4064dbe5186fd1fee371`。
- 可复算脚本与机读结果：`runs/review/20261112_single_point_v24b/review_v24b.py`、`independent_metrics.json`。

原始日志及先前解析目录均未覆盖。新解析保留 517 行轨迹、1,200 行实际命令、7 张原 JPEG；两个 series 完整，三张 CAPTURE 均无缺片。parse/assembly/diagnostic/conflict 均为空。

## 停止根因：相机约 3 Hz，误用了 0.2 秒活性门

回调计量记录 `/uav/camera/image` 共 156 帧，155 个源时间间隔合计 `51.666s`，最小 `0.331s`、最大 `0.336s`，源时间频率为 **3.000038710Hz**。对应墙钟间隔总 `52.498606919s`，频率 **2.952459296Hz**，间隔最小 `0.313796517s`、最大 `0.384845019s`。这些是实际接收计量，不是配置频率。

最后一次 CAPTURE 墙钟 elapsed 为 `50.821462692s`，第一条 STOP 实际零命令记录为 `51.038446s`，相差 **0.216983308s**；该时刻 clock `53.2s` 与末图源时间 `53.075s` 相差 `0.125s`。本局代码对当前相机的墙钟年龄也使用 `0.2s`，会在正常三帧/秒的帧间触发 `single_point_uav_image_stale`。

本局没有输出独立 camera_liveness 决策对象，因此 `0.216983308s` 采用各自原始曝光记录与第一条 STOP 发布记录的差，未伪称纳秒精度的内部判定时刻。回调源间隔和墙钟最小间隔均超过这一差值，支持此处为正常下一帧到来前的年龄门触发。

STOP 转换记录 `51.0385s` 时，clock wall age `0.087868442s`，odom wall age `0.013890643s`，scan wall age `0.040881305s`，控制传感器均 fresh。停止原因不是编译失败、节点未启动、没有相机、缺少三张图片或缺少 ROS 订阅者。核心最终 `FINISHED / single_point_uav_image_stale / 53.083s / errors=[]` 包含其后约两秒归零。

## 三帧各自真实来源

| 阶段 | 图像 stamp | 原前序 UAV odom stamp | 实际差值 | 对观察航点 `(0.35,0)` 的 prior pose 误差 |
| --- | --- | --- | --- | --- |
| capture_0 | 52.409s | 52.400s | 9ms | 0.006801160m |
| capture_1 | 52.743s | 52.740s | 3ms | 0.006801465m |
| capture_2 | 53.075s | 53.060s | 15ms | 0.006801656m |

三图源均为 `/uav/camera/image`、frame `uav/camera_link`；每图保留各自真实 `/uav/odom`、`odom → uav/base_link` prior pose、quaternion 和源时戳，没有插值、借用别帧 pose 或重标来源。相邻图像源时间差为 `0.334s / 0.332s`。

独立解析目录内三个文件分别为：

```text
native_L000012_1220a44c55aa365a.jpg
native_L000014_182fb89009136f44.jpg
native_L000016_edf74d9622579602.jpg
```

每个文件 SHA 均与原 transport SHA 一致，三者不同。这里只验证来源、原字节与 prior 关联；本局 `match_results=[]`，没有把图片存在推断为类别已识别。

## 路线、ROS 发布和官方结果

抽签为 `ground_4,ground_2,ground_3`。仅执行 ground_4 外观察路线：一腿到 `(0.35,0)` 后转向候选板。该腿静稳航点误差 `0.001951435m`；请求停车锚后原 guard 累计 path 增量 `0.000075478m`。转向后 prior pose 约 `(0.345502,-0.005102)`，yaw `-0.760647480rad`。这些属于有限观察过程，没有执行计划中的 `(0.35,-0.775) → (1.115,-0.775)` 入区路线。

`in_entry=false`，三帧采集后记录的命令全部为零。轨迹记录最大 x `0.3481234m`、最小 y `-0.005185m`，末 odom `(0.345518172,-0.005119870)`，与未入区事实一致。不能用候选映射或规划航点替代实际进入。

`/inspection/detections` 与 `/referee/current_target` 均 matched=1，策略允许发送，但真实 `actual_publish_attempt_count=0`、`actual_publish_return_count=0`；`attempts={}`、`recognition=null`、`match_results=[]`、`matcher_failures=[]`、`recognition_status=[]`。这说明本局没有可证实的识别匹配、发布调用、裁判接受或首点得分。没有 matcher 失败记录也不等于 matcher 已成功执行。

UAV、灯光和 cruise 仍静默；本局没有全任务结束消息。官方 failed/null 与本队有限动作分别记录，不能由本队退出码 0 或 FINISHED 替代平台结算。

## 解析器 partial 的真实原因及最小修正

此前解析出现十个 `unknown_sections`，其原 `section=survey`，topic 为 mode、trajectory_dropped、actual_commands_dropped、exposure_records、image_errors、observed_odom_periods、task_completion、approach、scan_records、module_availability。这是 `report_final` 在大 full snapshot 无法单包发送时，按 survey 顶层字段输出的合法 fallback 片段。

解析器原来遗漏此分支，现显式验证 `partial=true`、topic/value 与原 mode，保留到 `survey_fragment_observations` 和 `survey_fragment_assembly`；重复冲突显式留存。没有把它冒充 full snapshot，没有覆写原 metadata 或伪补缺失图像。

修正后 `unknown_sections=[]`，仍为 **partial**：本局 `ground_motion_full` 事件数量为 **0**，解析器原完成门要求真实 full 事件。未知分片不是原 partial 状态的触发条件；也不存在 CAPTURE、series、生命周期事件或快照字段缺失。`partial` 是取证打包状态，不代表官方成功或失败。

五项 fallback 反例先全部红，最小接线后通过；另补两项 camera_liveness 原样保留与错误容器校验。单点与旧 motion/survey/approach/route/short-drive 合计 **125 passed / 8.34s**，XML `runs/review/20261112_single_point_v24b/analyzers_final.xml`。共享解析器 SHA 为 `518ed650d66a40d4f4f9e27617cc7ff4314e219efd9f5edc95ebc984cebfdf37`。

## 下一版相机门独立复核

后来相机最小修补 SHA 为 `902424ca5ba3ec208a0028645d377f4ce92ff77e5ee56205c87fac95a9b349cb`，不属于本局已运行源码。它仅将静止识别/上报阶段的当前相机墙钟和正源龄上限设为 `0.5s`，保留负源龄下限 `-0.12s`；重复或倒退源时戳不会刷新到达时间，新增原始 timing 观察对象。

独立检查 8 项相机边界及原 10 项传输回归，共 **18 passed / 12.25s**；作者新增 13 项相机边界另独跑 **13 passed / 9.08s**。原始曝光与 prior 不变，control odom/scan age 仍为 `0.2s`。带当前 liveness、完整 provenance、三个 job 和八条最大状态的 core 为 `47,259 B`，survey metadata 为 `25,290 B`，所有 hook 都通过 64KiB 编码门。

结果在 `camera_transport_independent_final.xml`、`camera_author_boundaries_independent.xml`。这是下一版的本地补丁验证；是否能取得真实匹配、一次上报、合法进入与首点分，仍须下一局平台实测。
