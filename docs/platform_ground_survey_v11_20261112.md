# 平台地面观察练习 v11：证据回传完整，雷达过期后停车，尚未完成转向或计分

日期：2026-11-12。平台详情 ID：`1da890da-6b9e-4701-8efb-934da9850219`；runId：`run_20261112_014851_4596f50c`；平台编号 `#84264286`。

本轮是 `ground_observation_survey` 有界练习，`read_only=false`、`official_scoring=unverified`。它只发送地面原地角速度，线速度为零；不发送识别、到点、飞行、激光或任务完成消息。**平台终态为 `failed`、`score=null`**，失败原因为“流程未检测到完整结束标志（未提取到成绩）”，平台时长 **59.393 秒**。终态以 `runs/evidence/20261112/survey_v11_list_final.json` 第一条为准，更新时间 `2026-11-12 01:52:21.713795+08`。

## 已取得的真实证据

- 探针快照时长 **9.190 秒**，状态 `FINISHED`，原因 `scan_stale_or_unsafe`。
- raw **18 行**：launcher_started 1、probe_started 1、probe_snapshot 11、image_sample 4、launcher_exit 1；退出码为 0。
- compact trajectory **85 行**、actual_commands **228 行**，均完整，无丢行/冲突；4 张 native JPEG 与 4 张 thumbnail 均已重组并解码。
- 4 张 native 图均为 **800×800**，首图与 `last_associated` 图各双相机一张，均保留真实 header、frame、stamp、JPEG SHA256 和本源 prior odom。
- 核心快照记录 **119 次非零、108 次零速**发布。它们是发布调用计数，不能代替控制器接受回执；compact 命令采集更晚，共 228 行，不应把两个快照计数强行合并。
- 收到 ground odom 442 条、UAV odom 441 条、scan 71 条、clock 89 条、TF 769 条；双相机各 27 帧。

本轮公开抽签为 `ground_2,ground_1,ground_4`，只表示本局顺序。首点观察方向约 **−90.0007°**，由尚未确认的候选静态点号映射计算；该映射仍为 `candidate_unverified`，不是目标类别答案。

## 实际转向与停机

| 探针墙钟秒 | 状态 | 原因 | 请求角速度 rad/s |
|---:|---|---|---:|
| 0.0639 | WAIT | clock_stale | 0 |
| 0.1148 | WAIT | start_stability_dwell | 0 |
| 1.1345 | TURN | bounded_yaw_turn | −0.5 |
| 7.1874 | STOP | scan_stale_or_unsafe | 0 |
| 9.1680 | FINISHED | scan_stale_or_unsafe | 0 |

探针在 TURN 状态累计观测旋转 **0.463090 rad，即 26.5331°**。最终 odom 偏航角约 **−27.15847°**，初始约 **−0.00120°**；最终角度包含停止前后余下的观测变化，与 TURN 期累积旋转是两个不同指标。**没有到达约 −90°，不能记为转向完成或任务完成。** 也不能用这一次请求角速度与角度比值直接确定平台增益或桥接模型。

STOP 转换时，scan 墙钟接收年龄 **0.2481268 秒**，超出 0.2 秒门限；clock stamp=9.6、scan stamp=9.4，仿真时间差约 0.2 秒。ground odom 墙钟年龄约 0.16839 秒、仿真时间差约 0.12 秒，仍 fresh。停机由雷达接收墙钟过期触发，符合双时间门禁。之后传感器恢复 fresh，程序保持停止，没有重启旋转。

本局未发布 UAV 控制。候选机体半径、雷达偏移及速度上限仍是有界实验配置，不构成官方机体尺寸或控制语义验证。

## 曝光关联与传输改进

双相机首图 stamp=2.815，分别关联本源 prior odom stamp=2.8，skew=0.015 秒；双相机 `last_associated` 图 stamp=11.483，分别关联本源 prior odom stamp=11.48，skew=0.003 秒。四图均通过分析器的 `valid_prior_pose_observation` 检查；该检查只确认收到的先前位姿及时间差，不验证相机外参、投影或目标识别。

本局 `last` 与 `last_associated` 的原图 stamp 相同，发送端据 topic/stamp 去重，因此没有再发送另一张相同末图；metadata 仍保留两个阶段记录。这属于真实图像去重，不是补图或补造曝光位姿。

分析器为 **`status=collected`**：生命周期、full snapshot、独立 metadata、两组 compact series 与图像分片完整，`parse_errors=[]`、`diagnostic_errors=[]`、`conflicts=[]`。v10 的冗余大 trajectory 回传问题在本局未出现。**`collected` 仅表示诊断证据收齐，不表示比赛成功。** 退出码 0、FINISHED、图像完整及匹配订阅均不能替代官方成绩。

## 后续修补与证据边界

上传的 v11 包已修复活动期非法 odom/scan 未锁存，以及非法曝光 odom 历史残留。但独立审查发现，数值合法的瞬时超速、位移越界或 tilt 仍需在回调中锁停，防止下一正常样本在控制检查前覆盖它们；当前 v11 archive 保留当时版本，不回写修改。尚无证据证明该缺口在本局触发。

下一版拟增加有界 PAUSE：临时数据过期先停，重新满足稳定、新鲜和安全门禁后才能继续同一次观察；非法数据、越界、超速、累计预算或总时限仍必须永久停止。具体行为须在冻结后独立测试，不能将尚未完成的补丁当作已验证。

## 文件与台账

- raw：`runs/evidence/20261112/callback_survey_v11.jsonl`，220,299 B，SHA256 `e926f696db4d4ff2939dc666e63eac334bc1e0b0f112b14bf1813ecd2465a370`。
- 分析：`runs/analysis/20261112_survey_v11/summary.json`，native JPEG 同目录；本报告未改写分析目录。
- 上传源码：`dist/inspection_ground_survey_v11_src.zip`，78,135 B，SHA256 `dff72b3926ee14e161ec2165e2d9908ecea207bb4ee4b7ca53d1fcd20a39db81`。
- 脚本：`dist/run_ground_survey_v11.sh`，2,845 B，SHA256 `022494c64d58e7a16062688afbd3aec908f71d0af29b79bf8587064f58b461e0`。
- 产物：`runs/evidence/20261112/survey_v11_artifacts.json`；台账：`runs/records/ground_survey_v11_1da890da.json`，`kind=interface_probe`，不进入完整任务完成率或成绩统计。

上述 ZIP/脚本已按产物记录重新核对字节数与 SHA256，保持上传版本不变。本项目仍无有效官方任务成绩、单点得分或完整任务完成证据。
