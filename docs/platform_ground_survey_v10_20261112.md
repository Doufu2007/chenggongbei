# 平台地面观察练习 v10：取得连续轨迹与原生图像，未完成转向、未产生比赛成绩

日期：2026-11-12。平台详情 ID：`fbcdc168-a118-4729-8deb-bf5a4ecfb759`；runId：`run_20261112_013814_4b96e433`；平台编号 `#34646201`。

本轮是 `ground_observation_survey` 有界练习，`read_only=false`、`official_scoring=unverified`。它仅发送地面原地角速度，不执行完整巡检，不发送识别、到点、飞行、照射或任务完成消息。**平台终态仍为 `failed`、`score=null`**，失败原因为“流程未检测到完整结束标志（未提取到成绩）”，平台时长 **96.426 秒**。终态证据为 `runs/evidence/20261112/survey_v10_list_after_run.json` 的第一条；较早 detail 的 `submitted`/空 runId 已过期。

## 本轮直接取得的证据

- 探针快照时长 **25.651 秒**，状态 `FINISHED`、原因 `odom_stale_or_invalid`。
- 原始回调 44 行：1 条 launcher_started、1 条 probe_started、37 条 probe_snapshot、4 条 image_sample、1 条 launcher_exit；退出码 0。
- 独立 compact 轨迹 **234 行**、实际命令发布记录 **553 行**，两组均重组完整、未报告丢行或冲突。
- **4 张 800×800 的真实原生 JPEG** 均已完整拼接、通过 SHA256 与实际解码校验；另有 4 张独立 thumbnail。
- 核心快照记录 442 次非零与 109 次零速发布。它们是本队发布调用计数，不是控制器接受回执；compact 命令的采集时刻稍晚，不能强行要求两个快照计数完全相等。
- 收到 `/odom` 与 `/uav/odom` 各 1,148 条、`/clock` 251 条、`/scan` 201 条、`/tf` 2,257 条；双相机各 75 帧。

本轮公开抽签为 `ground_2,ground_0,ground_3`，只代表本局顺序。探针据未确认的静态候选映射，选择首个点的观察方向约 **−90.0013°**；点号映射仍 `candidate_unverified`，不能转写为裁判目标类别或答案。

## 转向没有达到预期方向

| 探针墙钟秒 | 状态 | 原因 | 请求角速度 rad/s |
|---:|---|---|---:|
| 0.0629 | WAIT | clock_stale | 0 |
| 0.1138 | WAIT | start_stability_dwell | 0 |
| 1.1328 | TURN | bounded_yaw_turn | −0.15 |
| 23.5956 | STOP | odom_stale_or_invalid | 0 |
| 25.5788 | FINISHED | odom_stale_or_invalid | 0 |

最终 odom 的观测偏航角约 **−8.51967°**，初始约 **−0.00120°**；它未到达候选目标约 −90°。这轮不能标为 `target_yaw_reached`，也不能认定已看到首点完整目标。存在连续轨迹与命令时间并不自动证明控制响应因果；当前也没有足够证据把偏差归因于某个桥接增益、限速或控制模型，下一轮需围绕同时间窗逐项核对。

`STOP` 转换时，`/clock` stamp=27.9，odom stamp=27.64，仿真时间相差 **0.26 秒**，超过 0.2 秒新鲜度门限；odom 墙钟接收年龄约 0.155 秒，scan 仍 fresh。此次停机正确使用了双时间门禁，不能为完成转向而放宽或移除该门限。后续数据恢复 fresh 时状态仍停机，未重新开始旋转。

## 末图缺少有效曝光位姿关联

双相机首图 stamp=5.174，分别关联本源先前 odom stamp=5.16，skew=0.014 秒，通过“先前本源位姿”检查。双相机末图 stamp=29.841，均记录 `odom=null`、`association_error=prior_odom_too_old`，分析器标为 `invalid_or_unverified_pose_association`。

因此，末图可以用于查看实际可见画面，**不能据末图断言精确相机朝向、目标坐标或投影对准**。不可用地面 odom 替换 UAV odom，也不可把邻近末尾的另一个 pose 补填成有效曝光位姿。

## 回传完整性与分析边界

分析器 `status=partial`，`parse_errors=[]`、`conflicts=[]`、生命周期无缺项。独立 `survey_metadata`、compact trajectory/commands 与 4 张 native JPEG 均完整。大 full 快照未能作为单条事件发送；其冗余 fallback 的 `survey.trajectory_10hz` 又超过 64KB，已通过 `evidence_errors` 明确报告。因此不能把本次分析总状态改为 `collected`；下一版应避免重复发送已独立分片的重型 survey 字段，保留异常证据。

回传记录是本队诊断通道，不能充当裁判或成绩来源；`FINISHED`、退出码 0、图像完整、匹配订阅及发布计数都不能替代官方任务完成与得分。

独立代码审查另复现了两处待修补缺口：活动期非法 ground odom/scan 可被下一合法样本在检查前恢复；错误 UAV odom 后旧历史仍可被用作曝光关联。它们是已复现的代码问题，尚未证明本局触发。详见 `docs/continuation_state_20261112.md`；修复应进入新包，不能覆盖本轮产物。

## 原始证据与台账

- raw：`runs/evidence/20261112/callback_survey_v9.jsonl`，304,715 B，SHA256 `88a450ae27ca8dc63bdc376dfe0ea4fe99154623da76c40d05c639eb0fa649f9`。**虽然文件名含 v9，此文件只属于本次 v10，不能记为另一次 v9 实测。**
- 产物：`runs/evidence/20261112/survey_v10_artifacts.json`；原提交与终态列表位于同目录。
- 分析：`runs/analysis/20261112_survey_v10/summary.json`，真实 JPEG 在同目录；未因本报告重复生成或改写该分析目录。
- 台账：`runs/records/ground_survey_v10_fbcdc168.json`，`kind=interface_probe`，不进入完整任务完成率与成绩统计。

v10 ZIP 与脚本保持上传版本不变。下一步应先修补停机/关联缺口、解释同时间窗内的转向偏差、重取有效曝光图，再验证首个真实地面点计分。
