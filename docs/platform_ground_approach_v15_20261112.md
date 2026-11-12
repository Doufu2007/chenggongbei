# v15 地面有界接近观察：平台真实证据

日期：2026-11-12。此次是练习诊断，不是完整任务或官方计分提交。

## 结论

当局公开首点为 `ground_1`，候选方位约 −90°。本局首次在该角度范围完成真实转向、零速静稳并进入前进；实际前向投影约 **0.120630 m**，尚未达到本次 0.20 m 观察距离，因 `approach_drive_wall_time_limit` 停机，未进入 CAPTURE，capture_count=0。不能据此声称已到正式任务点、识别成功或取得任务分。

**主要阻塞是长暂停，而非仅速度档位。** 55 秒前进阶段预算内，实际 DRIVE 命令区间合计约 24.139203 秒，PAUSE 约 30.904403 秒；第二次暂停持续约 29.754981 秒直至预算用尽。有效匹配行走区间观测到约 0.00498 m/墙钟秒，但不能把全阶段距离除以 55 秒解释为持续行走速度，也不能从请求 0.03 m/s 与该观测推断线性增益。

## 平台终态与产物

- 详情 ID `87616485-22d2-4849-8b8d-0b524d63c4b3`；runId `run_20261112_024853_b133c7ab`；编号 `#73170413`。
- 终态以 `runs/evidence/20261112/approach_v15_list_final.json` 的相同 ID 为准：**failed / score=null / durationMs=169902**，更新时间 `2026-11-12 02:52:18.293109+08`，原因“流程未检测到完整结束标志（未提取到成绩）”。平台总时长 169.902 秒与探针快照 87.832 秒属于不同计时口径。
- ZIP `dist/inspection_ground_approach_v15_src.zip`：90,837 B，SHA256 `966557c325cefbf49dbaef99af6334007bad1749d2fc6bb9080d3c277bb17ae8`。
- SH `dist/run_ground_approach_v15.sh`：2,836 B，SHA256 `8e81a36c1c6d4f214428288850816f4159b31a75477807a597b95d7b1cfe3b9b`。
- 原始回调 `runs/evidence/20261112/callback_approach_v15.jsonl`：531,068 B，SHA256 `0e205c23a3ed85392c7cff019788a644da1eb2f22261f8efe16719bc862a8a8f`。62 个事件，launcher_exit=0；退出码仅说明该脚本结束。
- 源码冻结值及独立审查见 `docs/control_review_approach_20261112.md`；未改上传包或原始回调。

## 时序与真实位移

下表时间均为记录器墙钟 elapsed；clock 列为收到命令发布时保留的仿真 clock，二者不能混用。

| 阶段/首次命令 | wall elapsed (s) | clock (s) | 请求 |
|---|---:|---:|---|
| TURN | 1.145946 | 3.9 | angular_z −0.5 |
| TURN_SETTLE | 29.662693 | 31.9 | 零速 |
| DRIVE | 30.738107 | 32.9 | linear_x 0.03 |
| PAUSE 1 | 31.416731 | 33.6 | 零速 |
| 恢复 DRIVE | 32.566153 | 34.7 | linear_x 0.03 |
| PAUSE 2 | 56.026732 | 57.9 | 零速 |
| STOP | 85.781713 | 87.1 | 零速 |
| FINISHED | 87.813311 | 89.1 | 零速 |

TURN 命令至静稳阶段约 28.516747 秒，冻结前进朝向 −88.603097°；候选目标 −90.001552°。末轨迹约 −88.576193°。完整累计角 1.548831122 rad 包含其他微小姿态变化，不替代 TURN 专属角度。

前进原点为 (−0.005120818, −0.005675939)，最终真实前向投影 0.120629887 m，横向 −0.002318702 m；全程累计路径 0.134585281 m、起点最大位移 0.126415876 m，均未达到 0.30 m 硬上限。首末里程计差约 0.126415944 m 含转向漂移及停车阶段，不得当作前进投影或控制增益。

## 暂停与扫描关联缺口

第二次暂停后没有再发非零控制。最终 scan_anchor=null；源 motion_log 已丢弃早期 239 条，保留日志中有 31 条 `odom_stale_or_invalid` 与 30 条 `scan_anchor_unavailable` 交替。wall 79.69 秒附近，odom stamp81.0、clock81.1，odom 已满足 freshness；scan stamp81.152 与当时可见 prior odom 相差 .152 秒，仍无法满足 scan/prior ≤.05 秒的关联门禁。

代码按 scan 回调当时已到达的 odom history 选取 prior，并先清空当前 anchor；后到 odom 不重配待配 scan。这与延迟 odom 回调造成 anchor 缺口相容。由于未保存完整逐回调历史，不能断言每一次暂停的具体原因。

下一版应先验证**待配 scan 与后到 prior odom 的同步**，保留 prior≤scan、skew≤.05 秒、当前新鲜度与局部移动/停车预算；不以放宽时间门禁消除暂停。随后在同一有界距离上测试新速度档位与停止距离，再决定较长路线预算。`.10/.20` 仅可作为待验证请求档位，现有证据不证明其速度、刹停或净空安全。

详细阶段速度、停车边界和 scan 几何独立复算见 `docs/approach_v15_motion_review_20261112.md`。

## 图像与证据完整性

首轮解析 `runs/analysis/20261112_approach_v15/summary.json` 为 **partial**：702 条轨迹、2,412 条实际发布命令、六张 native JPEG 已重组，trajectory/commands dropped=0，无 parse/assembly/diagnostic/conflict 错误。收到 core 但没有 full snapshot；14 个冗余 `survey` 分片原样保留为 unknown_sections，未改造成新模式答案。motion_log_dropped=239 必须另行保留，不能据 compact series 完整声称逐回调日志完整。

六张 native 是双相机 first/last/last_associated，不是 CAPTURE 三图。双 first 及双 last_associated 有有效 prior pose；双 last 的 prior odom 太旧，关联无效。UAV 末关联图可见窗框、地标及地面斜视照片，已实际目视检查；类别、点号映射、相机外参与识别上报仍未验证。

上传后发现离线解析器可能在整组 capture 事件全失时误记 collected，已先复现再修复；新旧观察专项 51 项通过。修后另存 `runs/analysis/20261112_approach_v15_capture_check/summary.json`，仍为 partial，expected_capture_phases=[]、missing_capture_phases=[]，因为源 count=0。没有覆盖首次分析或 raw。count=0 与没有传输缺图并不代表观察任务完成。

台账 `runs/records/ground_approach_v15_87616485.json`；全台账 **18 条、0 重复、files_verified=true**。完整任务有效计分局仍为 0。
