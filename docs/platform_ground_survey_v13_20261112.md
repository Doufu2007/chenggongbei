# 平台地面观察 v13：候选方向到角停机，仍无比赛成绩

日期：2026-11-12。详情 ID：`e8cd8e60-fddb-4fdc-9b65-25349077328a`；runId：`run_20261112_021236_269a9fac`；编号 `#19225516`。

**平台终态 failed、score=null，时长 65.001 秒**，原因“流程未检测到完整结束标志（未提取到成绩）”。来源 `runs/evidence/20261112/survey_v13_final.json`，更新时间 `2026-11-12 02:14:36.410354+08`。探针的 `target_yaw_reached` 只说明候选观察方向达到程序容差；没有识别、到点、飞行、激光或完成消息，也没有单点或完整任务得分。

## 本局配置与结果

显式最低请求角速 `.35 rad/s`、最高 `.5 rad/s`、转向墙钟上限 45 秒、全局上限 60 秒。保留 `.025 rad` 到角容差、`1.8 rad` 累计角、`.06 m` 位移、`.2 s` 传感器新鲜度及最多 3 次暂停恢复。

- raw 19 事件：launcher_started 1、probe_started 1、probe_snapshot 12、image_sample 4、launcher_exit 1；末尾退出码 0。
- 探针快照 21.703 秒，状态 FINISHED，原因 target_yaw_reached；diagnostic collected，parse/assembly/diagnostic/conflict 均为空。
- compact trajectory 210 行、actual_commands 478 行，分片完整，无掉行。核心快照 362 次非零与 112 次零速发布，采集时间早于末命令分片，不合并为同一时刻计数。
- 本局抽签 `ground_4,ground_3,ground_0`，候选首点 `[1.125,-.775]`；目标方向 `−.603251675 rad`（约 −34.563°），最终轨迹 yaw `−.5793556 rad`（约 −33.194°），残差 `.023896075 rad`（约 1.369°），落在 `.025 rad` 容差内。
- 累计观测角 `.578939357 rad`，最大观测位移 `.005971276 m`；resume_count=0。

| 实际命令状态 | 开始墙钟秒 | 线速度 m/s | 角速度 rad/s |
|---|---:|---:|---:|
| WAIT | .057904 | 0 | 0 |
| TURN | 1.326136 | 0 | −.361938463，随后最低 −.35 |
| STOP | 19.656201 | 0 | 0 |
| FINISHED | 21.677459 | 0 | 0 |

非零转向窗口为 18.330065 墙钟秒。motion_log 的 STOP 记录 19.6965 秒晚于首个实际零速；本报告采用实际命令边界。收到的转角与命令时间关联是观测事实，不能据单局求出桥接增益或模型。

**v12 的候选目标约 −90°，本局只有约 −34.56°，抽签不同。** 此局不能证明原 90° 观察已经完成，也不能将用时/到角差异归因于新最低请求值或新预算；这两项同时改变。

## 图像与关联

4 张 native JPEG 均 800×800，另有 4 张 thumbnail，原字节/SHA256/解码及分片完整。首图双相机 stamp=3.177，对应各自 prior odom=3.16，差 `.017 s`。UAV 末有效关联图 stamp=23.845，对应 odom=23.84，差 `.005 s`；地面末有效关联图 stamp=24.177，对应 odom=24.16，差 `.017 s`。四图均通过先前位姿接收关联检查；两相机末帧时刻不同，不应补成同步曝光，外参、投影和目标识别仍未验证。

## 产物与证据

- raw：`runs/evidence/20261112/callback_survey_v13.jsonl`，244,906 B，SHA256 `652fbf6fd2152d71e3ce211de4715054716d80b5fca98c1c80f3fe0676decc51`。
- ZIP：`dist/inspection_ground_survey_v13_src.zip`，79,204 B，SHA256 `8d175fab024decff6ba275fe7e8b0bab4437627c1ba2dc029e9eb583cdc65ac4`。
- SH：`dist/run_ground_survey_v13.sh`，2,900 B，SHA256 `0e812b18e9a0fe83c9071d33682efb79f255fdfa2308353b40dc878957827040`。
- 分析：`runs/analysis/20261112_survey_v13/summary.json`；manifest：`runs/evidence/20261112/survey_v13_artifacts.json`；台账：`runs/records/ground_survey_v13_e8cd8e60.json`。

本局作为有界接口练习记录，不进入完整比赛完成率或有效成绩统计。collected、FINISHED、到角停机和退出码 0 均不是官方成功。
