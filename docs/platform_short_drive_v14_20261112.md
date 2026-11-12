# 平台短直行 v14：实测投影达到 2 厘米后停车，无比赛计分

日期：2026-11-12。详情 ID：`2aeffd1d-eb67-4c2d-a8e2-a5fc8b9f065b`；runId：`run_20261112_021755_cef799e3`；编号 `#84358854`。

**平台终态 failed、score=null，时长 43.142 秒**，原因仍为“流程未检测到完整结束标志（未提取到成绩）”。来源 `runs/evidence/20261112/short_drive_v14_final.json`。本轮是独立 `ground_short_drive` 控制验收，仅发送 `/cmd_vel` 前向请求，未发送识别、到点、飞行、激光或完成消息。距离达到与诊断 collected 不能记为比赛成功。

## 控制与实测投影

配置为前向请求 `.03 m/s`、实测沿初始 yaw 投影达到 `.02 m` 停；首次运动后 10 秒总墙钟预算包含暂停，全局 25 秒；位移 `.06 m`、偏航误差 `.05 rad`、反向 `.005 m` 硬限，`.2 s` 双时间新鲜度，fresh 静稳满 1 秒后最多恢复 3 次。该局 resume_count=0。

| 实际命令状态 | 首条墙钟秒 | 请求线速度 m/s | 请求角速度 rad/s |
|---|---:|---:|---:|
| WAIT | .056251 | 0 | 0 |
| DRIVE | 1.120600 | .03 | 0 |
| STOP | 5.355516 | 0 | 0 |
| FINISHED | 7.355107 | 0 | 0 |

非零命令窗口 4.234916 墙钟秒。motion_log 的 STOP 5.3801 秒晚于首条实际零速，表格采用实际命令边界。程序快照 7.366 秒，原因 `short_drive_distance_reached`，随后没有重启。核心快照记录 84 次非零、108 次零速；compact 命令采集更晚，共 193 行，不合并为同一时刻的计数。

真实初始原点 `[.0000377842447,.00000800762335,.0550000771304]`，初始 yaw `−.0000209028243 rad`。末时 `drive.measured_forward_m=.0202138320583 m`，最大观测位移 `.0202144344615 m`。元数据中的 `first_motion_wall_s=9038.063910037` 是进程 monotonic 绝对时间，不是 elapsed；命令表与 trajectory 采用各自 `wall_elapsed_s`，不直接混用。

独立用 compact 的位置和上述原点、yaw 计算

`forward=(x-origin_x)*cos(initial_yaw)+(y-origin_y)*sin(initial_yaw)`

得到末投影 `.020213812499 m`，与高精度元数据差约 `.00000001956 m`，属于记录数值精度差。STOP 前最后一条轨迹为 5.333974 秒，投影 `.019929512448 m`；STOP 后首条为 5.435820 秒，投影 `.020088812465 m`。它们距命令归零边界分别 `.021542` 与 `.080304 s`；因此不能插入不存在的“精确停止点”。从后者至末轨迹，收到的位置投影再变化约 `.000125000 m`；这是停止后接收区间内的观测变化，不能单独归因为制动漂移。

轨迹中相对初始 yaw 的最大差约 `.001230897 rad`（.0705°）。首末 odom 位置差 `.020224568 m`、沿初始方向投影、最大位移是不同指标；不能把首末差值或命令积分当作响应增益。发布调用也不是控制器接受回执。

## 图像与回传

raw 18 事件：launcher_started 1、probe_started 1、probe_snapshot 11、image_sample 4、launcher_exit 1，末退出码 0。原始 `ground_short_drive` mode、DRIVE 状态及 `survey_*` 传输 section 均保留，未重标成旧巡航日志。

分析 `runs/analysis/20261112_short_drive_v14/summary.json` 为 **collected（仅诊断完整）**：73 条 trajectory、193 条 actual_commands、4 张 800×800 native JPEG 和 4 张 thumbnail 均完整，掉行 0，无 parse/assembly/diagnostic/conflict 错误。

双相机首图 stamp=3.154、各自 prior odom=3.14，差 `.014 s`；末关联 stamp=10.154、各自 prior odom=10.14，差 `.014 s`。四图均通过接收证据的先前位姿关联检查；该检查不证明相机外参、投影或目标识别。

已实际查看末图：地面相机接近均匀绿色；UAV 相机可见窗框、地标和边缘裁切图块。当前画面不足以宣称某一官方目标类别，不能据裁切片段补造识别结果。

## 固定产物

- raw：`runs/evidence/20261112/callback_short_drive_v14.jsonl`，210,759 B，SHA256 `3929b4de07037746136e621517bf2060bbd2ab760116df21e0f737e3fc3078bf`。
- ZIP：`dist/inspection_short_drive_v14_src.zip`，83,210 B，SHA256 `808cb3ed4b0f86ab7c4a338eb4b5557bb2f7eb64f05211496d32b3215069d92a`。
- SH：`dist/run_short_drive_v14.sh`，2,800 B，SHA256 `afdbfe444b01ac503a93c4dd5b6d84dbd6c28408712e1ed1d3ca31e5e5011c59`。
- manifest：`runs/evidence/20261112/short_drive_v14_artifacts.json`；台账：`runs/records/short_drive_v14_2aeffd1d.json`。

离线新模式与掉行判定回归见 `docs/short_drive_analyzer_20261112.md`，六组 92 项通过。项目仍无有效官方任务分，后续须取得真实目标观察与单点裁判计分证据。
