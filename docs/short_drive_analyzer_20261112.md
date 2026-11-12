# 短直行模式的离线证据解析

日期：2026-11-12。新增入口：

```powershell
python tools/analyze_ground_short_drive_evidence.py INPUT.jsonl --output OUTPUT_DIR
```

默认旧入口 `analyze_ground_survey_evidence.py` 仍仅接受 `ground_observation_survey`。新 wrapper 显式传入 `expected_mode="ground_short_drive"`，输出 `kind=ground_short_drive_analysis`。公共核心验证器也通过参数接收期望模式，默认仍为 `bounded_ground_motion`；没有改写模块全局常量，也没有为通过验证临时重标日志 mode。

原始 `survey_metadata`、`survey_motion_series`、`survey_image_part` 是观测传输格式名称，保持原值。新模式的 `survey` 容器原样保留 `mode=ground_short_drive` 以及 `drive` 对象。不会添造 target_yaw、候选点号映射或目标答案。startup、lifecycle、原始分片、DRIVE/PAUSE/STOP 状态、图像 SHA256、header 和曝光位姿均保留来源数据。

显式 snapshot/startup/lifecycle 模式不一致，以及新元数据顶层或 drive 对象 mode 不一致，都会使解析结果 invalid。源文件始终只读。`trajectory_dropped` / `actual_commands_dropped` 必须为非负整数；即便保留下来的分片齐全，源端有掉行也只能 partial。

验证：新模式及 CLI 8 项先观察失败再实现；随后真实 `GroundShortDriveGuard.additional_events()` 无重标兼容通过。掉行 6 项同样先红后绿。合计 probe、camera、ground-motion、survey、short-drive、response 六组 **92 passed in 3.44 s**。

用 v12 原始 raw 在独立临时目录重跑，与已有 summary 逐键比较，只有 notes 新增了 `short_drive_distance_reached` 同样不是比赛成功的说明；其余字段完全相同，原分析目录未改。

平台 v14 的真实 raw 已直接解析到 `runs/analysis/20261112_short_drive_v14/summary.json`：18 事件、4 张 native JPEG、73 条轨迹、193 条命令，无模式错误、无掉行、无解析或分片冲突。collected 只说明诊断证据完整；距离达到、退出码 0、FINISHED 与发布次数都不能替代官方任务得分。评分与任务完成字段继续为未知。
