# v38 有限恢复资源独立定点审查

2026-11-12；审查者 `/root/axis_final_review`。结论：本轮未发现阻断问题；32 次 / 单次5s / 累计60s / 全局1200s实现与冻结约定相符。未改生产源码、未上传，平台有效性待实际运行。

冻结模块 SHA256 `f50c7058d8a6727ccac83d03c228d6a2c9e01249c55171aa55758a0b272cdc1d`；运行脚本 SHA256 `ae32f4c32c20b1071c0d2eb7426f50c1748a9b9cb415d4547d2662f964fd24ce`。独立窄探针 10/10，通过记录见 `v38_probe_results.json`，静态检查2/2见相邻 v36 review 目录 `static_probe_results.json`。未重复作者全套测试。

- PAUSE 中真实里程计分别令 TURN→TURN_SETTLE、DRIVE→DRIVE_SETTLE；恢复采用更新后的 phase，当次恢复 tick 都返回零。
- 慢源钟时墙钟已过1.2s仍无法通过双钟1s停稳；之后才新建视觉epoch。原安全条件不能由增加的恢复次数代替。
- 4 次独立新鲜输入和停稳恢复逐次得到真实 count 1/2/3/4，账本 resume_count_after 对应一致；另从预置31边界实际恢复至32，下一次请求STOP且计数保持32。这里没有声称独立完整跑过32次；作者全程32次结果不作为本轮重复证据。
- 重复 freshness fault 不重置暂停起点。精确5s处 clock/odom/scan 三类活动 callback 分别在无需主tick时硬停并封账。
- 累计预置59.4s + 本次0.6s，在精确60s由watchdog硬停；completed/active/total账本一致，active清零。
- 保留实际 factory 构造的 BudgetGuard 对象，最终 DDS 查询中制造0.530038013s相机缺口，发布全零。未运行时替换guard。
- REPORT_WAIT及recognition已attempted后的相机缺口均硬停，无新恢复账本或重复尝试。

源码路径复核：v38 在最外层拦截 PAUSE，因此父类原三次上限不提前抢占；所有实际恢复仅在 `_stable_dwell` 合格后真实+1。`_budget_fault` 被主step、watchdog及活动control callback共同调用，stop保留首停原因并封账。父 `_handoff` 只清每点 recapture_count/attempts 等，不清 resume_count、pause records、累计wall或全局起点。每点一次重采与attempt锁保持；v36 factory仅构造时注入subclass。

回传链静态确认：`_sync_observation_context()` 调多态 `approach_evidence()`，完整 camera_recovery/recovery_resource 写入 observations.approach_context，再随 `survey` 进入三点 gzip bundle；轻量 `three_point_core` 不单独承载该资源账本。读取平台结果时应以重组且双SHA验证完整的bundle为准。

包独立校验：`dist/inspection_three_point_recovery_v38_src.zip` SHA256 `b9874ada14c3df72483ea85eedebdfa02fbe1a245bb13da926e6d92bd524b85b`。基底v35的50项逐字节不变，无重复ZIP成员，仅加v36/v38两模块。dist启动器仅比原运行脚本多一行临时 INSPECTION_DIAGNOSTIC_URL；其SHA256 `18d97806f0d941f1f338b42cbab5d10bef44c6b8695130b0c4e540281e65b93f`，没有执行网络请求。

边界：窄探针使用合成可观察输入及本地ROS transport stub；累计与次数上界以预置状态定点测试，不能替代官网持续运行。资源增加可能延长总等待，但原阶段deadline、1200s全局deadline、几何/运动/传输门保留。未声称实际连续三点完成、裁判接受或官方分数。

复现脚本在相邻 v36 review 目录 `independent_probe.py`。首次合成DRIVE探针缺少父类要求的leg_records，仅补齐本地探针夹具后通过；未修改生产代码。
