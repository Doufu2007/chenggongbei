# v16 原始时间配对与图查询节流：定点只读审查

日期：2026-11-12。仅审查 `ground_bounded_approach_sync.py`、共享 ROS adapter 的同步/锁边界及新启动脚本；未改控制源码或 v15 产物，未重复运行全套。实现代理正式冻结后已逐文件复核哈希一致。

| 文件 | SHA256 |
|---|---|
| `ground_bounded_approach_sync.py` | `6c4f5c5b5d12fff3c7311193381155b3799305be031bd41e556b5a4b0e60be3b` |
| `bounded_ground_probe.py` | `a362862525c5cc6c0a9ef1c217146b739638ceefddaecac6a69064179cd1e340` |
| `run_ground_bounded_approach_sync.sh` | `be785b79c2409815035e8155a948a631c48334b76b41c1b1204d3055f926a6da` |
| `test_ground_bounded_approach_sync.py` | `e25e6834275f0c2ec52fd482555bbef48776367839e911f14c5e6b13f500282c` |

## 当前结论

**冻结版可进入当前实际 360 束平台环境中同距离、同档位的时序修补练习。** v16 继承 v15 的 0.20 m 观察距离、0.03 m/s 请求、0.30 m 全局路径/位移、90 秒全局、45/55 秒阶段预算、.641 m 全环门槛及原有停车/恢复边界。它不包含提速或长路线放行结论。

本地同步专项与三项共享 adapter 看门狗/图查询检查合计 **18 passed in 1.29 s**；另有四组离线分析检查 **81 passed in 3.81 s**。这些均是本地合成验证，不是 v16 平台成功或官方分数。

正式冻结前最后一处修改仅校正诊断统计：根据真实 prior arrival 是否晚于 scan arrival 标记 `late_prior`，已存在 prior 延后被使用则另计 `deferred_existing_prior_promotions`。定点复核源码与新增回归 **1 passed in .14 s**，没有改变控制条件；未为此重复全部专项。

## 配对与停止边界

- pending scan 保存原始 header、scan 到达墙钟及 payload；已配 scan 保存原始 `wall`，新未配 scan 不刷新它。暂存队列上限 4、trace 上限 12，有丢弃计数。
- 新 scan 先经原全环覆盖、有效返回、最小距离阈值验证；unsafe/invalid 或倒退 stamp 立即锁停，不能借仍 fresh 的旧 pair 忽略危险数据。
- 后到 odom 可与 pending scan 重新配对；实际 prior 必须 `prior.stamp<=scan.stamp`，skew `<=.05s`，不借未来 pose；原始 scan 墙钟 age 与相对 clock age 保持原门禁。没有改写接收年龄来延长 freshness。
- 前一 pair 尚 fresh 可继续供原控制条件检查；它到期且新 pair 未形成时仍归零 PAUSE。配对成功只补足输入证据，恢复仍需原静稳驻留与最多 3 次恢复，预算不重启。
- STOP/FINISHED 或 epoch 无效会清 pending，不再 promote。invalid/reversed odom、reversed scan、clock reset 后的健康样本不能复活该局。
- odom 回调的完整校验、记录与 pending promotion 在同一 guard RLock 内完成；只延后该回调内的时序检查，路径/姿态等硬边界仍保留。新配对不重置全局累计运动。

## 图查询与看门狗

共享 adapter 默认 `graph_refresh_interval_s=0`，保留每 tick 刷新行为；只有 v16 工厂传入 `.5s`。描述性图查询不持 `command_lock`，阻塞时独立看门狗仍能发送零速；首次运动起的全局 deadline 不因阶段改变重置。

每 tick 先查 `/cmd_vel` publisher 的真实 DDS subscription count；活动期匹配丢失时先 STOP/发零，再决定是否查询描述性图。缓存的正匹配值不能独立放行运动。描述信息刷新异常也锁停并发零。

`graph_refresh_diagnostics` 记录的是 adapter 对 `get_subscriptions_info_by_topic` 的查询次数/耗时以及独立匹配检查计数；不覆盖构造时所有 ROS graph API，也不能据该值计算全部平台开销。实际键为 `graph_query_total_wall_s` / `graph_query_max_wall_s`。

## 离线证据合同

`graph_refresh_diagnostics` 作为可选 object 保留到 assembled，旧版本不增加必填项；新 `approach.revision` 与 `approach.sensor_sync` 原值保留。原始 mode 仍为 `ground_bounded_approach`，不改 raw 或假装不同任务。只读解析保真测试先复现 3 个失败，再通过四组 81 项回归。

v16 是否减少持续 PAUSE、是否完成 .20m 与三帧采图，必须由下一局真实回传确认；无裁判完成标志时仍记 failed/null，不以同步计数或退出码替代成绩。
