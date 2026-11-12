# 有限正 Z 响应实现独立审查

## 修后独立复核结论

2026-11-12。**Approved，限定为独立有界正 Z 响应实验；原三项阻断均已修复。** 最终复核源码 SHA256 为 `1ee83ca7f52a5e1e60de63f778e7caae939b9d4645f510d064bfc57f3016fd78`，脚本 SHA256 仍为 `e161324e54bd411e6d14e0051c28bb1d4f4a8454270d63fb64cd4adf262e3baf`。本结论替代下文旧快照的 Changes requested；旧问题与原始反例仍保留为历史。

逐行检查了原始冻结源与修后源的差异，以及新增 20 项回归，没有发现本次修补带来的新阻断。作者的 121 项关联通过为作者证据；本次没有重复同一套测试。独立重新执行以下三个原反例，源 SHA 均匹配最终快照：

| 原反例 | 修后实际结果 |
| --- | --- |
| 同 stamp 的 .50m 近回波／NaN | 两例均立即 STOP，正 Z 输出 0；旧 scan 已清除，`hazard_hidden=false` |
| 已有正 Z 后地面 publisher 异常 | guard STOP，停止调用不再抛异常；当次仍成功尝试并返回 UAV 全零，`uav_zero_attempt_returned=true` |
| norm=1.0009、真实 TF 旋转偏差 .04rad | 拒绝为 `static_laser_tf_mismatch`，输出 0；不再越过 .005rad 门 |

三个原 probe 文件未改。修后 metrics 保留在原同名文件；原始失败 metrics 与旧源、旧测试、旧 manifest 保存在 `runs/review/bounded_vertical_20261112/before_review_fix/`。当前完整 manifest 文件校验和冻结地面源哈希也一致。

扫描修复先校验新回调的实际完整环扫和危险距离，再对完整 payload 去重；仅相同重复 payload 不刷新活性。发布修复把地面／UAV 各次尝试隔离：地面异常将正命令改为 UAV 零；UAV 正命令异常也尝试零，并让 tick 返回当前锁停状态。失败发布仍只代表一次不确定的本地 transport 结果，不能推断命令是否被执行器接收；成功计数与返回的 command records 也只表示本地 publish 返回。TF dot 除双方 norm；odom tilt 除 norm²；原 q 继续保留。

**批准不代表平台飞行已验证。** 静态上方空间仍依赖明确的公开候选柱假设；非零正 Z 响应、停止／悬停效果与机体原点尚需实际观测。任何地面任务计分局不应在三点完成前使用此非零 UAV 实验；此实验不发送任务完成或伪造成绩。

## 原快照审查与复现记录

2026-11-12。结论：**Changes requested，暂不批准平台上传。** 本轮审查源快照为 `bounded_vertical_response.py` SHA256 `d6d002d2a9a0bfe8e4962ef7b49323327460ea85459f946c37e789ef74016869`。未修改生产源码、上传或运行 ROS；下列反例只运行真实 guard 与模拟传输。没有重复执行作者已报告的 101 项关联测试。

## 需修复的问题

### P1：同 stamp 的危险扫描被完整忽略

`bounded_vertical_response.py:277` 在比较源 stamp 相同后直接返回，早于 ranges 和几何合法性校验。同 stamp 本应不能刷新活性，但新的危险回调也不能借旧清晰记录继续放行。

独立复现：guard 已经 ASCEND，最近 scan stamp 为 `11.1`、min 为 `2.0m`；在 wall `1.15s` 注入同 stamp 的 `.50m` 近回波或 NaN。两种情况都继续保留旧 min=2.0，随后 `step` 输出正 Z `.05m/s`。

```text
python runs/review/bounded_vertical_20261112/independent_duplicate_scan_probe.py
```

证据：同目录 `independent_duplicate_scan_metrics.json`，两例 `hazard_hidden=true`、`all_expected_stops=false`。

修复验收：每条实际扫描先验证危险／非法内容；仅完整内容相同的重复曝光可以被忽略，且不能刷新 arrival。重复 stamp 内容变化应锁停，不能沿用旧安全输入。新增针对近距、NaN 和内容一致但时间过期的回归。

### P1：地面发布失败会阻止 UAV 零命令尝试

`bounded_vertical_response.py:474` 先发布地面零，再发布 UAV；两者之间没有异常隔离。已有正 Z 后，若地面 publisher 抛异常，`node.stop` 虽把 guard 锁为 STOP，却在第一条地面零发布处异常退出，完全没有尝试 UAV 零。看门狗复用同一个 `_publish(0)`，因此也会被相同故障持续挡住。

```text
python runs/review/bounded_vertical_20261112/independent_publish_fault_probe.py
```

证据：同目录 `independent_publish_fault_metrics.json`。停止前最后 UAV 请求为 `.05m/s`；停止时 `following_publications=[]`、`uav_zero_attempt_returned=false`。

修复验收：停止路径的各零输出独立尝试，单个通道异常不能抑制其他通道归零；记录每个实际尝试／返回／异常。正命令前的地面发布失败应进入失败锁停并尝试 UAV 零，不能继续发布正 Z。至少覆盖 stop、回调触发停和 watchdog 的同类失败。

### P2：TF 角度比较没有归一化允许范围内的四元数

`bounded_vertical_response.py:312` 允许 quaternion norm 误差 `.001`，但后面的角度 dot 直接使用未归一化四元数。norm=1.0009 时，真实归一化旋转偏差 `.04rad` 的 TF 仍能超过标称 `.005rad` 门的 dot 阈值。

```text
python runs/review/bounded_vertical_20261112/independent_tf_normalization_probe.py
```

证据：同目录 `independent_tf_normalization_metrics.json`，`normalized_angular_error_rad=.04`、`comparison_threshold_rad=.005`，仍 `accepted=true`、正 Z `.05m/s`。同模块 `_pose` 的倾角派生也应使用 norm² 或归一化副本；原始回调证据继续保留，不应改写成归一化后“原始”消息。

修复验收：允许数值 norm 误差时，几何比较必须使用归一化副本；正负等价 quaternion 都应成立，超角度真实偏差必须拒绝。

## 已核对但不等于平台验证的部分

- 候选柱限制初始 xy 半径 .05m、z .035–.075m，并在水平半径中包含完整 .447m 外廓、倾角、净空及 .04m 漂移；上下范围包含硬上升界。源常量列出的 12 个静态墙／窗盒和公开 world 哈希有作者逐项核对测试，来源不含私有裁判／贴图分配。当前平台三维环境仍是明确假设。
- origin 与 `motion_started_wall` 只首次建立；ASCEND／PAUSE 的 10 秒从该墙钟计算，暂停不清累计。全程 45 秒、.20m 硬升、.04m 漂移、.30m 路径与反馈速度／角速度检查存在。
- clock／odom 重复源 stamp 不刷新活性；时间倒退会锁停。公开 scan 的上述重复内容缺口另列为 P1。
- 描述性图查询放在 command lock 外；每 tick 与发布边界取实际 DDS 匹配数，独立 watchdog 使用同 command lock 串行发送零。发布通道异常隔离缺口另列为 P1。
- 正常达到 .15m 后转零并观察至少 5 秒；不假设零速度可以悬停，不请求下降、灯、识别、到点或完成消息。
- gzip 使用 mtime=0，SHA、字节数、分片序号和 session 均存在；`official_score=null`、`full_mission_complete=false` 保留。启动脚本只运行上传本队源码，不启动仿真／裁判，退出 trap 保留实际退出码。

作者报告的正常路径和传输测试不能覆盖上述三个独立反例。修复后应先对这些反例定点复核，再作是否上传的判断。
