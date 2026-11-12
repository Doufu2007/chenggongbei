# v31 有限高度闭环独立审查

结论：当前主协议软件审查通过；未发现阻断这一次已批准的初始静态柱内有限试验的问题。此次没有附加前向点云 collector，不能宣称已采到前向云，也不允许据此扩大前飞或穿窗控制合同。

冻结目标：

- `src/inspection_mission/inspection_mission/bounded_height_hold.py` SHA256 `e925786494c3dc51d7de6c8f47779d5064e97e13513c8daba4df5b3db982e391`
- `scripts/run_bounded_height_hold.sh` SHA256 `40477f039c6c765d1f683f68f33ee6dff0b51635ec14c77e08c3942d3185a6d7`

独立探针 `runs/review/height_hold_independent_20261112/independent_probe.py` 仅写审查产物，调用生产 `create_height_node`，通过真实注册的 callbacks、`tick` 和替换后的 `_publish` 接线；ROS transport 为合成夹具。未改生产源码，未上传平台。共 8 组、18 个定点案例通过，没有重复作者的 198 关联测试。

| 核验 | 独立结果 |
| --- | --- |
| 实际 factory 全协议 | 自选合成局部响应模型，13.35s 后 `FINISHED / support_return_verified_height_protocol_complete`，269 条 UAV 请求与 guard 记录逐条相等；三个非零 phase 均有成功请求和推进 odom |
| 固定反馈公式 | 实际发出的 HOLD / DESCEND 请求逐条按原始目标、测高独立重算，与固定双层限幅式一致；地面仅全零、UAV 仅正 Z 或零 |
| HOLD 连续驻留 | z 越界和 vz 越界两种帧均在 tick 前被好帧覆盖，坏帧仍立即清除驻留 anchor；从 3.05s 重新计时，没有借用旧 1.2s anchor |
| 首次进入 DESCEND 已近支撑 | 实升 .024m、vz=-.01 与 -.021 两例均没有首个非零；前者 `descent_verified=true`，后者 false，均进入 ZERO_RETURN |
| 永久预算 | HOLD、PHASE_GATE 和 ZERO_RETURN 三种状态在原 motion 起点+25s 锁停，不重置起点或 origin |
| 发布后边界 | ground 慢返回、DDS 慢返回、DDS 异常、ground 异常、匹配丢失、反馈值改变、motion25s 到期共七例，均独立尝试 UAV 零、没有继续发送原非零、没有增加成功正请求计数 |
| HOLD8s 与 watchdog | ground 返回跨越 HOLD8s 时中止；无新 tick 时 watchdog 永久转 ZERO_RETURN |
| UAV 发布失败 | 失败正请求不计为成功，仍尝试并记录 UAV 零，锁停原因保留 |
| 证据合同 | 全协议 gzip 分片恢复的压缩/解压长度、双 SHA 和 evidence 对象一致；官方分数为空，`full_mission_complete=false` |

`descent_verified=true` 的首低案例符合当前规格的高度/速度联合终止定义，不等价于“已发布下降请求”。该例的下降非零计数明确为零；正式分析应保留这一差别。

合成模型采用 `vz=2*(u-.02695)`、全零 `vz=-.0539` 和支撑面截断，仅用于走通软件协议与接线。13.35s、3s 驻留和返回均为合成数据，不是官方 ROS/Gazebo 实测、悬停能力或任务得分证据。公式里的 trim 仍是单局局部插值候选。

调用阻塞期间不能保证硬实时归零；本次审查证明的是在 ground/DDS 已返回、代码仍有机会重检时，后续原非零被阻断。成功 publisher 返回也不是执行器应用时间证明。已有静态地图、初始支撑和近水平局部柱假设保持原合同，未知全空间没有新增可观测性。

可复跑：`python runs/review/height_hold_independent_20261112/independent_probe.py`。数值与逐例原因在同目录 `metrics.json`，文件字节哈希在 `manifest.json`。
