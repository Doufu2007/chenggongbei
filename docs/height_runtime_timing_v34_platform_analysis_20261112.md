# v34 官方有限高度时序观测：独立分析

本次原 v31 有限高度合同得到日志支持：升高约 0.10m、真实保持、受控下降和原墙钟 1 秒回支撑条件均成立，终态为 `FINISHED / support_return_verified_height_protocol_complete`。这是固定初始静态柱内的有限实验结果。官网仍显示 **0 分、测评未通过、未检测到完整结束标志**；不能将实验完成称为巡检任务完成或比赛得分。

此次时序证据将两种现象分开：一次 UAV odom 回调内有 **32.632915ms wall / 0.268046ms thread CPU**；全程最大 odom/scan 样本记录间隔则为 **170.159931ms / 175.402672ms**，远大于任一单次已计时 callback/spin 的最大跨度。后者不能直接解释为单个回调处理耗时，前者也不能据此归因 DDS、主机调度或锁等待。

## 运行与完整性

- 官网 ID：`0d99c6fc-e6b7-4a69-aa35-abad82dbcbc0`；runId：`run_20261112_085752_4f79c7da`；测评编号：`#22362608`。
- 原始证据：`runs/evidence/20261112/callback_height_timing_v34.jsonl`，12 个事件，`launcher_exit=0`；`probe_finished.errors=[]`。
- 原始 JSONL：299,790 bytes，SHA256 `8ed9fd939c35bc80ebf7462e1b45a6d7730c7c725c19922346652326936ec3a3`。
- 7 个 gzip+base64 分片：索引恰为 0–6、无缺片/重复、全部共享相同 session/编码/双长度/双 SHA 元数据。
- 压缩后：218,381 bytes，SHA256 `e5645a117e0d8ead8081f899b9b54ab161519a914fa5e15ef37ee79e35d2a606`。
- 解压后：1,861,180 bytes，SHA256 `10ec4cc4df9932f2e20155227ceeb077cd3b75a1bf29ca9f68d0ac5af7401752`。
- Session：`2f0caa808b9a4cdda6455a3c9fa360ba`。唯一 core 与完整恢复载荷的全部共有字段逐项相同。
- 顶层仍是 `schema=bounded_height_hold_v1`、`mode=bounded_height_hold`。v34 以完整载荷内的 `runtime_timing.revision=v34_timing` 标记；core 本来不含 runtime_timing，不能据此误判缺失。`official_score=null`、`full_mission_complete=false` 均保留。

本地诊断模块为 `cb2c0b4dadb30b8b2ff1700e19e669ff7759bbf69dbaf58a71058cd5f8dee6be`，启动器为 `29201e1323e2cb9e1d6b7b9763701617c11a5c6bb0f4b8368bb6ce6e15e010d1`。独审清单中 15 个文件此次实核全部匹配，其中 5 个冻结 v31/父层/telemetry 文件保持相同字节。提交 ZIP 内诊断模块也与已审本地模块相同；ZIP SHA256 为 `b55882ce5627270292685a854b716455737a1fc5d3efa5d7de0103869af49924`。这些核验建立本地文件与回调载荷的字节完整性，**不构成回调来源的独立认证**；原始 source 全部仍为 `unverified_callback`。

复现命令：`python runs/analysis/20261112_height_timing_v34/analyze.py`。命令仅读取生产源码/原始证据并写入该分析目录，不加载 ROS、不驱动仿真、不上传。它复用 v31 的逐帧几何审计函数，但独立严格解包 v34，并新增原支撑合同时序核验。

## 几何轨迹、控制与完成条件

原始轨迹 666、命令 279、水平 scan 108、下视 scan 271、TF 消息 239，原始五类样本的 `dropped` 全部为 0。217 个正 z 命令、62 个零命令，没有负命令；地面始终零。正命令的量测引用和固定控制公式逐项复核，错误 0；低高度切断线以下仍非零、hold 8 秒截止后非零、运动 25 秒预算后非零，均为 0。

初始支撑 z=`0.055000708229m`，相对最大升高 `0.100952693822m`，最大水平漂移 `0.000037244329m`，累计路径 `0.205193199317m`（独立重算相同）。运动起点相对 guard 为 `1.114164695s`，首次 FINISHED 命令为 `13.714314358s`；最后清理命令为 `13.737431262s`。源端 odom 从 stamp 2.94 推进至 16.34；无反向或重复不前进。

| 阶段进入 | guard 墙钟 elapsed(s) | 源 odom stamp(s) | 独立核验 |
| --- | ---: | ---: | --- |
| ASCEND_TO_010 | 1.114165 | 3.98 | 起始稳定后开始 |
| HOLD_010 | 3.455395 | 6.22 | 到达后先归零再进入保持 |
| DESCEND_002 | 6.638090 | 9.36 | 保持成立后先归零再进入下降 |
| ZERO_RETURN | 12.190888 | 14.82 | 相对高 0.024968379m，vz=-0.002561699m/s；同时满足 ≤0.025m 与 |vz|≤0.02m/s |

保持窗口独立逐帧核验有 148 个连续合格样本，源跨度 **3.04s**，记录到达跨度 **3.094927913s**，相邻源间隔最大 0.08s、记录到达间隔最大 0.170159931s。窗口最大高度误差 0.001996765m、最大 |vz| 0.000992366m/s，满足 ±0.005m 和 0.01m/s。自报 dwell 的精确端点 6.28–9.30 也存在，147 帧全部合格，源/到达跨度分别 3.02/3.075937459s。未发生 hold reset。

### 回支撑的原合同与额外严格口径

冻结源码 `bounded_vertical_response._stable(wall)` 要求：速度≤0.005m/s、角速度≤0.01rad/s、相对稳定 anchor 的位置距离≤0.002m，且 **墙钟 stable_s=1s**；height 层另要求 |z-origin|≤0.005m。这里没有“源 stamp 跨度也≥1s”的条件。源和墙双 3 秒是 HOLD 的合同，不是支撑合同。

本次首次满足支撑量测条件的 tick 被完整保留，recorder 相对时基入口/退出为 `[12.705052350,12.705256328]s`，使用 odom stamp=15.32。FINISHED tick 为 `[13.714007239,13.714210867]s`，使用 stamp=16.30。anchor 和 `guard.step` 实际墙钟标量未直接记录，但都位于对应 control_tick 内，因此真实 anchor→decision 墙钟间隔可界定为 **[1.008750911,1.009158517]s**。全部中间原始帧满足速度、角速度和高度带，距离 anchor 样本最大 0.000031958840m。该证据支持原墙钟 1 秒回支撑合同完成。

首次 FINISHED 前，支撑原始样本只跨 **0.98s 源 / 0.996394349s 记录到达**。所以沿用 v31 分析脚本的额外“FINISHED 前源和到达都满 1 秒”布尔为 false；这不是冻结 guard 的软件失败，也不能把原合同称为双时钟 1 秒通过。清理阶段的后续原始帧继续合格，整段延长至 1.02s 源 / 1.036680344s 到达。`metrics.json` 保留额外严格布尔及说明，并另给出 `original_guard_local_protocol_supported=true`。

支撑接近度不等同于独立触地传感器。采样间隔内的全部连续物理行为也不能由离散帧证明。

## 时序覆盖与七类入口

共有 5,686 个完成 span。全程各 label 汇总全部保留；详细 trace 只保留最近 2,048，丢弃较早 3,638，保留完成时刻为 recorder `8.722629591–13.752298306s`。慢 span 门槛为 20ms，最大的 32 项保留、丢弃另外 4 项。`instrumentation_errors=0`。不可把嵌套 receive/spin 或 tick/control_tick/DDS 的 wall/CPU 相加。

下表 wall 最大值与 CPU 最大值均为该 label 的全程独立最大值，**不保证来自同一次调用**；CPU 与 wall 计时器独立取样，微秒级 CPU 略大于 wall 不代表负等待。

| label | 完成数 | 最大 wall(ms) | 最大线程 CPU(ms) | 首–末入口(recorder s) |
| --- | ---: | ---: | ---: | --- |
| receive:/clock | 136 | 0.077559 | 0.075277 | 0.002289–13.696365 |
| receive:/uav/odom | 666 | 32.632915 | 0.449087 | 0.071500–13.737084 |
| receive:/odom | 666 | 11.654874 | 11.657092 | 0.070611–13.736487 |
| receive:/scan | 108 | 2.136869 | 2.137033 | 0.024722–13.646720 |
| receive:/uav/laser/scan | 271 | 0.180940 | 0.180678 | 0.003650–13.752203 |
| receive:/tf | 238 | 1.300064 | 1.302683 | 1.958165–13.624870 |
| receive:/tf_static | 1 | 0.324725 | 0.323370 | 0.010704–0.010704 |
| spin_once | 2,350 | 33.125446 | 11.816195 | 0.001671–13.697697 |
| tick | 272 | 7.865701 | 0.602403 | 0.002398–13.714002 |
| control_tick | 272 | 7.849325 | 0.570267 | 0.002401–13.714007 |
| dds:control_tick | 489 | 0.025006 | 0.025233 | 0.002714–13.714017 |
| dds:publish_boundary | 217 | 7.454954 | 0.046703 | 1.114238–12.149957 |

七类回调全程入口/退出汇总均存在。详细尾段中依次有 clock49、UAV odom247、地面 odom247、scan39、下视99、TF80；tf_static 发生较早，尾段中没有，不能从尾段缺失推断未收到。被保留的所有 receive span 均含 topic、source stamp、入口 state/phase 与 exit/wall/CPU 字段；tf_static 的全程消息和静态变换还保留在原 TF 记录。

source stamp 是消息原始 clock/header 标量，TF 只取消息首个 transform；TF 流混合不同变换，尾段 source gap 从 -0.02 到 0.32s，并不表示单一传感器时间倒流。它不是源端 wall 发布时间，不可将“callback入口wall-source stamp”作为网络延迟。guard 与 recorder 的起点也不同；以 249 个精确源 stamp 联结回调和轨迹，得到 recorder.started_wall−guard.started_wall 的保守边界 `[0.000005642,0.000125393]s`，分析没有将两种 elapsed 直接混同。

## 回调内耗时与到达间歇

最大 spin 为 recorder `[3.064616596,3.097742042]s`，wall=33.125446ms、CPU=0.759604ms。它完整包含 UAV odom `[3.065026653,3.097659568]s`，source=5.86，wall=32.632915ms、CPU=0.268046ms，状态 ASCEND/ASCEND_TO_010。这个特定样本的 spin 墙钟耗时主要位于 receive 包装内，两个嵌套值不能相加；该回调 wall−CPU≈32.364869ms，但没有进一步测量其锁等待或被调度离开等分项。

同一 odom 原始记录与前一帧间隔为 52.988451ms（source 只进 0.02s），后一帧间隔 12.952607ms。原始 trajectory 的 wall 捕获发生在 receive 实现内，可能已晚于包装入口；因此该 52.99ms 不能直接称为 callback 入口间隔。这个慢回调位于被覆盖的早段，slow 列表只保留它和包含它的 spin，实际相邻所有 callback 已不可完整恢复，报告不虚构邻接序列。

| 全程原始样本流 | 最大记录到达间隔(ms) | 对应 source 间隔(ms) | 该流回调最大 wall(ms) |
| --- | ---: | ---: | ---: |
| UAV odom | 170.159931 | 60 | 32.632915 |
| 水平 scan | 175.402672 | 150 | 2.136869 |
| 下视 scan | 83.580091 | 50 | 0.180940 |

最大 odom 记录间隔是 guard `5.359636273→5.529796204s`，source `8.12→8.18`；全程 odom source gap 最大另为 0.08s。该间隔中仍保留了多次较慢 spin 的片段，例如 recorder `5.375227204→5.395423031s` 与 `5.425728855→5.445925353s`，没有单个 170ms callback/spin。最大 scan 间隔为 guard `2.951331980→3.126734652s`，source `5.75→5.90`；它包含上述 32.63ms odom 回调，但 scan 自己的最大处理墙钟仅 2.14ms，且源样本本身相隔 0.15s。两种证据均要求将源采样/到达间歇与主 callback 执行区间区分；它们不能独立确定上游发布、DDS队列、取消息或主机调度各占多少。

完整保留尾段允许直接比较入口与退出后的静默：scan 最大入口间隔156.959290ms，其中前一个 scan 执行后至下个入口155.935502ms，source间隔150ms；UAV odom最大入口间隔34.750581ms，前回调退出至下入口34.544058ms，source进20ms。尾段扫描间隔远大于其回调本身耗时。逐对入口/退出/source数据见 `retained_callback_gaps.csv`；完整原始样本间隔见 `full_sample_gaps.csv`。

## spin、tick 与发布边界 DDS 查询

spin_once 全程 wall 总量13.578276927s、线程CPU总量1.372332249s。原 spin 调用包含最多20ms的timeout，低CPU墙钟跨度本身不证明 DDS 故障。272次 tick/control_tick数一致，外层tick包含control_tick及其观测开销；control_tick里可能多次调用订阅匹配查询，所以489个 `dds:control_tick` 不是489次控制决策。217个 `dds:publish_boundary` 对应原 `_publish` 路径的最后匹配查询位置，也不是“全运行只查询一次”。

全程 publish_boundary 查询最大 wall=**7.454954ms**，该次详细记录已被旧 trace 覆盖，因为它小于slow门槛20ms；不能给出它的准确入口、相邻 callback 或准确 CPU配对。其 CPU 必不超过全程该label的46.703µs，因此wall−CPU至少7.408251ms，仍不支持具体原因归因。全程control_tick/tick最大7.849325/7.865701ms的详细早段也不在保留trace内，不能强行宣称这几个最大值来自同一tick。

保留尾段中最大的publish_boundary查询发生于11.594419002→11.594436022s，为17.020µs wall /4.086µs CPU；它的外层control_tick为259.145µs wall，tick为271.568µs wall。前一receive:/odom退出到查询入口11.253221ms，查询退出到下一receive:/odom入口8.074639ms，两个odom源stamp14.22→14.24。前后间隔远大于查询自身17µs。

本次最后一个publish_boundary查询发生于12.149957164→12.149961176s，为4.012µs wall /3.893µs CPU；含它的control_tick=446.719µs、tick=462.568µs。前一UAV odom退出至查询入口18.674075ms，查询退出至下一地面odom入口0.606310ms。之后进入ZERO_RETURN，零发布/清理路径按原设计没有该类最终非零查询。此处的“最后一次”和“发布边界最后查询位置”已分别列出，避免同名混淆。

该次没有重现上一轮超龄终止，`protocol_abort_reason` 为空、`post_stop_faults=[]`、发布错误为空；它不能证明旧超龄已修复或永不复现。保留的详细尾段仅约5秒，不能把旧跨度邻近情况补全，更不能倒推出DDS take/反序列化、操作系统调度/配额或guard锁等待原因。

## 官方结果与分析产物

已有官方页快照 `height_timing_v34_official_page_final.json` 显示测评未通过、0分、已用56.1s，原因是“流程未检测到完整结束标志（未提取到成绩）”。裁判日志还记录非零 `/uav/cmd_vel` 取消后续陆地巡航计分。诊断代码没有发布巡检计分/完成标志，该官网结果与有限高度实验的本地状态属于不同层次事实。

目录 `runs/analysis/20261112_height_timing_v34/` 保存严格解包器、验证清单、恢复gzip/JSON、去分片事件、逐帧几何指标、时序指标和7份CSV。生产控制文件未修改，未上传或操作浏览器。下一次较高高度试验必须使用独立、预先声明的目标和预算，不能由本次结果自动放宽冻结v31高度/路径/时间合同。
