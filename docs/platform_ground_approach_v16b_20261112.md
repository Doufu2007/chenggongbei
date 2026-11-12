# v16 / v16b 平台有限接近：真实证据复核

日期：2026-11-12。仅整理已有原始回传、首次分析及平台终态；未改 raw、已有分析目录、控制源码、启动脚本或旧上传包。当前仍没有完整官方任务的有效成绩。

## 平台终态与上传边界

最新两条记录以 `runs/evidence/20261112/approach_v16b_list_settled.json` 为准：

| 版本 | 记录 ID | runId | 状态 | score | durationMs | 编号 |
| --- | --- | --- | --- | --- | --- | --- |
| v16 | e8d077bd-89ca-4b0e-8ff8-dfefbd7e10d6 | run_20261112_031225_e3fbe459 | failed | null | 159174 | #14154215 |
| v16b | b4cd8cda-f442-41dc-8dd6-2a2a4c5c5484 | run_20261112_032806_33175195 | failed | null | 167614 | #97659318 |

API 的 `failedReason` 原值为“流程未检测到完整结束标志（未提取到成绩）”。已按 UTF-8/Unicode 核对，台账直接取源字段；终端编码导致的显示乱码没有被手工写回原始记录。网页显示 0 分不能替代 API 的 null。较早 v16 的 running 快照已过期。

v16 的回传隧道继承固定 HTTP404 ingress；`approach_v16b_artifacts.json` 记录了隔离通道重试原因。这解释 v16 没有诊断 raw，不能据此判断 ROS 动作未运行，或把通道问题当作未完成官方任务的唯一原因。v16 的抽签、运动、同步效果均未知。

v16b 首次请求 `approach_v16b_submit.json` 返回 HTTP409/EVAL_IN_FLIGHT_LIMIT，不登记为新局；真正接收凭证是 `approach_v16b_submit_retry.json`。receiver health 文件中的 HTTP405/accepted=false 只反映该请求方法，收齐程度按实际回传核实。文档不复制临时通道地址、密钥、账号或签名下载链接。

两份源码 ZIP 均为 97,185B、SHA256 `38c337af70dd5185c3dcebdd3e0a3534a06ee1addd91987dc5c2c60c3ceecc47`。脚本不同：

- v16：`dist/run_ground_approach_v16.sh`，2,883B，SHA256 `3c7de09e59a9cc01749a62e0fee85ef4f34c317c2c5795393c772f97eae6ee3c`。
- v16b：`dist/run_ground_approach_v16b.sh`，2,872B，SHA256 `67d45c70c1e8fa57bfccc6dfeaa36e7110274e7225e624501892dc6acaae0131`。

## 诊断完整性与 partial 的实际含义

v16b raw 为 `runs/evidence/20261112/callback_approach_v16b.jsonl`，525,717B，SHA256 `3668579cdf8cbe7bdee072a7c9a87b4d127b890eb1b48cadd436af63c8515f10`。已有分析为 `runs/analysis/20261112_approach_v16b/summary.json`，本次未覆盖重解析。

- 共 62 事件：launcher_started 1、probe_started 1、probe_snapshot 55、image_sample 4、launcher_exit 1；退出码 0。
- core_count=1、full_count=0。现有 collected 合同要求完整 full snapshot，所以为 **partial**；不是解析异常，也不是任务成功。
- trajectory 为 665/665 行、actual_commands 为 2516/2516 行，series 完整，源 trajectory/commands 掉行均 0；六组 native 完整。
- parse/assembly/diagnostic error、conflict、missing lifecycle/series/metadata/snapshot fields 均为空。14 段冗余 `survey` 作为 unknown_sections 保留，没有静默丢弃或宣称已认识。
- `motion_log_dropped=384`，因此 compact series 完整不等于逐回调日志完整。保留日志只能证明实际保留的样本，不能推断未保留样本的状态分布。
- capture_count=0；没有 CAPTURE 三图合同被触发，不凭空要求三张照片。first/last/last_associated 六图属于双相机观察，不是正式到点识别证据。

## 实际动作：未走到 0.20m 观察距离

当局抽签 `ground_1,ground_0,ground_3`；候选映射标记 `candidate_unverified`，目标候选坐标 `(0,-1.35)`。target_yaw=-1.5708262686424004rad，drive_yaw=-1.5470751843603814rad。drive_origin=(-0.004663389023588876,-0.005694901313399071,0.05500012475164811)。

末状态 FINISHED，原因 `approach_drive_wall_time_limit`；前向投影 0.09577410397334182m、横偏 -0.001743778781805655m、累计路程 0.1095597118990008m、最大起点位移 0.10157727301464067m、累计转角 1.5489991288055613rad、resume_count=1、capture_count=0。源码仅做有限地面观察，没有发识别、到点、飞行、照射或比赛完成消息；所以不能期待本局产生完整任务结算。

按实际命令首次进入阶段记录，时间为节点 wall elapsed；平台 167.614 秒和节点末快照 87.712 秒是不同口径：

| 阶段 | wall elapsed 秒 | /clock 秒 |
| --- | ---: | ---: |
| WAIT | 0.057610 | 3.2 |
| TURN | 1.116505 | 4.2 |
| TURN_SETTLE | 29.607456 | 32.2 |
| DRIVE 第一次 | 30.654060 | 33.2 |
| PAUSE 第一次 | 35.061629 | 37.6 |
| DRIVE 恢复 | 36.158344 | 38.6 |
| PAUSE 第二次 | 50.921054 | 53.2 |
| STOP 首零速 | 85.665247 | 87.3 |
| FINISHED | 87.667975 | 89.2 |

命令区间累计 TURN=28.490951s、TURN_SETTLE=1.046604s、DRIVE=19.170279s、PAUSE=35.840908s；其中第二次 PAUSE 持续34.744193s，STOP=2.002728s。55秒 DRIVE 墙钟预算包含 PAUSE，不能称机器人连续行驶55秒。

独立有效运动区间估计只取相邻轨迹 wall dt>0且≤.35s、sim stamp递增、起样本前最近命令 age≤.1s且整个区间保持 DRIVE/.03/0；不使用未来命令，不跨 PAUSE。共180区间：投影增量 .0933187485959488m，wall18.756352s，平均 .004975314421266396m/s；sim18.44s，平均 .005060669663554708m/s。此结果不证明线性增益，也不能外推 .10 请求的实际响应。

## 同步修补已真实执行，持续 freshness 仍未打通

sensor_sync：incoming673、immediate398、late_prior123、deferred_existing0、expired_pending149、capacity_dropped0、superseded1、pending0。promotion_trace 丢509、末保留12条。所保留 late-prior 例子中，真实 prior odom 比 scan 到达晚约 .116–.168s，scan-minus-odom约 .025–.046s，原始 header/arrival 保留，年龄仍受 .2s 限制。

这证明“scan先到、真实前序odom后到再配对”在平台实际发生；不证明持续 fresh 或最终成功恢复。保留 motion_log 有25条 scan_stale_or_unsafe、24条 odom_stale_or_invalid、13条 pause_stability_dwell、2条预算结束记录；不能把这些计数当作全程回调统计，也不能以 v15/v16b 跨局差异作因果改善结论。

以 STOP 前后实际日志为例：wall85.6425s 仍为 PAUSE，clock87.3、odom87.08，sim age=.22s；已配 scan86.665，wall age=.663291516s、sim age=.635s。wall85.6929s 的 STOP 日志仍是同一 scan，wall age=.713711792s。更早 wall85.3894s，odom 已fresh但 scan仍stale，也说明不只是单个odom门控。

末快照 scan wall_age=2.733031695s 包含 STOP 后约2秒，因为 STOP 阶段不再更新控制器 scan/pair；不能把这个最终年龄称作活动期2.7秒传输延迟。freshness 阻塞的依据是上述活动期 PAUSE 样本。

graph_refresh_diagnostics：interval=.5s、refresh174、graph_query1044、total=.05893820402889105s、max=.000244772001678939s、match_check1735。它只测这些graph query调用，不覆盖全部ROS调度、反序列化、锁竞争或DDS传输，尚无证据证明图查询是持续PAUSE主因。

`local_guard_costs.json` 明确是本地合成纯 guard 测量，不是平台证据；没有测ROS deserialize/take、message_to_ordereddict、RMW/DDS、平台CPU配额、真实executor或跨线程锁等待。不能以本地低开销排除这些未测因素。下一次性能诊断应原样保留回调入口、分段wall/CPU、锁等待及接收间隔，先分辨瓶颈，再决定修补，不放宽现有安全阈值来隐藏过期。

## 场景资料与下一步边界

stage scan 只有 start/after_turn，分别360束，min约 .6500024/.7356800m，prior skew .011/.016s；没有 mid_drive/after_drive，不能用开始或转向后的扫描充当终点净空。

六张 native：UAV/ground 的 last_associated 和 first 有有效 prior；两张 last 的关联无效。未进行类别确认、目标号映射或正式识别上报，不能写成七类图库已补全。

平台辅助模块实操另见 `docs/platform_tools_availability_20261112.md`：仿真中心加载赛事场景、AI开发助手发送问题、算法解析运行 Pure Pursuit 均提示“测试阶段·暂未开放”；预计11/12仅是页面文案。这阻塞相应辅助入口，不证明测评失败只因这些模块未开放。

独立路线控制器已冻结并完成只读复核，见 `docs/control_review_ground_route_20261112.md`。它尚没有 .10 请求真实响应、停车包络或长路线时窗证据；本报告不把本地三图合成闭环当作真实路线成功。下一局优先以同一 .20m/.03mps/90s 运动边界取得性能诊断，获得连续 fresh/运动与停车证据后再决定扩大路线。
