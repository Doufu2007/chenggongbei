# v17 平台计时与有限接近：真实证据分析

日期：2026-11-12。只读分析已有 raw、首次重组与API快照；未改控制源码、启动脚本、旧上传包、raw、已有分析目录或续接卡。以下结论限于本次有限地面观察，不是完整巡检任务成绩。

## 平台状态与证据范围

记录ID `441364ff-c6c4-4b1d-9683-1b249b505094`，runId `run_20261112_034630_6bea29f7`，编号`#59088134`。最新权威快照 `runs/evidence/20261112/approach_v17_list_final.json` 的更新时间为`2026-11-12 03:55:16.108911+08`，终态 **failed / score=null / durationMs=164794（164.794秒）**；failedReason原值“流程未检测到完整结束标志（未提取到成绩）”。此前after_callback的running/0及更早list_next的queued/0已过期，台账按最新终态更新，不因诊断退出码0填造成功或有效0分。

raw `runs/evidence/20261112/callback_approach_v17.jsonl`，547,426B，SHA256 `9cc1380ef74ce12020684dcf1993c09329303709d8000d4182f1bf8e50619e99`；既有分析 `runs/analysis/20261112_approach_v17/summary.json`，本次没有覆盖重解析。

62事件：launcher_started1、probe_started1、probe_snapshot55、image_sample4、launcher_exit1，退出码0。core1/full0，因此诊断仍partial；666/666轨迹、2643/2643实际命令、六native完整，trajectory/commands源掉行0。parse/assembly/diagnostic error、conflict及missing lifecycle/series/metadata/snapshot fields均为空；14段冗余survey保留unknown。逐回调motion_log掉430，保留计数不能代替全程状态分布。

`callback_timing.revision=v17_timing`已实际回传，Linux clock_gettime的perf_counter/current-thread CPU计时分辨率均标记1ns；这表示时钟API报告的分辨率，不等于测量准确度。控制器仍`v16_sensor_sync_graph_throttle`，运动/安全合同未改。

上传件：

- `dist/inspection_ground_callback_v17_src.zip`：105,838B，SHA256 `2b08c03d4b94a84da38acfe3f994fc9a9e9d159ac5a7f96b07f22409449c94e1`。
- `dist/run_ground_callback_v17.sh`：2,854B，SHA256 `7b8838728b33b27be048f30a52302edd73543e7edafaa7883d421302190d8bfc`。

## 计时：本队回调处理未呈现百毫秒阻塞

真实callback_timing总量如下。receive从Python订阅回调入口开始，消息此前已反序列化；CPU仅当前线程。spin_once包含wait/take/deserialize/callback，其中包含receive，不可将两者总耗时相加。

| 区段 | 次数 | wall均值毫秒 | wall最大毫秒 | CPU均值毫秒 | CPU最大毫秒 |
| --- | ---: | ---: | ---: | ---: | ---: |
| receive /clock | 874 | 0.09681 | 0.30309 | 0.09605 | 0.29975 |
| receive /odom | 2905 | 0.38311 | 12.61251 | 0.38263 | 12.58374 |
| receive /uav/odom | 3076 | 0.14952 | 0.64245 | 0.14922 | 0.64201 |
| receive /scan | 699 | 1.02377 | 2.04618 | 1.02376 | 2.04850 |
| receive /tf | 8662 | 0.04426 | 0.18012 | 0.04388 | 0.17902 |
| receive 地面图像 | 262 | 0.14077 | 0.44296 | 0.13895 | 0.44039 |
| receive UAV图像 | 261 | 0.14649 | 0.42681 | 0.14474 | 0.42498 |
| tick | 1761 | 0.21452 | 1.17217 | 0.21470 | 1.17363 |
| spin_once | 18938 | 4.65740 | 32.34966 | 0.60475 | 13.31305 |

/odom有2901/2905条落在wall≤1ms桶；其余3条≤5ms、1条≤20ms。/scan452条≤1ms、247条≤5ms，最大约2.05ms。所有receive的wall总2.88924s、CPU总2.88171s；tick总wall .37776s；spin总wall88.20193s、CPU11.45270s。后者包含大量正常等待及receive，不能将wall-CPU全部解释成锁等待，也不能把与receive的差额全部归给反序列化。

12个保留慢事件全部来自spin_once，最大32.35ms；另380慢事件未保留详情。最大span未达到.1s，未见本队receive或单次spin百毫秒持续阻塞证据。锁等待未独立测；平台CPU配额、RMW/DDS与公开桥的延迟未被分别测量，不能据这些较小处理时间断言任一环节无问题。

graph query共1062次、总 .058909999s、单次最大 .000240337s，refresh177次；此口径仍只涵盖graph query调用。

## 回调入口不普遍过期，缓存fresh窗口仍反复中断

入口年龄定义为“回调入口处最近已接收clock减原始header”，并非绝对网络传输延迟。负数意味着该header领先最近clock样本，不能理解成消息来自未来。

| 输入 | clock-header最小/最大秒 | 入口wall间隔均值/最大秒 | header间隔均值/最大秒 |
| --- | --- | --- | --- |
| /odom | -.18 / .18 | .030583 / .197267 | .030034 / .18 |
| /uav/odom | -.18 / .18 | .028898 / .223591 | .028384 / .18 |
| /scan | -.102 / 0 | .127068 / .157623 | .125009 / .152 |
| /clock | 无header年龄 | .101722 / .202309 | 无header统计 |

699条scan入口clock-header全部≤0；2905条odom入口没有>.2s，401条在(.1,.2]桶，另4条在(-.2,-.12]桶。因此，不能把本局暂停概括成“scan/odom一进本队回调就普遍过期”。odom相邻header有203/2904个间隔落(.1,.2]秒，入口wall间隔有205/2904个落(.1,.2]秒；最近2048间隔中也保留“.18/.02/.18/.02...”的实际序列。后续clock继续推进，会使缓存pose变旧；即便单个到达样本fresh，也不保证整个恢复驻留期间连续fresh。

同步诊断：incoming683、immediate369、late_prior144、expired169、capacity_dropped0、superseded0、pending0；promotion_trace丢501、末保留12。末trace的late prior等待约 .09597–.17562s，仍保留原scan到达时间，配对时剩余fresh预算很短。它证明真实先后顺序和配对发生，不证明这些间隙由某个具体源头产生。

保留motion_log：odom_stale_or_invalid24、scan_stale_or_unsafe24、pause_stability_dwell14、budget结束2。例：wall86.4197s，odom/scan均fresh，仍等待pause_stability_dwell；wall86.5208s，odom fresh但scan wallage=.244670809s；wall86.5713s，odom simage=.24s且scan stale；wall86.6723s两者恢复fresh；wall86.7733s再次odom simage=.24s/scan wallage=.243304216s。驻留锚点在实际freshness故障时清空，因此没有形成足够的连续一秒驻留。

STOP边界wall86.8745s，两者其实fresh，但55s DRIVE墙钟预算已经耗尽。末快照scan age2.23878s包含STOP后约2秒控制器scan冻结，不能称活动期2.24秒传输延迟。

UAV camera_info另有51/244条入口年龄>.2s（最大2.984s）、入口gap最大3.730s；原生UAV图像只有3条>.2s，地面图像最大.017s。camera_info落后可能需要后续单独定位，但当前地面guard的odom/scan门控未使用该camera_info，不能把它直接当作本次DRIVE停止原因。

## QoS实际配置

创建订阅时，/odom、/uav/odom、/scan和双相机/相机信息均为depth1、RELIABLE、VOLATILE；已观察publisher也是RELIABLE/VOLATILE。/clock为depth1、BEST_EFFORT/VOLATILE；TF为depth10，/tf_static与/referee/ground_tasks使用TRANSIENT_LOCAL。此记录为实际请求与创建时查询，不是动态publisher全历史，也不能直接证明可靠传输导致延迟或depth1造成样本缺失。

## 运动与v16b的描述性比较

当局抽签`ground_1,ground_0,ground_4`，与v16b同样首点ground_1，但非配对复现实验。候选坐标(0,-1.35)、target_yaw=-1.570824300171313、drive_yaw=-1.5470134289515747，mapping仍未验证。末投影 .07684314765955447m、横偏 -.0013139131685821403m、累计路程 .09061670087167867m、最大起点位移 .08321750915167712m、累计转角1.5493288691673905rad；resume1、capture0，原因approach_drive_wall_time_limit。

| 真实记录 | v16b | v17 |
| --- | ---: | ---: |
| TURN请求累计秒 | 28.490951 | 28.510710 |
| TURN_SETTLE秒 | 1.046604 | 1.089638 |
| DRIVE请求累计秒 | 19.170279 | 15.239003 |
| PAUSE累计秒 | 35.840908 | 40.866028 |
| 最后连续PAUSE秒 | 34.744193 | 39.785043 |
| 最终前向投影米 | .095774104 | .076843148 |
| late_prior配对计数 | 123 | 144 |
| expired_pending | 149 | 169 |

v17首PAUSE在TURN中：wall26.028279→27.109264秒；恢复后30.760767进入TURN_SETTLE、31.850405进入DRIVE、47.089408再次PAUSE、86.874451首STOP零速、88.891491 FINISHED。55秒DRIVE预算包含暂停，未达到.20m，没有CAPTURE。

按与v16b相同的有效运动筛选：相邻wall dt>0且≤.35s、sim stamp递增、首样本前最近命令age≤.1s、整个区间均DRIVE/.03/0，不跨暂停、不用未来命令。v17共148区间，投影增量 .0754469276795407m / wall15.043171s = .005015360636367205m/s；sim14.82s口径=.005090885808336079m/s。v16b相同方法约 .004975314m/s。这个接近只描述同档有效运动，不证明计时补丁有益/有害，也不能线性外推.10请求。

两局抽签后段、输入调度和资源环境不同，未构成同seed配对；更多暂停和更短投影不能归因于instrumentation开销。退出码0与FINISHED只表示有限诊断正常收尾；六图为first/last/last_associated，last两图prior关联无效，且stage scan仅start/after_turn，无正式识别或得分证据。

## 下一项单变量实验建议

下一版已选择只将 **地面/odom订阅RELIABLE改为BEST_EFFORT**，保持depth1/VOLATILE、其他输入、所有运动/.20m/.03mps/90s、freshness/skew/驻留/停车规则与计时不变。它检验reliability配置对本次输入节律的影响，不是已经确认的根因修复。

比较同口径/odom header与入口gap桶、入口年龄、late_prior/expired、连续fresh驻留能否完成、DRIVE占用与真实投影、原始CAPTURE情况；保留未改/uav/odom作同局参考，但不把两topic当作严格随机对照。一局变化不能排除环境调度波动；若仍有相同header大间隙，也不能单凭一局锁定上游公开桥。depth1→10是可另行检验突发序列截断的假设，此次不与reliability混合。任何结果都不放行跳过原有安全门控或扩大动作。
