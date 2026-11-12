# v18 有限地面观察：真实动作与独立证据复核

日期：2026-11-12。本次只读已有 raw、首次重组、API 快照与固定公开候选 world；未改控制源码、启动脚本、上传包、raw、已有分析目录或续接卡。新算术与逐束几何输出位于 `runs/review/20261112_approach_v18/independent_metrics.json`，可用同目录 `review_v18.py` 复现。

## 平台终态与结论

**有限观察动作本局完成，官方完整任务仍未取得成绩。** 记录 ID `5038ac33-4026-490a-8c1e-5b682b2a2f06`，runId `run_20261112_035930_d1431193`，编号 `#88131099`。最新权威快照 `runs/evidence/20261112/approach_v18_list_final_check.json` 更新时间 `2026-11-12 04:03:43.843859+08`：**failed / score=null / durationMs=141089（141.089 秒）**，failedReason 原值“流程未检测到完整结束标志（未提取到成绩）”。较早 after_callback 的 running/0 已过期，不能登记为已完成或有效 0 分。

真实回传支持：地面车转向后连续前进约 .20m、没有 PAUSE、完成三次原生采图、持续发布零速后退出码 0。它不支持地面目标识别成功、裁判到点认可、飞行、穿窗、照射、降落或完整巡检结算。`FINISHED / approach_observation_complete` 是本队有限观察状态，不是裁判结束标志；没有为取得平台 success 伪造识别或完成消息。

## 来源、完整性与冻结项

| 文件 | 字节数 | SHA256 |
| --- | ---: | --- |
| `runs/evidence/20261112/callback_approach_v18.jsonl` | 578279 | `6a16572c7b368fe9bbe8f90b30810537dc98db51039a8542be48c0a83159ef45` |
| `dist/inspection_ground_odom_qos_v18_src.zip` | 107155 | `f2f387765eb6f89b46a3558919316a3c97f40c65c15245d6a603bf85663232d3` |
| `dist/run_ground_odom_qos_v18.sh` | 2823 | `fe2643346dae8f0e155625520f1f696df7aa2937812b3a13d96bfa727657851f` |
| 候选 `inspection_task1.world` | 27189 | `595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1` |

既有分析 `runs/analysis/20261112_approach_v18/summary.json` 保持原样。55 个事件：launcher_started 1、probe_started 1、probe_snapshot 48、image_sample 4、launcher_exit 1；退出码 0。core 1/full 0，因此解析状态 partial，表示没有单块 full snapshot，不等于解析错误或完整任务失败分类。parse/assembly/diagnostic errors、conflicts、missing lifecycle/series/metadata/snapshot fields 均为空；14 个冗余 survey 分段保留 unknown，没有删除原数据。

trajectory 653/653、actual_commands 1455/1455，两序列完整，源掉行均 0；motion_log_dropped=0（10 条状态/理由变化记录，并非每个回调的姿态全量）。7 张 native 图均重组完整且关联有效，CAPTURE 所需三图齐全。core 发布计数为非零 1225/零 229，命令序列为非零 1225/零 230；1454 条命令在 core elapsed=68.034s 之前，额外一条是 wall=68.203879s 的 FINISHED 零速。报告快照与随后取得命令分段的边界不同，不把多一条零速当成缺包或非零运动。

v18 startup revision=`v18_odom_best_effort`，控制器仍 `v16_sensor_sync_graph_throttle`。本轮仅把 `/odom` 请求 reliability 从 RELIABLE 改为 BEST_EFFORT，depth=1/VOLATILE、其余订阅 QoS、运动与安全合同不变；创建时查到的 `/odom` publisher 仍 RELIABLE/VOLATILE。冻结源码见 `approach_v18_artifacts.json`：`ground_odom_qos.py` SHA `8569c30561a90fb4c43f8ea8578e81d6443e2c3c523b6a2a20d9182a74a5794f`，adapter SHA `5f21933d9bc4f2983d14ba41f844036958ff9754f128ec63f9a5f46785d1cc99`，脚本源 SHA `17fff8969d70b6f15a5c2fcf90042c31b868b9f265ff0ceec09e829e56b9948e`。未放宽 freshness、scan/pose 配对、停车或预算以换取动作完成。

## 连续运动、停止与真实位移

抽签 `ground_0,ground_1,ground_4`，首点 candidate `ground_0` 对应候选坐标 `(1.125,-1.35)`，mapping=`candidate_unverified`。目标 yaw=-.8760771134398732rad，DRIVE anchor yaw=-.852160739132431rad。

下面使用完整实际命令序列首个状态边界，精度高于四位小数 motion_log；区间终点为下一状态首命令，不含最终状态的无后继尾部。

| 阶段 | 首命令 wall(s) | 到下一状态 wall(s) | 命令 |
| --- | ---: | ---: | --- |
| WAIT | .060534 | 1.162688 | 零速 |
| TURN | 1.223222 | 20.510049 | v=0，omega=-.5 到 -.35rad/s |
| TURN_SETTLE | 21.733271 | 1.109256 | 零速 |
| DRIVE | 22.842527 | 41.294896 | v=.03m/s，omega=0 |
| DRIVE_SETTLE | 64.137423 | 1.016837 | 零速 |
| CAPTURE | 65.154260 | .844554 | 零速 |
| STOP | 65.998814 | 2.027162 | 零速 |
| FINISHED | 68.025976 | — | 至 68.203879s 仍为零速 |

无 PAUSE，resume_count=0。controller 在完整 odom 更新口径报告前向投影 .2001512261153392m、横偏 -.00434374440176305m、累计路程 .20887782498340296m、最大起点位移 .20276676318040412m、累计转角 .9228175076333133rad。均在原 .20m 目标、.30m 路程/位移、1.8rad 转角、.02m 横偏与 .05rad heading 上限内；.20m 为停止目标，非绝对不超出的离散采样硬上限。

独立 10Hz 序列算得最大起点位移 .202766793m、DRIVE/驻留采样最大横偏 .004345660m、相对 drive_yaw 最大 heading .010317861rad。采样累计路程 .207372140m 小于全 odom 更新累计路程，两种采样口径不互换。相邻真实 trajectory 行筛选 396 个有效运动区间：0<wall dt<=.35s、sim stamp 递增、区间首样本前最新命令 age<=.1s，且整个区间命令均 DRIVE/.03/0；不跨状态、不用未来命令。得到 .199533583706789m / wall41.176086s = **.004845860865m/s**，sim40.42s 口径 .004936506277m/s。请求 .03m/s 与实际响应不相等，不据此线性外推 .10 档。

首 DRIVE_SETTLE 零速前/后的真实 10Hz 样本分别 wall64.074021/64.179205s、stamp65.62/65.72s，二者距离 .000334987m；后样本距首零速 .041782s，线速 .0000064m/s。由该后样本到最后一个观测，最大采样距离 .000032216m；所有后续命令零速。STOP 到末样本的 20 个点最大线速 .0000123m/s、角速 0，距首样本最大 .000013971m。这里只是实际采样漂移与命令证据，不能当连续停车距离的严格上界。

DRIVE_SETTLE 到 CAPTURE 的命令间隔 1.016837s（clock 1.0s）；其间保存的 10 个 trajectory 点只覆盖 wall .968699s/sim .96s，不能把下采样行本身称为全量一秒驻留证书。保存点最大线速 .0000113m/s、角速 .0000029rad/s、相对首点漂移 .000010183m，支持静稳观察解释。原 guard 的实时驻留和 freshness 仍按原规则执行，未实现此前讨论的 PAUSE 驻留架构改动。

## 三次 CAPTURE 的原始曝光姿态

三张图均是 `/uav/camera/image`，姿态来自先于曝光的 `/uav/odom`，frame=`odom`、child=`uav/base_link`、image frame=`uav/camera_link`，association=`prior_uav_odom`、association_error=null。UAV 的 z 仍约 .055m，车体携带的相机观察不能称为完成了飞行或空中巡检。

| phase | image stamp(s) | prior stamp(s) | skew(s) | xyz(m) | yaw(rad) | 线速(m/s) | 角速(rad/s) |
| --- | ---: | ---: | ---: | --- | ---: | ---: | ---: |
| capture_0 | 66.814 | 66.800 | .014 | (.126072594,-.158803209,.055000068) | -.862374557 | .000011426 | .000000057 |
| capture_1 | 67.148 | 67.140 | .008 | (.126075125,-.158806231,.055000051) | -.862374555 | .000011716 | .000000015 |
| capture_2 | 67.481 | 67.480 | .001 | (.126078441,-.158806586,.055000203) | -.862372376 | .000018039 | .000109839 |

独立计算三四元数 norm 均为 1，重算 yaw 与原 yaw 一致，目标 yaw 误差最大 .013704738rad、drive yaw 误差最大 .010213818rad。曝光分别发生在首 DRIVE 零速后 1.160750/1.501048/1.844048s；曝光前最近实际命令均零速，距曝光 .004949/.030769/.020623s。三姿态两两最大距离 .000006753m、最大 yaw 差 .000002182rad；三张图 SHA 校验通过。这些微米级仿真差值是本次数值输出，不能声称物理定位精度。

原生图片位于 `runs/analysis/20261112_approach_v18/`：`native_L000013_b9429ec3dc23cab3.jpg`、`native_L000015_065f0f56563a1ec7.jpg`、`native_L000017_b9621149fa948b9a.jpg`，均 800×800。本次目视 capture_0 与 capture_2，画面为同一低视角仿真场景，可见窗框与斜视地面图板。尚未据这些图验证官方七类图库、ground_N 语义或识别正确率，没有把目视猜测写成类别答案。

## 四阶段激光与固定候选几何

直接调用既有工具 `load_candidate_world` 与纯函数 `compare_scan`，固定候选公开提交 `82b041043849a5d86e085390bf0df787366d98cd`，只解析静态 collision boxes 和候选 rplidar 安装位姿。`world_from_odom` 使用预先固定 identity 假设、1mm 容差；不拟合变换、不插值、不改 stamp，不把 ground_bounded_approach 重标成其他 mode。函数的 pose-only 参数由各 record 原 prior position/quaternion 精确组装，原 scan header 与原 prior 元数据一并保留在新 review JSON；不是补造新的 raw。

| 阶段 | scan stamp / prior stamp(s) | skew(s) | 有效/比较束 | MAE(m) | 最大绝对残差(m) | 最短观测(m) | .641m gate 余量(m) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| start | 3.901 / 3.900 | .001 | 360/360 | 2.597e-7 | 2.749e-6 | .650002420 | .009002420 |
| after_turn | 25.006 / 25.000 | .006 | 360/360 | 2.884e-7 | 2.816e-6 | .715518355 | .074518355 |
| mid_drive | 45.510 / 45.500 | .010 | 360/360 | 4.210e-5 | .000217422 | .792472541 | .151472541 |
| after_drive | 66.666 / 66.660 | .006 | 360/360 | 2.614e-7 | 2.078e-6 | .869575202 | .228575202 |

四阶段共 1440/1440 束有限、在 range_min/max 内，所有束在原 1mm 容差内，无未比较束。angle endpoint 数值差 -4.135e-7rad，保持原浮点值。全环原阈值 `.447+.114+.06+.02=.641m` 未放宽；起点仍只有约9mm门禁余量。mid_drive 在移动中，原 prior 线速约 .004795m/s、角速 .001626rad/s，scan/pose 相差 .01s，残差比静止阶段较大与此相容，但没有消除时差或证明其唯一来源。

替代 x+ .1m 的四阶段 RMSE 为 .281392/.279945/.227005/.269327m；替代 yaw+ .05rad 为 .391756/.389171/.380715/.377301m。它们只反驳这两个列明假设，不证明变换唯一。四个阶段包含转向和约20cm平移，支持同一候选静态场景继续作为有限实验几何假设；未独立确认官方地图版本、全部外参、未扫到的高处障碍、目标 ID、穿窗净空或米级完整路线安全。稀疏四帧不能替代每个实际控制周期的实时 scan guard，也不能证明全程绝对无碰撞。

## 输入节律、处理时间与 v17 的比较

本局 sensor_sync：incoming 517、immediate 517、late_prior 0、deferred 0、expired 0、capacity/superseded 0，pending 0；trace 掉 505、末 12 条保留。534 个 `/scan` receive 比 incoming 多 17，STOP 后控制器不再处理 scan，不能据此登记为 17 条传输丢包。末 trace 原 scan 到达 age=0、prior skew .006/.016s。STOP motion_log 边界 scan wall age=.113997s 且 odom/scan fresh；末 scan wall age≈2.137s 包含 STOP 后冻结，不能称活动期2秒传输延迟。

| 区段 | 次数 | wall 均值(ms) | wall 最大(ms) | CPU 均值(ms) | CPU 最大(ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| receive /odom | 3336 | .392775 | 11.634558 | .392214 | 11.635805 |
| receive /uav/odom | 3331 | .144018 | 1.843536 | .143255 | .504837 |
| receive /scan | 534 | 1.042341 | 2.227682 | 1.042239 | 2.226713 |
| tick | 1347 | .214834 | .905166 | .215010 | .905137 |
| spin_once | 16652 | 4.050846 | 29.621424 | .594178 | 12.580737 |

receive 从已反序列化的 Python 回调入口计时；CPU 为当前线程。spin 包括 wait/take/deserialize/callback，包含 receive，不能相加，也不能将 wall-CPU 一概叫网络延迟。12 个保留慢事件全为 spin（另370条未保留）；graph refresh135/query810，query总 .044415278s、最大 .000241605s。没有看到本队单次 receive/tick/spin 百毫秒处理阻塞的证据，并不等于已独立测清 DDS、共享桥或平台资源配额。

| 指标 | v17（首点 ground_1） | v18（首点 ground_0） |
| --- | ---: | ---: |
| /odom header gap 最大(s) | .18 | .04 |
| /odom 入口 wall gap 最大(s) | .197267 | .039021 |
| /odom header gap (.1,.2] 计数 | 203/2904 | 0/3335 |
| /uav/odom header gap 最大(s) | .18 | .14（仅1/3330>.1s） |
| /uav/odom 入口 gap 最大(s) | .223591 | .150696 |
| /scan 入口 gap 最大(s) | .157623 | .161124 |
| late_prior / expired | 144 / 169 | 0 / 0 |
| PAUSE 总 wall(s) | 40.866028 | 0 |
| 有效 DRIVE 前向速度 wall 口径(m/s) | .005015361 | .004845861 |
| 前向投影(m) / capture | .076843148 / 0 | .200151226 / 3 |

v18 `/odom` 3335 个相邻 header gap 均<=.05s，入口 clock-header 为 [-.08,0]；`/scan` 入口 age 为 [-.066,-.001]，header gap 最大 .152s、入口最大 .161124s。age 是最近 clock 与原 header 差，不是绝对传输时延，负数不代表消息来自未来。UAV camera_info 的落后仍存在：189 条中38条入口 age>.2s、最大2.986s，入口 gap 最大3.739721s；UAV image 一条 age=.285s、入口 gap 最大 .699704s。当前地面 guard 不使用 UAV camera_info；这些问题不能作为本局地面停止原因，但后续视觉控制仍需处理。

**因果边界：** 软件差异是单变量 `/odom` reliability，但两局不是相同 seed；首目标从 ground_1（约-90°）变为 ground_0（约-50.2°），且未改 QoS 的 `/uav/odom` 本局也明显改善，存在共同输入/环境调度改变的可能。结果支持“BEST_EFFORT 配置本局可用，当前反复暂停的表现已解除”，不支持“已证明 QoS 是唯一根因”、总体稳定率95%或更高档位安全。保留原安全合同，下一步需要同样受界限保护的短档位校准和逐段任务覆盖。

## 台账与校验

台账 `runs/records/ground_odom_qos_v18_5038ac33.json` 按最新终态 failed/null/141.089s 记录，kind=interface_probe、practice、full_mission_completed=null、各项得分全 null。`python tools/run_records.py validate runs/records --verify-files` 已验证 22 条、0 重复、files_verified=true；所有正式完整任务有效成绩仍为 0 局。新 review 脚本只做离线算术和原始文件哈希前后校验，没有运行机器人、访问网站或新增提交。
