## 21:50 最新接续：完整 v44 已上传，等待真实运行

本文件保留历史接续；其中认证详情 JSON、原始回传和未公开的分析路径仅在本机归档。公开仓库未包含这些文件；原始记录中的 SHA 指本机原件，公开副本哈希见总清单。

- 占位地面 v43 已结束，官方 failed、score=null、182088ms。未取消或覆盖它。
- v44 在21:50:14上传成功，POST201，ID `9142b55c-dcc7-4919-87d1-c8041a551108`；初始 submitted、runId空，尚无新得分。源码及脚本与下方原始冻结SHA一致；公开脚本已脱敏，双SHA见 PUBLIC_SNAPSHOT_MANIFEST.json。

## 21:45 历史接续：完整 v44 已修复，1823 项通过，提交被其他测评占位

- 完整 v42 已官方结算 **10 分**，只有 `ground_4` 计分。旧回传通道在排队期间断开，不能确定该局停止分支。官方证据 `runs/evidence/20261112/full_v42_detail_final.json`。
- 完整 v43（冻结 v41 同源码，仅恢复回传）ID `b07f927e-cd6c-46f5-bdac-eb10f97e8ab6` / `run_20261112_211323_9aa9f2e8` 已官方 failed，未提取成绩，397993ms。本队程序在地面约179秒后停止，exit1，draw g2→g1→g4，completed0，返航和飞行均未开始。双 SHA 回传包解在 `runs/analysis/20261112_full_inspection_v43/ground/`。确定根因是 entry 规划出4.6725mm微轴，定稳后不断追逐漂移的近场方位，耗尽45秒。不是跨操作计时未重置或物理无响应。
- v44 已修复并冻结，完整测试 **1823 passed / 249.26s**，报告 `runs/review/full_v44_regression_20261112.xml`。仅两个生产模块变化：axis_plan省去原执行器不接受的≤20mm微轴并保留真实坐标和最终径向门；总桥将大诊断延迟到全部控制结束，任意安全清理错误仍阻止飞行。原45秒、15米、40弧度、传感器时效、净空、视觉判定未放宽。总桥独立9类故障注入与规划1471姿态/约12万采样均通过。
- 新包 `dist/inspection_full_mission_v44_src.zip`，13378987B，SHA `eb1cf9eb5263f89fda0614cf5fe5302d307818337d8920d4f59bbc0a8de804bf`；脚本 `dist/run_full_mission_v44.sh` SHA `022d908fd0f9b6cfbcf50bf1af8552aba7ce59d6cca57245bebe3cf6fc44e03c`。76个源码文件与ZIP逐字节相同，bash-n通过。**尚未成功提交**：POST返回409 `EVAL_IN_FLIGHT_LIMIT`，不能说已排队。
- 占位是账号内另一条 `ground_v43_src.zip / run_ground_v43.sh`，ID `0c8def80-8378-4679-865a-2d25c6f2a9e7`，21:34创建，run `run_20261112_213557_0e082881`，最新 running。不是本轮代理提交；没有取消或覆盖。已异步询问用户是否另一个任务发起，等终态后提交完整v44。只读下载比较：其原76源与冻结v41一致，仅加ground_practice.py观察入口，无微轴修复、无飞行；因此不代替完整v44。
- 平台会话PTY14846仍认证，凭据只在内存。v44 receiver PTY30654/port53915/max10800秒，callback `runs/evidence/20261112/callback_full_inspection_v44.jsonl`，delivery独立；v43 receiver已停止，cloudflared52902仍绑定同端口。v44新随机路径已GET405/POST202实际验证，自测独立存 `callback_full_v44_selftest.jsonl`。能力URL仅在dist启动脚本，不进文档/GitHub。
- 不再重复全套测试。后续依据平台真实断点继续修，不能把本地通过称作完整得分；真实完整演示视频尚未产生。具体诊断文档 `docs/full_mission_v44_ground_fix_20261112.md`。

## 12:50 历史状态：官方仍离线，备用隧道又断连

- v42仍为submitted、runId空、serverStatus=down、pendingResubmit=true、成绩null，无accepted callback。已排队的唯一完整任务ID保持 `69f58da5-1669-4433-a15b-3adb4e3f2392`，不要把此状态写成已启动或运行失败。
- 12:35公网GET405曾真实成功；约12:47补做invalid POST验证时返回非JSON且没有抵达receiver账本。检查 **PTY77290已退出1，`client_loop: send disconnect: Connection reset`**。目前v42已上传脚本中的临时回传地址**不可依赖**，不要沿用下方“通道可用”历史状态。尚无本局运行号，故无法恢复本局任何运行证据；不能把未来无回传认定为算法没启动。
- receiver **PTY94978 / PID21296 / port64553**仍活，accepted文件尚不存在，delivery文件目前只有此前一次GET405健康检查。平台HTTP **PTY18418**仍已认证。若继续实测，需先恢复可持续的回传通道或平台可用日志；不应继续盲目复制新队列。
- Cloudflare两次申请429、备用SSH随后被reset均为实际外部连接故障，修复代码本地103项绿不等于网络可靠。`full_v42_callback_health.json`保留GET成功及后续POST失败两个事实。
- 重新读取本队技术求助详情成功：`support_post_recheck.json`确认平台审核回复与历史一致：工作空间在官方网盘，测评侧构建/运行日志均不展示。公开评论无回复；本地已保存课程/指南/转录/PDF仍无实际网盘链接/提取码，不再无变化扩大搜索，也不额外发新消息。
- 用户最新问题“全部都补完了”须回答**没有**。已完成完整任务候选代码及本轮启动诊断修复；完整平台验收/穿窗、照射、降落计分/真实视频与正式文档仍未完成。当前官方离线并且回传断连，不能承诺后台继续已完成或新局已跑。

## 12:40 历史接续

- **完整 v42 已实际提交**：ID `69f58da5-1669-4433-a15b-3adb4e3f2392`，12:37:55 创建，当前 submitted、runId 空、serverStatus=down、pendingResubmit=true；平台明确会自动重新提交。只跟踪这一条，不重复建局。成绩 null，尚无真实运行事件，不能声称任务已执行。
- 源码继续用冻结 `dist/inspection_full_mission_v41_src.zip`，13,378,153 B / SHA `a2eaafc89bbfc8a185496e5df0f707f365652790d0747b548aa29cd8d9c6663d`。仅新启动诊断 `dist/run_full_mission_v42.sh`，5,011 B / SHA `7299ec1761e61737e7af545e401a2c01d0da515636d6d01f795ffa96b9729617`。平台保存的两文件已重新下载核对长度及 SHA，全一致，证据 `full_v42_uploaded_bytes_verified.json`。
- v42 启动脚本、receiver、telemetry、总桥最终联合 **103 passed / 19.77s**。接收器可选 `--delivery-log` 最终生产 SHA `ef7d0d752d4981de9dae468b2e80f48ea41213538e4403012f99d2e37537a18d`。根代理已非作者核对，无阻断；控制和视觉未改，不能将本地检查当实测成绩。
- 新 receiver **PTY94978 / port64553**，max7200s，约12:30启动；accepted `runs/evidence/20261112/callback_full_inspection_v42.jsonl`，独立投递账本 `delivery_full_inspection_v42.jsonl`。公网 GET405 已验证，accepted 未混入测试事件。当前只有 GET 健康检查，未有 launcher 事件。
- Cloudflare 新申请429，已停止申请；曾误把错误中的 API 域名当成分配地址，已纠正且未写入/提交错误地址。备用 OpenSSH 反向通道 **PTY77290** 已端到端验证，回传地址仅在新 dist 脚本。免费域名可能轮换，检查时勿把管理/文档域名当真实 endpoint；不要在报告输出能力路径。旧 v41 receiver PID4408已停止。
- 平台会话仍 **PTY18418**。详情查询 `GET /api/ros-simulation/evaluation/detail?id=69f58da5-1669-4433-a15b-3adb4e3f2392&withLogs=1`，保存为 `full_v42_detail_latest.json`；本局台账 `runs/records/full_inspection_v42_69f58da5.json`。v41服务器恢复后 refresh-logs 查询读超时，无新日志证据。
- 平台约12:31短暂up，12:36再次down；因已有终态 v41 且新诊断完成，决定只将新完整任务排一次自动队列。当前不再要求用户提供缺失工作空间；不回到局部升降探针。
- 完整五项任务尚未获得平台计分，正式真实视频/PDF仍缺。有效基线仍是 v25 首个地面点10分。用户问“全部都补完了”已明确回答没有。
- 时间只读报告 `runs/review/full_mission_timing_20261112/review.md`：名义完整代理520–571秒，真实响应外推1420–1670秒，均非完整实测；具体平台时限未知，v41 abnormalEnd=false，不能认定291秒就是超时。
- 子代理全部已结束；不重复大套测试。后续先等本ID分配runId/launcher回传，依据真实断点再修完整流程。

## 12:26 历史接续

- 用户询问“全部都补完了？”，已明确回答没有：完整代码已上传，但完整实测和正式真实视频/文档尚未完成。唯一已确认有效计分仍是 v25 首个地面点 10 分。
- 完整 v41：`d2d6ba08-ac43-42f7-a77f-a08eee68c53d` / `run_20261112_114542_50c9a8cd` / `#55477230`，failed / score=null / 291350ms。当前 detail 仍 serverStatus=down；最新 refresh-logs 返回 502，“测评服务器上暂无该次运行的日志（可能已被清理）”。本队 accepted callback 从未出现，仍不能区分没启动和投递失败。
- 独立启动核查未找到确定包内错误；已复现诊断单次请求、静默吞异常以及接收器拒绝无记录的盲点。v41 源码 ZIP 和脚本继续冻结，不以诊断缺失归因算法或伪造完成标志。
- 已新增完整任务 `scripts/run_full_inspection_v42.sh`，只改启动诊断：started/exit 最多各三次、每次三秒，要求 202+JSON accepted=true；ROS/source 两阶段 heartbeat；安全错误分类，真实任务退出码不变。14 项专项已绿，源/视觉/控制 76 条冻结文件未变。接收器可选独立 delivery-log 正按红绿测试实现；不要把其预期 RED 当 v42 回归。
- 接收器原进程 PID4408/PTY2120/port63984 仍活至原期限；旧 cloudflared PID13492 已因明确的 Tunnel not found 错误停止。下一轮必须建立新通道，不能复用已失效 v41 URL。平台 HTTP PTY18418 认证仍有效，凭据未落盘。
- 下一次仍须完整五项任务，不退回局部探针。先完成接收器与必要联合核对，生成只改诊断的 v42 脚本、验证新通道，再待真实服务器可用提交并跟踪。
- 执行计划：`docs/superpowers/plans/2026-11-12-full-mission-delivery-diagnostics.md`。axis 负责 launcher（已完成14专项）、ground_next_gate 负责接收器、height 只读核时间预算；根代理统一收口，不重复全套千项测试。

## 09:50 历史接续

- v38已结束：`dccfbacf-1728-4500-b45c-5ae4105d5f66` / `run_20261112_093301_624e5479` / `#13840696`，官网未通过/0显示/未提取/202.8秒、09:37:19结束。实际三地面draw g3→g2→g0，首点观察完成后 `single_point_visual_binding_rejected`，3帧采集、识别与到点attempts均空、0完成点。实测累计1.639658439m/2.072342365rad，仅一次scan_stale恢复1.104354451秒，相机重采0；不能写成v36相机恢复在实测已触发通过。8/8包双SHA/长度通过；原始SHA ba8cc75e76b677ba5ef4e11861c2bc9af2c9b0033f1a08dc718b280cb1060854。工具原只收旧mode，已增加两个实名恢复mode并15/15专项通过，未改原始证据。解包/三原图/离线视觉重放在 `runs/analysis/20261112_three_point_recovery_v38/`。重放三帧均无accepted candidate；K取真实CameraInfo首尾相等的稳定假设，缺运行时拒绝worker结果/每曝光K，不能称精确复现。axis_final_review独立核资源/时序；height_v34_analysis只读视觉根因诊断中。
- v37已完成作者36项/collector42项及独审4/4，先前终态/跨阶段延迟cloud误占槽缺陷已修。最终height SHA65408ea8d17f2cb1629eef1a1509b68e547ed1392c2c2c086fbd47ca2ee1b96d；ZIP `dist/inspection_cruise_height_v37_src.zip` 9599549B/SHA0ff78f3748f33937eaa899c2615a2ac5dd57dd803293ad647b0967c67493c3b9，仅在v34原48项基础加两个模块。上传脚本 `dist/run_cruise_height_v37.sh` SHA cf7131ca66ac24c453d85cae03cc43c74c4a25e4490a969511ce348d58b405d8。
- v37已09:48:38官网练习提交，ID `1fa480e1-6989-497d-bbd2-f59d481dbd08` / `run_20261112_094846_c41549bc` / `#93288803`。当前heightTab（IAB2标签8）在该详情，待运行。旧v38 receiver23048已Ctrl-C停止；新receiver91317/port52023/max900秒约09:43启动，输出 `runs/evidence/20261112/callback_cruise_height_v37.jsonl`。复用33128 cloudflared；新随机回传URL仅在dist脚本，GET405验证。CUA刚重置过，仅heightTab变量存活，fs需重新import；本轮CUA尚未completion_summary，待观察实际终态后再写。
- ground_next_gate已完成60种三点+返航几何（max13.99m/37.75243254rad，名义min ring .650）；独立返航v40设计获root批准15m/40rad/1800s、turn1000s/drive650s，其余原门及32/5/60资源保留。它正以独立文件实施，同guard跨三点返航不重置；还未测试或上传。不要将名义60/60误写为2cm误差鲁棒：g2南偏20mm三个yaw被原门正确拒绝，75样本72可规划。

## 09:33 历史接续

- v38 已于09:32:49通过官网练习上传，ID `dccfbacf-1728-4500-b45c-5ae4105d5f66`。当时页面“已提交”，编号与runId尚未生成；不能当成运行或计分通过。源码包SHA `b9874ada14c3df72483ea85eedebdfa02fbe1a245bb13da926e6d92bd524b85b`、上传脚本SHA `18d97806f0d941f1f338b42cbab5d10bef44c6b8695130b0c4e540281e65b93f`。接收器会话23048、端口52023存活，尚未有回传文件。当前内置浏览器heightTab/标签8在该测评详情。
- 非作者v36/v38审查已完成，无阻断：v36 5/5、v38 10/10、静态2/2，包基底50项逐字节不变，仅加2模块。报告分别在 `runs/review/ground_camera_recovery_independent_20261112/REVIEW.md`、`runs/review/ground_recovery_budget_independent_20261112/REVIEW.md`。完整资源账本需重组gzip bundle，不能只看core轻量段。
- height_v34_analysis继续v37 .85m固定柱实验实现；ground_next_gate只读审查地面到飞行及窗定位最小缺口，未改源码。

## 09:28 历史接续

- v34 完整独立分析已完成：原固定低高度合同成立，实升0.100952694m，HOLD源/墙3.04/3.094928秒，回支撑原墙钟1秒成立；额外“FINISHED前源/到达双1秒”不成立，必须区分。官网仍未提取任务成绩。报告 `docs/height_runtime_timing_v34_platform_analysis_20261112.md`。
- v36相机恢复新模块冻结2c7addc67afda66e4aab53b1796dc0537756fb6ffd907647eb998a6699b809ee，22/22专项、108必要关联通过，axis_final_review独审中。
- v38资源适配由主代理新增 `ground_three_point_recovery_budget.py`，冻结f50c7058d8a6727ccac83d03c228d6a2c9e01249c55171aa55758a0b272cdc1d；脚本ae32f4c32c20b1071c0d2eb7426f50c1748a9b9cb415d4547d2662f964fd24ce。32次/单次5秒/累计60秒/全局1200秒合同，原freshness/净空门不改，跨点账本不清零。9/9专项，必要关联58/58，红绿证据已有；交axis_final_review定点独审，尚未上传。
- v38包 `dist/inspection_three_point_recovery_v38_src.zip`，9607969字节，SHA b9874ada14c3df72483ea85eedebdfa02fbe1a245bb13da926e6d92bd524b85b；以v35为基底50项原字节，仅加v36/v38。运行脚本 `dist/run_three_point_recovery_v38.sh` SHA18d97806f0d941f1f338b42cbab5d10bef44c6b8695130b0c4e540281e65b93f，加入本轮回传环境行；接收会话23048/52023/1800秒，仍复用33128隧道。GET完整随机URL已405核验，未污染实际证据。等待审查后直接官网练习。
- height_v34_analysis在实施独立v37固定柱：目标绝对.85m/硬1.00m/运动90秒/全局110秒/path2m，保留控制门，scan高于墙后+inf仅依固定静态柱无回波新合同，不当未知空间自由；最多4帧完整点云/原header/TF/prior，停止后编码。父层冻结不改，作者专项与独审后再上传。

## 09:01 历史接续

- v35 连续三点已结束，官网未通过、未提取成绩、184.1秒。首点 ground_1 观察完成后，RECOGNIZE_WAIT 因 image_wall_age=0.530038s 超过保留的0.50秒门停止；抽签 ground_1→ground_3→ground_2，完成0点，识别和到点 attempts 均空。累计路径1.113765653m，累计角2.169321339rad。
- v35 已核7/7分片、双SHA、长度和core一致。相机最大回调入口间隙0.696315秒、source跳0.668秒；odom最大0.069165秒、scan最大0.196376秒、tick最大8.013毫秒。独立matcher每帧约1秒，不支持“堵塞ROS主回调”的结论。相机stale未进FRESHNESS可恢复分支是永久停止机制。axis_final_review补完整分析；flight_fov_diagnosis正在新独立v36修复，保留.50/.20/.05门及原总预算、每点一次重采和全局三次恢复，不改冻结v35。v35已用完三次恢复，同样时序必须因额度耗尽停止，不能宣称新恢复能救回该局。
- v34 已真实上传结束：ID `0d99c6fc-e6b7-4a69-aa35-abad82dbcbc0`、runId `run_20261112_085752_4f79c7da`、#22362608。官网未通过、未提取成绩、页面0、56.1秒，08:59:11结束。它是无计分声明的低高度实验；官网提示非零飞行后不再计地面分。
- v34 原始 `runs/evidence/20261112/callback_height_timing_v34.jsonl` 共12事件、299790字节、launcher_exit=0。height_v34_analysis正在独立核对时序与高度；初读protocol_complete尚不能代替复算。官网证据 `height_timing_v34_official_page_final.json`、`height_timing_v34_final.jpg` 已保存。
- v34 包SHA为 b55882ce5627270292685a854b716455737a1fc5d3efa5d7de0103869af49924；加入本轮临时回传后脚本SHA为 486a9a99ea61c59cda29037692274e1958383f802940106581fdafe1dfbc0af0。接收器会话45086、port52023、max900s，精确停止旧v35 python30504后复用同端口隧道33128；随机路径只在运行脚本，不写报告。
- 三个网站辅助模块08:15–08:20已实际触发，仍未开放，不重复无变化操作。
- 官方有效得分基线仍v25首点10分；完整巡检未完成，正式材料不得虚构完整成绩。

# 当前真实平台状态（最新更新：2026-11-12 08:49，UTC+8）

## 08:49 接续状态（优先于下方历史记录）

- v25有效官方计分基线仍为首点10分，完整巡检尚未通过。
- v33有限倒车已实测完成并独立重算：ID`87e16c7f-bc1c-4329-aef2-bfff61a13e85` / `run_20261112_083208_b88d3f13` / `#30991200`。官网仍未通过/未提取成绩/341.2秒；它没有计分声明。真实南移148.180mm，三负X分别向南10.189/11.476/11.482mm，归零至静稳最大累计路径1.348mm，113/113分片、双SHA/core/完整性通过，2336非零fresh违规0。3次scan超龄PAUSE恢复；不能泛化成长路线稳定性。报告`ground_relocated_reverse_v33_platform_analysis_20261112.md`。
- **v35连续三地面点正在本队练习平台运行**：08:43:50上传，ID`af0fe738-c09b-4f4c-a2ed-fe51b0e0b298` / `run_20261112_084355_23ba4d48` / `#92599511`。回传路径`runs/evidence/20261112/callback_three_point_axis_v35.jsonl`；目前启动两事件，未有最终轨迹。不得提前写成三点成功。包与脚本在`dist/inspection_three_point_axis_v35_src.zip`、`dist/run_three_point_axis_v35.sh`。接收会话89951（端口52023，2400秒），隧道会话71962；仅原始回传用于分析，不读私有裁判数据。
- axis运行器源码e033…602f76已由非作者独审11/11放行，报告`control_review_ground_three_point_axis_runtime_20261112_v2.md`；v35包以冻结v26为基底，加入两个新axis模块及冻结vertical依赖，原基线模块逐字节相同。
- v34只增加时序诊断，不改变v31控制合同，已独审8/8组放行；源码cb2c0b4d…f8dee6be、脚本29201e13…15e010d1。源码包`dist/inspection_height_timing_v34_src.zip`已生成，但**未上传**，等待当前唯一巡检测评结束。报告`control_review_height_runtime_timing_v34_20261112.md`。用于定位v31共同64.643ms记录空档，尚不能说原因已修。
- 08:15–08:20三个辅助模块实际操作仍被“暂未开放”拦截，已记录。用户要求使用这些工具，不代表绕过其功能限制；继续开放练习入口。

## 08:20 接续更新（优先于下方07:48旧记录）

- 有效计分基线仍为v25首个地面点10分，没有完整巡检成功证据。
- v30已真实执行：`e5d26785-01cd-41a6-acb8-02098e088649` / `run_20261112_080340_fad7c927` / `#11416510`，failed/null/38.644s。原点正转+.119963rad后因净空门停止，负X脉冲0；不能说倒车已验证。几何独立诊断见`ground_reverse_v30_clearance_diagnosis_20261112.md`。
- v31高度闭环已结束：`f21b350d-dcba-4121-ae93-9781c554471c` / `run_20261112_081239_56ea65d7` / `#85465159`，官网未通过/33.8s/未提取成绩，页面显示0分。原始回传显示上升阶段因单次scan到达间隔.207634s超过固定.20s门永久归零，未进入HOLD；实升峰值.055820m，HOLD/下降/返回支撑验收均未通过。报告`height_hold_v31_response_analysis_20261112.md`，官网快照`runs/evidence/20261112/height_v31_official_page_final.json`。
- v33为南侧预定位后负X响应，flight_fov_diagnosis正在独立新文件实现，尚未上传。v32无确认文件/上传证据。
- axis新运行器源码`e033c119790ad8a32a066cd21cd9fd841e5b967092b8d0ac0355fd5d5f602f76`作者回归通过；非作者axis_final_review正在最终复审，未上传，仍依赖真实倒车响应。
- 08:15–08:20重新实际操作三个辅助模块：加载iCAN场景、向AI助手发送当前调度问题、运行本队只读视觉代码，三者仍显示“测试阶段 · 暂未开放”。详见`platform_tools_availability_20261112.md`。继续开放的练习测评，不绕过拦截。

## 07:48 历史记录

本文件接续较长的 continuation_state_20261112.md。以原始平台快照、冻结源码哈希及本队原始传感器证据为准。用户授权持续练习；正式竞赛材料尚未提交。

- **有效官方计分基线仍为 v25：首个地面点 ground_0，10.00 分。** 详情 `c4202874-407a-4361-844a-0f95ba7b1d7f`，runId `run_20261112_060713_501f412f`。不是完整巡检完成。
- v26 三点在 ground_4 首段横偏 20.263mm 停止，尚未计点。已确认三点规划重新引入斜线，违背 v22 已采用的轴向修复。新轴向候选规划已独审通过，但新执行器和倒车仍待实测。
- **v28 三组方向辨识已真实完成；官方任务状态仍 failed，score=null。** ID `7724ce5c-3e6b-469e-a368-2e6aa7025044`，runId `run_20261112_073445_70f255cc`，编号 `#15047677`，100.328s。它不发识别、到点、飞行或任务完成消息，故不能用本实验取得任务分。
- v28 原始回传 35 事件、25 个 gzip 分片；双 SHA、长度、core 对照及所有轨迹重算通过。1473 odom、219 scan、727 命令，drop=0。三脉冲的实际方向与机体朝向偏差约 0.135/0.199/0.167rad；单次短样本不能当全域控制模型。各归零请求后的累计路径约 0.064/0.110/0.489mm。证据见 `runs/analysis/20261112_direction_v28/README.md`、台账 `runs/records/direction_v28_7724ce5c.json`。
- v27 低高度上升后，零指令导致回落，尚无悬停结论。Z trim 新实现的独审曾发现 ground publish 延迟后过期负向命令仍发送；已修为返回后重查真实 DDS/时间/阶段/源龄。新源码 SHA `54744be026c23d43bfe8dba5686ffd7d25d9028a8d4eaf06e667501b5984d89c`，待独立复核后才上传 v29；尚未上传。
- v30 独立负 X 响应正在实现。新转角预算 2rad、目标 +pi/2；位移、净空、freshness、总时长及停车约束不扩，先验证真实倒车再接南侧点。
- 新轴向三点 runtime 正在独立文件实施，不改 v25/v26。预先新转角预算 28rad（几何名义最大26.703538）；path12.5m、总1200s、其他原约束保留，未上传。
- 网站仿真中心、AI 开发助手、算法解析此前多次实际操作均弹“暂未开放”，不能声称已运行。记录 `docs/platform_tools_availability_20261112.md`。测评入口可用，用户构建/运行 stdout 不展示，使用本队有限回传取得证据。

尚未获得完整三点、穿窗、空中照射或返航降落的官方计分证据，正式视频/PDF不得写成已经完成。完整飞行控制器仍有视场覆盖问题，详见 `docs/flight_fov_diagnosis_20261112.md`。
# 完整任务整合续接：2026-11-12 11:10 +08

## 12:14最新实际状态：v41已失败且日志服务器不可达

- v41 ID `d2d6ba08-ac43-42f7-a77f-a08eee68c53d` / `run_20261112_114542_50c9a8cd` 已终态failed，duration291350ms，11:53:04更新，score=null，原因未检测完整结束标志。当前serverStatus又down。不得沿用下方queued/submitted旧状态。
- 本队callback文件从未出现；既不能证明算法运行了，也不能仅凭无回传断言脚本未执行。用户问“全部都补完了”，已明确回复尚未全部完成，完整平台验收/计分/正式真实演示材料仍缺。
- 新前端详情需withLogs=1。已从当前公开chunk确认并调用：GET detail?id=...&withLogs=1只有空裁判tab；POST /api/ros-simulation/evaluation/refresh-logs（id+userId）返回502，message `无法从测评服务器读取日志：evaluation server unreachable (circuit open)`。证据full_v41_detail_with_logs.json/full_v41_refresh_logs.json。
- axis_final_review重新只读核包：76文件manifest/CRC/53Python AST通过；2901B脚本无BOM/CR/NUL、bash-n/内嵌Python compile通过，205B launcher_started被实际receiver validator接受。该事件位于ROS setup/源码路径发现/算法import之前，后段错误不能单独解释无回传；暂无可复现包内启动根因。
- cloudflared日志11:56:56掉线晚于官网11:53结束，不能直接拿该掉线当本次失败根因。12:10仍边缘重连超时；receiver2120仍活。完整包/脚本保持冻结，不无依据重打包或重复上传；下一步先恢复实际服务器日志/回传再定位完整运行缺口。
- 本局台账已更新failed/null，runtime_started=null表示未知而非false。尚未制作或声称正式真实完整运行视频/PDF。

## 11:28已上传完整v41，优先跟踪本局

- **11:47恢复**：详情serverStatus=up/serverDown=false/pendingResubmit=false，status=queued，已分配runId `run_20261112_114542_50c9a8cd`。继续跟踪本ID；截至此刻尚无callback、成绩null。receiver公网GET405仍健康。此前down段为历史状态，勿当当前仍离线。

- 11:32详情关键字段明确 `serverDown=true, serverStatus=down, pendingResubmit=true, pendingResubmitHint=将自动重新提交并继续测评`；故本局未启动、runId空、无本队callback，不能称代码测评失败。保留本局队列，不重复上传；下一步等官方服务器恢复后跟踪同ID实际运行。

- 官网练习POST已返回201/success，ID `d2d6ba08-ac43-42f7-a77f-a08eee68c53d`，创建11:27:54+08；当前submitted，runId尚空，未取得任何本局成绩。
- 完整源码 `dist/inspection_full_mission_v41_src.zip` 13378153B，SHA `a2eaafc89bbfc8a185496e5df0f707f365652790d0747b548aa29cd8d9c6663d`；启动 `dist/run_full_mission_v41.sh` SHA `7eeb6f05399bdfd82b538322065fa0fb9a2e4404ac5576ac986716a729b50309`。76个包内源与本机逐hash一致，bash-n通过，最终full+bridge27项绿。不要覆盖重打本局包或脚本。
- 最终floor `a0ae1f6eb8e9950e69eb0e4a7bceffef32abeb1809c51901b6266b2bdbcdafd9`：150完整全类fastpath、无候选300完整全类fallback，14专项全绿，正常warm均.189s/max.197s；cold仍2.807s，首帧按原.5门如实过期。
- 证据 `runs/evidence/20261112/full_v41_submission.json`、`full_v41_detail_latest.json`；本局台账 `runs/records/full_inspection_v41_d2d6ba08.json`。详细GET `/api/ros-simulation/evaluation/detail?id=d2d6ba08-ac43-42f7-a77f-a08eee68c53d`，保存文件是success/data嵌套。
- 会话18418仍可JSON命令；receiver2120、tunnel54329仍在。CUA故障已completion_summary，不反复重试。open_in_codex打开本局URL返回queued；不得称网页已自动显示。
- 跟踪至真实终态；每次failure读取本局原始callback与裁判原因修复完整任务，不恢复局部升降路线。当前全部代理已完成，可据回传followup分析。

## 后续优先更新：等待一次最小fallback收尾即可提交

- full flight现冻结 `619d3dab44c0c47251d8a603946dbc84aa8f60367d1ea0b6e38b4898222209cd`，已接floor生产接口；水平面Z(.005m候选界)与原视觉XY不确定性分开预算，原fix/stamp/位置不改、原.12/.145门不变。用实际localize输出误差.056395的离线body命令闭环317s/19.342m/FINISHED；这是合成动力学，不是平台成功。
- root性能检测发现旧floor warm .49-.75s与.5s有效期冲突。height将参考优化为150全类+镜像，正常warm约.22s；axis与ground两套真实monotonic异步factory都验证冷首帧如实过期、后续新鲜结果连续确认并恰好一次识别。没有重写曝光stamp或扩大.5s门。
- 150-only中间版ce852363必要测试13绿1红（镜像+90度正控），禁止称14绿。root已明确让height现在做150完整全类fastpath、无候选才300完整全类fallback，保留全部原门/类别冲突检查；等最终14绿+SHA，axis只需相关联合复核即可打包，不再泛化扩展。
- ground_next_gate已独审full619d的图像/fix/光反馈闭环与两原负控，无阻断；并完成解包器新增ground_three_point_return模式，16解析测试绿。该代理已完成，可需要时followup；height与axis仍活跃。
- 仍未上传v41。平台会话18418、receiver2120、tunnel54329均见下方，URL只在当前工具上下文。构建后即通过练习POST，不再局部升降。

- 用户要求下一局执行完整五项任务，合计90分；已授权修复后练习上传及失败重试。尚未上传v41，当前官方成功基线仍v25地面首点10分。
- 总桥 `full_inspection_runtime.py` 最新SHA `f8fe6bb1be6d43782d4f844ce65697ac36617d3cb7caa14ed54e218c87b1d5f5`。7专项绿；修复清理零返回False仍起飞；有axis记录则核对本次新零记录；每5秒旁路非阻塞heartbeat。
- 新十图库 `ground_template_catalog.py` 恢复两张下载失败图片，原地面门不变。Recorder异步gzip分片已修超64KiB，SHA `46871282184487d3ad859e65b3e93c0a74cc325a297b9ad0eb8808d2be1e6022`。最新root定点组合19项绿。
- v40作者85项绿，ground return SHA `b70cd5dde22054cae748cbb6bfcff7e26c5b40c5ba80f1760c8b2aac355d79dc`。非作者真实factory proof与飞行验证相容；10冻结父hash一致。
- 完整飞行 `full_flight_inspection.py` 最新SHA `2a2d34bfc5fdddf2e62460a7b5fd362d9fb70d9be2fdc0ff17619d57b61cc813`，18专项/必要组合162项绿。四个已复现门控bug已修并独审：垂直扫掠遗漏、零迟返陈旧落地完成、同stamp内容/墙钟改写、落地yaw；window四图实际匹配通路通过，窗口/空中拒识和错stage也留有界JPEG副本。
- 最后生产缺口：正确UV的空中小板合成图，原ORB10图全空；IPM+参考缩小的离线原门10图通过无错类。height_v34_analysis正在新floor模块，axis_final_review负责主模块接线，ground_next_gate负责独审。三者均活跃，不重复建代理/复跑大套。
- 相机候选外参z已改.130；公开world和launch支持，v34同stamp基座/UAV原点近一致。没有平台camera TF实收记录，不得称已实测标定。地面板plane .012不可重复从camera offset扣除。
- CUA与Node均报kernel assets路径错误，reset无效；可用替代为已观察的 `tools/platform_session.py` HTTP会话，PTY **18418**已认证且GET evaluation/access为200、inspection订阅有效。凭据仅会话内存，勿保存密码/认证响应。
- 新receiver PTY **2120**，port **63984**，max7200s，输出 `runs/evidence/20261112/callback_full_inspection_v41.jsonl`；新cloudflared PTY **54329**，进程13492。端到端GET405验证，无测试事件混入。临时URL仅由当前工具上下文掌握，后续仅放dist脚本，不进文档。
- 打包工具 `tools/build_full_inspection_submission.py`、入口 `scripts/run_full_inspection.sh` 已准备。等待floor接线和独审后构建 `dist/inspection_full_mission_v41_src.zip` + `dist/run_full_mission_v41.sh`，插临时回传export后重算manifest script SHA，再通过官网练习POST上传，绝不再上传升降局部包。
- 非作者飞行证据 `runs/review/full_flight_independent_20261112/review.md`；图库/Recorder/桥独审 `runs/analysis/20261112_v38_visual_root_cause/`。v37台账已更新failed/null，不能把页面0分fallback当真实提取成绩。

