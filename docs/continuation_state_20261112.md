## 07:25 ???????/trim??????planner????????

- ????docs/ground_observation_route_v22_axis_observation_20261112.md??????????????????????????????ground_three_point_axis_plan.py?SHA e1bb65499f24eba7389b645dd29c10539bd7942fa51675d57912d95454deba8e????????????ROS?g4??(.35,0)?(.35,-.1)?g1/3?????g2???(0,-1.8)??????(0,-2.69)??????DRIVE_REVERSE_TO_XY??????????RED?21??????????501???5??????docs/ground_three_point_axis_design_20261112.md?
- 60?????11.67m/26.703538rad??25rad?????????????????????1032.684s????budget?runs/analysis/20261112_axis_plan/budget.json?
- ?????flight_fov_diagnosis??ground_direction_response?envelope_independent_review???bounded_vertical_runtime??bounded_vertical_trim??????????????????????????

## 07:18 ???v26????????v27?????????

- v25????10????????????????????????/PDF????
- v26 ID0425d992-ddd4-4b91-9d21-b3d6a394728e?run_20261112_070005_77bdabed?#90809618???failed/score=null/98.393s?ZIP SHA044ca528340f29decdf0c5776bc06a9be8e5ec2e3fb06c634609cbe46cffeb61???ground4,ground1,ground3??DRIVE??20.263156mm?????0????????runs/analysis/20261112_three_point_v26/??????drop???callback???????SHA?
- v26?/bearing??????yaw?-.255???bearing?-.06???????????????????????????????????spec/plan?????flight_fov_diagnosis???envelope_independent_review?????????.010m????.04m??stop reserve.03m??????XY???
- v27?????? ID997eb6e5-bdda-4abc-88c2-ecb252319b4a?run_20261112_070718_ea5ea8b0?#22925131???failed/score=null/34.857s????????????1ee83ca7f52a5e1e60de63f778e7caae939b9d4645f510d064bfc57f3016fd78?????????????ZIP94c99826fb88b7dad20ffa513f20e316709fb9a62efd41880c58f1951f9998ea?
- v27??rise.151669001m???.000058400m???????2.842s????+1mm???path.303491994??.30??+.05?vz?+.0461????-.0539??????????????????????????????/??gzip?SHA/453odom/191cmd??runs/analysis/20261112_vertical_v27/???docs/vertical_v27_response_analysis_20261112.md???vertical_v27_997eb6e5.json???????
- ????trim??spec/plan??????bounded_vertical_runtime???+.10????.15???.025/1s?.030/1s?-.010/.3s??????10s????/45s?/.30m??????PID?
- ????mission??FOV?????????.5853m??????1.2m??????????docs/flight_fov_diagnosis_20261112.md???????????
- ?????runs/analysis/20261112_motion_evidence/v26_v27_motion.png?PDF????????????????
- ????exec session67835????tunnel6233??127.0.0.1:63974?v26receiver99006???v27receiver76759???3600???functions store v27_receiver??????????/???????????????????????????????

## 06:12 最新：首个官方地面点10分

- v25 已真实终态 **success / score=10.00 / durationMs=275179**；详情ID `c4202874-407a-4361-844a-0f95ba7b1d7f`，runId `run_20261112_060713_501f412f`，编号 `#86630201`。
- 官方UI赛后诊断明确：`ground_0 到达并计分 +10`、`总分10`；来源 `runs/evidence/20261112/single_point_v25_list_final.json`、`runs/evidence/20261112/single_point_v25_official_success.png`。
- raw `runs/evidence/20261112/callback_single_point_v25.jsonl`；62事件、885764B、launcher_exit0；独立分析由evidence_refresh收尾。父代理新目录 `runs/analysis/20261112_single_point_v25/`。
- 实际首点ground_0：三帧各自曝光/前序pose、在线一致被困人员候选；一次识别发布后收到原始String `ground_0`；实际末位 `(1.1131353,-1.3556102)` 停车并一次到点声明。ROS发布返回、原始status与官方+10证据各自保留。
- source `ground_single_point.py` SHA `902424ca5ba3ec208a0028645d377f4ce92ff77e5ee56205c87fac95a9b349cb`；ZIP `dist/inspection_single_point_v25_src.zip` SHA `6c3f39642065ee14b07775a37fa1b9eac6dce91a37977382b337ad27858bbf1e`；SH SHA `79d9cf1f4e980599c9e2ee8cae061a8ecc9828678aa92cac2c652fcb6fef8c5f`。
- 本局没有发送全任务完成消息，仍自然结算为success/10分：因此“没有完整结束标志就必然取不到任何成绩”的旧推论已被本局反证；不推定任意退出都能结算，也不把首点10分称完整巡检成功。
- v24b原始结果仍failed/null/128875ms。真实根因：相机header约3Hz，wallgap最大.384845s，而新增相机活性门.20s；修后相机独立.50s、重复/倒退曝光不刷新arrival，odom/scan .20s与原曝光prior .05s不变。43作者专项、18独立相机/传输、13边界、125解析范围通过，详见相机修复与v24b独立报告。
- 当前任务继续：route_controller设计覆盖60合法三点顺序的连续地面走廊（无生产改动）；mission_bridge在完整控制器分离水平/上下包络（不影响v25冻结包）；evidence_refresh独审本局。正式完整视频和正式PDF仍待完整平台任务证据。
- 网站仿真中心、AI开发助手、算法解析已实际操作，均提示暂未开放，详见 `docs/platform_tools_availability_20261112.md`；本轮复查既有技术求助仍0评论。

以下为历史状态，应以本节最新官方证据为准。


# 晟功杯全域巡检：2026-11-12 续接卡

## 05:27 后最新状态（优先读本节）

- **v23本队有限观察路线完成，官方完整成绩未取得。** ID `9fdfe2de-a2f6-4ea5-8f62-74ff099ed85d`，runId `run_20261112_051547_2766d9ab`，权威 `route_v23_list_final.json` 为 failed/score=null/171.756s。本队 FINISHED `route_observation_complete` 99.806s/exit0，60事件、984轨迹、2208命令、7JPEG无缺口。首抽ground1,4,3，三CAPTURE原图曝光98.724/99.058/99.391和独立真实prior齐。两腿静稳误差1.077/6.629mm，末转后11.109mm；固定候选地图四scan共1440束相容。详见 `docs/platform_ground_route_v23_independent_20261112.md`、`runs/records/ground_route_v23_9fdfe2de.json`。
- **三帧冻结视觉候选一致。** route_controller独立运行全部12参考图的固定matcher，三帧均只保留trapped_people且世界中心与本局首ground1区域相容（约3.2cm整体偏差）。不是官方识别接受，不固定下一局类别。输出 `runs/analysis/20261112_route_v23_matcher`。
- **生产异步worker已落地并待最终独审。** `ground_matcher_worker.py` SHA `2362025e5a3f013cc6bb124c190e9b8cbaa303675c48ac8347aa5cbdf6697ed1`，16 worker+32matcher专项通过；本机真实单图226tick/19.999Hz/max53.42ms。文档 `docs/ground_matcher_worker_20261112.md`。本机benchmark不等于平台ROS实时验证。
- **单点接线由compact_probe_review实施中。** 新独立 `ground_single_point.py`，不改旧route默认行为；真实三帧/firstID/epoch/pose绑定，点外识别→有限入区→一次声明，保留recognition_status原文，transport不当官方接受。首版仅ground0/1/4满足既定3m及净空合同才推进，其他只观察/识别不伪声明。尚未打包实测。
- **用户要求三个辅助工具已再实际操作。** 05:19–05:24：仿真中心加载iCAN场景、AI助手代码模块发送接口问题、算法解析将TODO换实际只读图像/SIFT检查代码并运行，三者仍弹“测试阶段·暂未开放”，预计日期虽是今天实际仍拦截。未绕过限制，继续练习测评。

## 05:18 后最新状态（优先读本节）

- **v22已终结failed/null/166.037s**，ID `fc669360-a189-4dca-b804-19166267589f`，runId `run_20261112_050555_42cc6111`，编号`#20243600`。首ground3，两腿已完成(.591039,.000069) / (.586491,-1.015232)，误差8.962/16.670mm；末转新增6.745mm平移，末误差20.953mm触原20mm保护，CAPTURE0。报告`docs/platform_ground_route_v22_independent_20261112.md`及raw/independent分析/metrics均已齐，台账已校验。
- **本局真实视觉已识别候选**：冻结matcher在v22最后真实UAV原图接纳被困人员，100inlier，world中心(1.122855,-1.917171)，距候选ground3中心约8.12mm，与本局首ID相容；结果`runs/analysis/20261112_route_v22_matcher`。这是离线本局几何候选，未在线发送识别/到点、未得官方分，不能写死下一局类别。
- **v23已提交并待实测**：所有腿提前零请求10→2mm，新增每次普通DRIVE_SETTLE后累计path>10mm硬停（PAUSE不重置），原20mm航点/净空/所有其他门与预算不变。78专项通过。包SHA `64ba09f3430869e596a4b752f129ca002ea06ec13796ced7474bf06878db4bb4`，脚本SHA `24d223fc39abc486742696a5b83f31942e03280862b8478f7f8f1752243c8726`；源码route SHA `5ab108c4128db8b32c18cee084fce636e8ce80700f18dd37e468006a2a58b945`。提交`route_v23_submit.json`、raw`callback_route_v23.jsonl`；仍未包含matcher/worker，没有发送识别/到点/完成。
- **异步worker原型已独审并正移植生产**：route_controller正在新增独立`ground_matcher_worker.py`+tests，先红后绿，不碰control/vision/profile。避免Pillow新依赖，使用已有cv2读取验证原图。prototype13/独审8通过，最终benchmark289tick/约20Hz/max51.556ms、spawn隔离；不是平台实时证明。compact_probe_review正在完成首点识别→入点→一次声明设计doc，evidence_refresh准备新局分析/合同接线检查。


## 05:08 后最新状态（优先读本节）

- **v21已终结failed/null/71.162s**，ID `d734af08-1c3e-4e3e-ab29-55d71e20df0b`，runId `run_20261112_045635_0d5b0afc`，编号 `#68443203`。首ground4，小斜段实际前向.150709m、cross+.0201294m触发原硬停，未到观察位。真实高档位移方向约-.00762rad，bodyyaw~-.11827，相差+.11065；停车后仍有差异、无倾斜/惯性充分解释。单局不能断言世界坐标控制。证据 `20261112_route_v21_independent`、`runs/review/20261112_route_v21`，报告`docs/platform_ground_route_v21_independent_20261112.md`，台账已校验。
- **v22已打包并提交练习，待结果**：只将ground4外观察位(.35,-.05)改(.35,0)，直行后停车原地朝板；候选板仍完整在视野、中心至板净空.761988m，原预算内。其他4点/控制/guard不变。52受影响回归通过。包SHA `f64e9c9b059b489c1c22b3ba5bb9665761616bbac0eb5ed6d41f92620010003a`；脚本SHA `995523370fec423799bb5c385d8ac53cfa933f11e68347f4dfbab86b982a7ec0`。提交`route_v22_submit.json`，raw`callback_route_v22.jsonl`。不凭submitted或exit0宣称成功。
- **v21控制律源冻结** `5dff1b80246f81863d5f3ab5c3f306c6085efca5fe31e48f4565764453b42673`，仅去近端.03骤降。v22计划源 `c2805ab778833df505c68d65eadb3f8c5e2dd9865ac3de9af2a8fc4d07c33913`，其他源码与v21上传保持不变，旧dist不覆盖。
- **matcher异步原型在独立runs开发，尚未进src/平台**：`runs/analysis/20261112_ground_matcher_worker_dev`。本机spawn进程2.6–3s匹配时主循环约20Hz，保留原图stamp/pose/K/模板hash/job/epoch，pending仅最新1帧。独立审查发现set_context对已有latest result的epoch失效未更新及输入关联校验不足，route_controller正修；不能把benchmark当ROS实时验证。compact_probe_review正在设计首点识别→入点→一次声明，不改src，须有本局真实视觉与位置证据。


## 04:54 后最新状态（优先读本节）

- **v20 第一真实观察路线已终结 failed/null/124.666s**，ID `724cd86f-6052-4dc8-a21e-f16d5136af7a`，runId `run_20261112_044320_9ff8afba`，编号 `#68277819`；`route_v20_list_after_callback.json`权威。首抽 ground_2，第一腿实到(.5911686,.00005794)，航点误差.00883162m、13.697s；第二TURN 45.0075s后停止，yaw~-1.50283、距目标差.06306rad。无PAUSE/resume0、两路BE高频有效；没有第二腿/最终图/识别/得分。
- **根因已用真实命令定位**：route在误差<=.08时minimum从.35骤降.03，P请求≈.047→.038，18.064s只转.014875rad；较大请求时实际≈.05535rad/s。v21正在由compact_probe_review最小删除此骤降分支，保持原.35最低/.5最大、窄角窗及45/100时限不变。真实残差测试先红后绿；理想1:1夹具会跨窄窗，保留原硬停回归，另显式使用.15响应作逻辑例。尚未上传，旧dist不覆盖。
- v20证据已完整重组到`runs/analysis/20261112_route_v20`，raw原样，独立`runs/review/20261112_route_v20/review_v20.py`与metrics；报告`docs/platform_ground_route_v20_independent_20261112.md`。621轨迹/1342命令齐全，start/firstleg共720束与固定候选world相容；两路BE的header .02s，最大入口gap约.026/.030s。台账`ground_route_v20_724cd86f.json`已校验。
- **matcher已独立修好并冻结**：32专项、9独立审查通过；SHA `1b9c9b3cd658f5192307161dbbe21954968e9680c98424250d2d0836e1dc710d`。真实5图仍[1,1,0,0,0]，全部60对保留；跨相机平面错误反例已拒识。文档`docs/ground_plane_matcher_20261112.md`与review、manifest均就绪，尚未接ROS/上报/上传。route_controller正在新v20原图做相同固定阈值离线分析，不借上一局类别。


## 04:39 后最新状态（优先读本节）

- **v19 实际已终结 failed/null/146.274s，官方完整任务有效成绩仍为 0 局。** ID `17f62d65-e85d-46ed-bd78-be604d0b0de3`，runId `run_20261112_043048_33aebe21`，编号 `#31369881`；权威 `speed_v19_list_after_callback.json`。回传 launcher_exit=0，有限运动投影 .200286m、两个停车静稳通过；高档 .10 请求峰值 .059546m/s，触停后路径 .000773m。但 UAV odom 前序不新鲜，CAPTURE=0、reason=`approach_capture_wall_time_limit`。报告 `docs/platform_ground_speed_v19_20261112.md`；raw 和 analysis 路径见报告，不能称观察或计分成功。
- v19 上传仅冻结v18源码包新增v19模块，未混入matcher。1095全套与9独立反例通过；ZIP SHA `57106a6b23a5ce21b1c6b5e6c3b37a14b15ad09147a04cc87f0b1fa14bf3761a`，SH SHA `7856a1282d8f5b8caf00113477f983b00c21cf5c30a69ba632129e264f86dc3c`，旧包不覆盖。
- **下一版正在实现第一外观察路线的最小订阅修复**：`compact_probe_review` 负责把两路 `/odom`、`/uav/odom` 显式 BEST_EFFORT + callback timing，白名单仅扩允许这两个 topic/value；默认/旧v19行为不变。修正计划与运行时预算文案一致，不扩大既定300global/190drive/单turn45累计100/capture10。尚未冻结、未上传。高档短脉冲不能当持续资格，首路线仍须守全部原净空/位移/角度门槛。
- **网站三个工具已再次实际操作，仍未开放**：04:28–04:31（UTC+8）仿真中心 iCAN加载、AI助手查询、算法解析OpenCV运行均弹同一未开放提示。算法页只有TODO模板，未得到运行/AI回答。详见 `docs/platform_tools_availability_20261112.md`。用户要求已执行，不重复索要材料或确认。
- **纯感知matcher已新增，尚未接控制或上传**：独立 `ground_plane_matcher.py`，31专项、1126全套通过；对固定真实5图得到v16b营地和v17被困者候选，v18三图拒识。`evidence_refresh` 发现跨相机平面的投影反例，`route_controller` 正在先红后绿补正深度/原图凸投影门；未完成独立复核前不能声称已通过审查。旧vision/profile不变，模板未自动打包。


## 04:12 后最新状态（优先读本节）

- **v18 有限观察动作真实通过，官方整场成绩仍未取得。** ID `5038ac33-4026-490a-8c1e-5b682b2a2f06`，runId `run_20261112_035930_d1431193`，编号 `#88131099`；终态 `failed / score=null / 141089ms`，来源 `approach_v18_list_final_check.json`。该包没有发送任务上报/完整结束，不能把观察动作通过改写为官方测评成功。
- 原始 `callback_approach_v18.jsonl` 已收齐 launcher_exit0；新分析 `runs/analysis/20261112_approach_v18/summary.json`：reason=`approach_observation_complete`、实测前向 `.200151226m`、CAPTURE=3、resume=0，三张不同stamp的真实UAV图已提取。/odom BE实际生效；入口最大wall gap `.039021s`、header gap `.04s`；517扫描全部immediate，late_prior=0/expired=0。抽签首点ground_0与v17不同，因此QoS影响有支持证据但单局不作严格因果证明。
- **v19 正在实现独立短距档位校准，尚未上传：** 复用v18QoS/计时及全部原`.20m/90s`预算；首`.05m`全程`.03`后实际停车≤`.01m`并静稳，再做`.10`请求的额外≤`.05m`/≤5s wall及sim有界脉冲与独立停车验证，最后慢档至总`.20m`并采三图。高档`.04m`触发停止，保留`.01m`停车；不把慢档证书当高档证书。实现代理compact_probe_review；evidence_refresh将独立审查。不改长路线、不同时改PAUSE架构。
- **感知离线新增进展：** `runs/analysis/20261112_sift_geometry_exploratory/README.md`保留48对固定SIFT结果；人工warp的v16b匹配rescue_camp_02、v17匹配trapped_people_02，具有分散内点/凸投影，但原图直接匹配无同等有效对应。不能把人工裁图当自动识别。冻结Canny文档扫描在v16b/v17无四边形，v18三帧仍在盲跑；另按固定候选外参做整幅地平面IPM，原型由route_controller执行，未改生产识别、未上传模板。

## 04:01 历史状态（已被上节取代）

- **完整官方任务有效计分局仍为 0。** v17 已真实终态 `failed / score=null / 164794ms`；ID `441364ff-c6c4-4b1d-9683-1b249b505094`，runId `run_20261112_034630_6bea29f7`，编号 `#59088134`。权威快照 `runs/evidence/20261112/approach_v17_list_final.json`。raw 已完整收尾并解析到新目录 `runs/analysis/20261112_approach_v17/`，报告 `docs/platform_ground_approach_v17_20261112.md`。
- **v17 定位证据：** /scan 本队 receive 最大 2.046ms、/odom 12.613ms、tick 1.172ms、spin_once 32.350ms；scan 入场 clock-header 全部≤0，odom 入场最大 .18s。不能称入场普遍过期，也没有百毫秒本队处理阻塞证据。后段 odom .18/.02s 供应间隙与 scan 等待 prior .096–.176s 消耗 fresh 窗口；PAUSE 反复清空静稳锚。最终仅前进 .076843m，CAPTURE=0；不等于任务完成。
- **v18 已提交练习对照，尚待结果：** ID `5038ac33-4026-490a-8c1e-5b682b2a2f06`，提交证据 `runs/evidence/20261112/approach_v18_submit.json`。只改 `/odom` requested reliability→BEST_EFFORT，depth1/VOLATILE 与其他 QoS、全部运动/同步/freshness/预算保持 v17。全套 `1079 passed / 63.94s`，JUnit `dist/odom_qos_v18_full_suite_20261112.xml`；独立源/脚本及 manifest 见 `docs/ground_odom_qos_v18_plan_20261112.md`、`approach_v18_artifacts.json`。ZIP SHA `f2f387765eb6f89b46a3558919316a3c97f40c65c15245d6a603bf85663232d3`，107155B；SH SHA `fe2643346dae8f0e155625520f1f696df7aa2937812b3a13d96bfa727657851f`，2823B。
- **本次临时接收器：** raw `runs/evidence/20261112/callback_approach_v18.jsonl`；接收 PTY 43890 / loopback64807，tunnel PTY77734（显式 `--config NUL`，公共GET已验证405，health证据独立保存）。认证 PTY67835 仍仅内存保存凭据。勿把临时通道写入文档；运行终态后停本队接收/隧道。
- **辅助模块用户要求已实际执行：** 仿真中心、AI 开发助手、算法解析三个入口均实际操作并被“测试阶段·暂未开放”拦截；不是忘记使用。详见 `docs/platform_tools_availability_20261112.md`。官方审核回复已说明“构建、运行日志不展示”，因此缺 stdout 不是源码未启动证据。
- **后续分支先审再改：** 仅 PAUSE 的独立连续静止 odom 证据可与 scan 当前许可解耦，但所有非零命令前仍验证原完整 condition；正在只读审查，尚未实现。不要靠放宽 freshness、增加预算或伪造结束标志修复。

## 03:32 历史状态（已被上节取代）

- **完整官方任务有效计分局仍为0。** 当前用户授权持续练习测试；退出码0、FINISHED、诊断collected、本地测试通过和网页0分均不等于有效官方成绩。03:23及更早节保留历史，下面v16b“409待重试”已过期。
- **v16 / v16b 均已真实终结为failed/null**，最新依据 `runs/evidence/20261112/approach_v16b_list_settled.json`。v16 ID=`e8d077bd-89ca-4b0e-8ff8-dfefbd7e10d6`，runId=`run_20261112_031225_e3fbe459`，159.174秒；v16b ID=`b4cd8cda-f442-41dc-8dd6-2a2a4c5c5484`，runId=`run_20261112_032806_33175195`，167.614秒。API failedReason原样为“流程未检测到完整结束标志（未提取到成绩）”，UTF-8核对正常。v16没有callback raw；隧道404不能推出ROS未运行或同步改善。
- **v16b实际接收凭证是submit_retry**，先前409未登记为新局。源码与v16相同；真实raw为 `callback_approach_v16b.jsonl`，525,717B/SHA`3668579cdf8cbe7bdee072a7c9a87b4d127b890eb1b48cadd436af63c8515f10`。已有分析 `runs/analysis/20261112_approach_v16b/summary.json` 未覆盖。62事件、core1/full0故partial；665轨迹/2516命令/6native完整、series源掉行0，无解析/组装/冲突；motion_log_dropped384，不能称逐回调全量。
- **运动仍未达到观察距离**：draw=`ground_1,ground_0,ground_3`，候选映射未验证；实际投影`.095774104m`，capture0，reason=`approach_drive_wall_time_limit`。55s DRIVE预算含PAUSE；真实请求DRIVE约19.170279s、PAUSE约35.840908s，第二PAUSE约34.744193s。有效轨迹区间`.03`请求下约`.004975314m/s`（wall口径），不能外推`.10`响应或线性增益。
- **迟到prior配对已在平台发生，持续fresh仍阻塞**：incoming673、immediate398、late_prior123、expired149；末trace保留12、掉509。活动期PAUSE wall85.6425s，odom simage`.22s`、scan wallage`.663291516s`；STOP边界`.713711792s`。末快照scan age2.733s包括STOP后冻结约2秒，不能称活动期2.7秒传输延迟。graph query总`.058938204s`只覆盖这些调用，尚无证据锁定ROS调度/序列化/锁/DDS瓶颈。
- **下一版v17仅补回调性能证据**：`/root/compact_probe_review`实现固定容量计时、入口年龄与QoS诊断；运动仍`.20m/.03mps/90s`，不靠放宽安全阈值消除PAUSE。`/root/evidence_refresh`接可选top-level `callback_timing`保真，不改变旧数据完整性合同。以父代理后续冻结hash、上传与真实新局为准，不能把实现中指标当平台测量。
- **独立路线最终只读审查完成**：`docs/control_review_ground_route_20261112.md`。长腿TURN_SETTLE出口重验窄yaw窗并在原预算回TURN；更紧线速配置约束慢档；最终TURN后/采图中位置与航向复核。5项定点通过，三文件hash已与实现代理一致；尚无`.10`真实响应/停车/300s时窗证据，不以本地合成三图放行长路线。
- **平台三辅助入口已实际操作，均未开放**：仿真中心加载赛事场景、AI开发助手发送问题、算法解析运行Pure Pursuit均弹“测试阶段·暂未开放”；页面预计11/12仅文案。见 `docs/platform_tools_availability_20261112.md`，不能归为测评未结算唯一原因。
- **证据报告与台账**：`docs/platform_ground_approach_v16b_20261112.md`；新增 `runs/records/ground_approach_v16_e8d077bd.json` 与 `ground_approach_v16b_b4cd8cda.json`，全台账20条。保留null分数/完成状态，v16抽签未知；不复制账号、通道密钥或签名URL。

## 03:23 历史状态（已被上节取代）

- **完整任务有效计分局仍为0。** 用户授权继续练习测试，不重复索要官方ZIP或确认；下面02:52/02:30状态保留作历史。诊断退出码、FINISHED、本地测试通过及网页显示0分均不能当有效官方成绩。
- **v16已确认failed**：ID `e8d077bd-89ca-4b0e-8ff8-dfefbd7e10d6`，runId `run_20261112_031225_e3fbe459`，编号`#14154215`。较新的`runs/evidence/20261112/approach_v16_list_resume.json`对应项为failed、score=null、durationMs=159174，更新时间`2026-11-12 03:23:06.769717+08`，原因“流程未检测到完整结束标志（未提取到成绩）”。用户页面约159.2秒/0分的显示不等于提取到有效0分；较早名为`list_final`/`list_settled_check`的running快照已过期。
- **v16诊断回传通道错误已定位**：`approach_v16b_artifacts.json`的retry_reason记录旧隧道继承了固定HTTP404 ingress，故为重试新建隔离通道。此问题解释诊断回传缺失，不足以证明ROS动作未执行，也不能把它当作平台未结算的唯一任务原因。本轮检索尚无`callback_approach_v16*.jsonl`可用于确认真实同步/行走/CAPTURE结果；不要据v16缺回传宣称同步修补有效。
- **v16b已备包，待允许重试**：`approach_v16b_artifacts.json`显示源码ZIP与v16相同SHA`38c337af70dd5185c3dcebdd3e0a3534a06ee1addd91987dc5c2c60c3ceecc47`（97,185B），仅启动脚本通道不同，SH SHA`67d45c70c1e8fa57bfccc6dfeaa36e7110274e7225e624501892dc6acaae0131`。截至此轮读取，`approach_v16b_submit.json`为HTTP409/EVAL_IN_FLIGHT_LIMIT，无新id/runId，不能记为新局已提交成功；应在v16终态后使用主代理的新提交结果更新。接收器health文件返回405/accepted=false，只表明该HTTP方法未被接收，不能当实际诊断POST已收齐；新局须确认真实launcher/probe事件。
- **v16同步版本已冻结并经定点审查**：`docs/control_review_sync_v16_20261112.md`保存4文件哈希、18项同步/共享watchdog检查及最后1项诊断统计复核；只放行同`.20m/.03mps/90s`和当前360束环境的修补练习，不放行提速/长程。真实v16b若有回调，仍用`tools/analyze_ground_bounded_approach_evidence.py`，创建新分析目录，不混入v15。
- **独立路线证据工具已完成**：`tools/analyze_ground_observation_route_evidence.py`，真实mode=`ground_observation_route`，仅该mode独享trajectory3200/commands14000，沿用三图缺失partial。已与实现代理核对`survey.approach.route`及四scan stage：start/first_leg_end/observation_arrival/final_heading；五组分析92通过，完整LaserScan fixture后新11项复核通过。说明`docs/route_evidence_analyzer_20261112.md`。当前仅兼容，路线控制器仍独立实现，必须先取得v16b反馈/后续审查再决定上传，不把设计或本地合成fixture记作平台成功。
- **最接近可用的真实运动证据仍为v15**：约−90°转向后进入前进，`.120629887m`投影后预算停机，第二PAUSE约29.75秒，CAPTURE=0。同步修补与较高档位收益均待真实新局；其完整报告与18条已验证台账见下方历史记录。当前文档不记录账号、通道密钥或签名下载地址。

## 02:52 历史状态（已被上节取代）

- 完整任务有效计分局仍为0；当前用户授权持续练习测试，无需重复索要ZIP/确认。下方02:30开发状态为历史，不能据此认为v15尚未上传。
- **v15已结算**：ID `87616485-22d2-4849-8b8d-0b524d63c4b3`，runId `run_20261112_024853_b133c7ab`，编号`#73170413`，终态以`approach_v15_list_final.json`为准：failed/null，169.902秒。draw=ground_1,ground_2,ground_4；真实转向至约−88.60°后静稳并前进.120629887m，未达.20m，reason=`approach_drive_wall_time_limit`，capture_count=0。
- **下次先修扫描/里程计同步**：55s前进预算中实际DRIVE命令区间约24.139203s、PAUSE约30.904403s；第二PAUSE持续29.754981s直至预算停机。日志`odom_stale_or_invalid`与`scan_anchor_unavailable`交替；scan_anchor最终null。scan回调只配当时history，后到prior odom不重配，与延迟回调缺口相容。保留prior≤scan/skew≤.05s/freshness/移动及停车边界，验证pending scan重新匹配，不靠放宽门禁或单纯加时消除症状。
- 有效234轨迹区间请求.03，观测投影.118582874m / wall23.790426s=.004984479m/s，sim23.44s=.005058996m/s。不能把全阶段.1206m/55s当恒速，不能推断线性增益。先在短距离验证新档位与停车，再确定长路线预算；`.10/.20`仍是未验证请求值。
- raw=`runs/evidence/20261112/callback_approach_v15.jsonl`，531,068B/SHA`0e205c23a3ed85392c7cff019788a644da1eb2f22261f8efe16719bc862a8a8f`，62事件、launcher_exit0。主分析`runs/analysis/20261112_approach_v15/summary.json`为partial；702轨迹/2412命令/六native，trajectory/commands掉行0，motion_log_dropped239，无full snapshot。stage scan只有start/after_turn/mid_drive，未到after_drive/CAPTURE。
- 六native为双相机first/last/last_associated，不是三张CAPTURE；双last prior pose无效。UAV末图可见地标与斜视照片，类别、识别上报、正式点号映射仍未知。没有正式到点/得分证据。
- v15控制与SH冻结值保持不变，上传前主代理967全套与独立43项通过。共享离线解析器已补整组capture丢失→partial：按真实capture_count要求完整UAV native，51新旧分析专项通过；修后另存`20261112_approach_v15_capture_check`，不覆盖首次分析/raw。count0不凭空要求3图，也不等于任务完成。
- 本局总报告`docs/platform_ground_approach_v15_20261112.md`；独立速度/停车/几何`docs/approach_v15_motion_review_20261112.md`；台账`runs/records/ground_approach_v15_87616485.json`，全台账18条/0重复/files_verified=true。
- 下一路线设计已存`docs/superpowers/specs/2026-11-12-first-ground-outside-observation-design.md`；须结合本局同步缺口、新档位实测与停车证据定参，不将设计当成已跑通路线。

## 02:30 历史开发状态（已被上节取代）

- 完整任务有效计分局仍为0，用户授权持续练习测试，无需再索要官方ZIP或重复确认。练习失败标签不得伪造成成功。
- v13实际首点为ground_4，目标约−34.56°，18.33秒TURN后实测到角停车，末残差约1.369°；不是原来ground_2的−90°成功证据。v14实际前向投影约.020213832m后停车，末偏航变化约.00123rad。两局平台终态均failed/null（65.001s、43.142s），原文件/完整台账见本卡最新报告链接。
- v14约2cm移动后，首末各360束scan仍与固定公开候选world相容。工具新增显式 `--expected-mode ground_short_drive`，结果 `runs/analysis/20261112_short_drive_v14/scan_candidate_geometry.json`，报告 `docs/ground_v14_scan_geometry_20261112.md`。配对仍是静稳端点+40/−30ms，不把微米计算残差当定位精度。
- 当前实现代理 `/root/compact_probe_review` 正在独立 `ground_bounded_approach.py` / `run_ground_bounded_approach.sh` / 专项测试中实现：当局首点转向→归零静稳→实际前进.20m→静稳3帧观察。TURN≤45s（min .35/max .5），DRIVE≤55s，global90s；全程路径/起点位移各.30m，局部scan运动/停车储备共.06m，全环门槛.641m；仅地面cmd_vel，不发识别/到点/完成。尚未冻结或上传，勿把部分测试结果当最终。
- 设计：`docs/superpowers/specs/2026-11-12-ground-observation-route-design.md`。实现首批19项通过，仍在做采图/看门狗/启动脚本必要边界与最终回归。
- `/root/evidence_refresh` 已备 `tools/analyze_ground_bounded_approach_evidence.py`，显式mode为`ground_bounded_approach`，仅此mode轨迹cap1200、命令cap5000，允许capture_0/1/2；实际模块events/scan fixture仍待验证。旧模式边界不放宽，有掉行应partial。完成源冻结后followup该代理复核。
- `/root/ground_next_path` 正在做v13真实ROI与12公开参考图的CLIP图像特征近邻：全4×12已算完，最近邻多为天空/飞鸟照片，未显示同一照片关系，不能报类别；结果在 `runs/analysis/20261112_survey_v13/image_neighbors/`。固定文字提示四旋转仍全部低margin拒识。
- 当前平台会话仍为PTY67835。v14终态已保存`short_drive_v14_final.json`；当前没有在进行的练习。下一局尚未新建callback通道，待源冻结/审查后再创建，不能重复用旧raw混局。
- 最新既有全套基线917 passed/48.98s，`dist/geometry_v14_full_suite.xml`，时间02:25:09+08，不包括之后开发中的approach。README和提交指南已去除“仍只读/未在平台运行”的旧现状，未改正式profile的verified状态。
- 未来垂直合同设计：`docs/bounded_takeoff_contract_next_20261112.md`。初始下视激光三束inf、真实TF下移.07m，不能当上方净空；有限起飞需明确静态地图柱假设，尚未实现或测试。

本卡用于恢复当前练习与证据分析，不含账号密码、访问令牌或签名下载链接。平台练习与官方任务成绩必须分开记录。

最新已结算为 v14 短直行：平台 failed/null；实测投影约2.02厘米后归零，仅控制验收，详见下方v14。当前仍无官方任务得分。

## 上一轮：v13 地面原地观察练习

- 详情 `e8cd8e60-fddb-4fdc-9b65-25349077328a`；runId `run_20261112_021236_269a9fac`；编号 `#19225516`。终态来源 `survey_v13_final.json`，**failed / score=null / 65.001秒**；不能因target_yaw_reached标记比赛成功。
- draw=`ground_4,ground_3,ground_0`，候选方向−34.563°，末轨迹−33.194°，残差.023896075rad在.025rad容差内；累计角.578939357rad，最大位移5.971mm，resume0。实际TURN1.326136→STOP首零速19.656201，历时18.330065秒。
- v12目标约−90°、v13约−34.56°；抽签与目标角不同，不能证明90°观察已完成，也不能作最低角速/预算的因果比较。
- raw `callback_survey_v13.jsonl`，244,906B，SHA652fbf6fd2152d71e3ce211de4715054716d80b5fca98c1c80f3fe0676decc51；19事件。analysis `20261112_survey_v13`：collected诊断，210轨迹/478命令/四native完整、掉行0；核心362非零+112零速只表示发布调用。
- ZIP `inspection_ground_survey_v13_src.zip` 79,204B SHA8d175fab024decff6ba275fe7e8b0bab4437627c1ba2dc029e9eb583cdc65ac4；SH `run_ground_survey_v13.sh` 2,900B SHA0e812b18e9a0fe83c9071d33682efb79f255fdfa2308353b40dc878957827040，均在dist/，保持上传版。
- 报告 `docs/platform_ground_survey_v13_20261112.md`；台账 `runs/records/ground_survey_v13_e8cd8e60.json`。

## 最近完成运行：v14 短直行

- 详情 `2aeffd1d-eb67-4c2d-a8e2-a5fc8b9f065b`；runId `run_20261112_021755_cef799e3`；编号 `#84358854`。终态以 `short_drive_v14_final.json` 为准：**failed / score=null / 43.142秒**，更新时间 `2026-11-12 02:20:31.217261+08`，原因未检测到完整结束标志/未提取成绩。早期queued已过期。
- raw `callback_short_drive_v14.jsonl` 210,759B，SHA3929b4de07037746136e621517bf2060bbd2ab760116df21e0f737e3fc3078bf；18事件，launcher_exit0。
- 独立新mode入口 `tools/analyze_ground_short_drive_evidence.py` 已将真实raw直接解析到 `runs/analysis/20261112_short_drive_v14`。kind=ground_short_drive_analysis；保留mode/DRIVE状态与原字节，不改raw或猴补。92分析相关回归通过，见 `docs/short_drive_analyzer_20261112.md`。
- 诊断collected：73轨迹/193命令/四native完整，掉行0，无错无冲突。实际DRIVE1.120600→STOP首零速5.355516，随后持续零速；核心84非零+108零速，resume0。
- drive投影实测.020213832m，max位移.020214434m，reason=short_drive_distance_reached。用compact+真实原点/初始yaw独立重算末投影.020213812m。STOP前轨迹5.333974投影.019929512m，STOP后首轨迹5.435820投影.020088812m；保留.021542/.080304秒边界差，不填造精确停点。
- 图像仍不足以宣称目标识别：地面末图近均匀绿色；UAV末图可见窗框、地标及画面边缘的裁切图块。四图曝光关联有效只确认先前位姿，不确认外参或类别。未发布计分消息，score/任务完成仍未知。
- ZIP `dist/inspection_short_drive_v14_src.zip` 83,210B SHA808cb3ed4b0f86ab7c4a338eb4b5557bb2f7eb64f05211496d32b3215069d92a；SH `dist/run_short_drive_v14.sh` 2,800B SHAafdbfe444b01ac503a93c4dd5b6d84dbd6c28408712e1ed1d3ca31e5e5011c59，已复核保持上传版。
- 报告 `docs/platform_short_drive_v14_20261112.md`；台账 `runs/records/short_drive_v14_2aeffd1d.json`。全台账17条、0重复、files_verified=true。
- 后续独立bounded approach正在设计；分析器先等明确新mode/schema，不猜测重标已有raw。v13四旋转CLIP结果仍全部低margin拒识，不能替代官方目标识别。
- compact已给新mode最小schema：`ground_bounded_approach`，`survey.approach`保留真实元数据；cap1200轨迹/5000命令，增加capture_0/1/2图像phase，scan_records保留原payload。新入口 `tools/analyze_ground_bounded_approach_evidence.py` 已准备，旧survey/shortdrive容量与phase限制不变；8项新回归后，七组100分析检查通过。当前仅最小schema兼容，尚待新控制模块真实events/精确scan fixture复核，不据此声称平台运行成功或scan关联有效。

## 上一轮：v12 地面原地观察练习

- 平台详情 ID：`2bb54a18-08ec-4d1f-a5f4-893600919dff`；runId：`run_20261112_020204_e8d882d5`；编号 `#32837877`。
- 终态来源 `survey_v12_list_final.json`：**failed / score=null / durationMs=80194**，更新时间 `2026-11-12 02:07:06.812303+08`。没有单点得分或完整任务证据。
- raw：`runs/evidence/20261112/callback_survey_v12.jsonl`，267,908 B，SHA256 `c1597a0d07428319b19e62db570f442659b89cac6f5674716d3d67e78feb10fc`，20事件，launcher_exit=0。
- ZIP：`dist/inspection_ground_survey_v12_src.zip`，79,112 B，SHA256 `f265d5e17518609ea955cddbf80fdee59b20129692406d140ea3bf0c9c2decf7`；SH：`dist/run_ground_survey_v12.sh`，2,847 B，SHA256 `6126a5b30c74c6003f96d4ae87bf76160bd7dc96c2955792ce0313675315c377`。上传包保持原版。
- analysis：`runs/analysis/20261112_survey_v12/summary.json`，collected只表示诊断证据完整。268轨迹、661命令、四张native800×800及四张thumbnail完整，掉行0，无parse/diagnostic/conflict错误。
- 探针快照28.174秒，draw=`ground_2,ground_3,ground_4`。实际命令TURN从1.130779开始，两次scan过期进入零速PAUSE，6.424230及15.728622恢复同一目标；26.131088按原25秒预算归零。resume_count=2，未重置预算。
- 累计角约65.24°；末yaw约−65.23°，离候选−90°仍24.77°；最大位移4.37mm。候选点号映射仍未确认，不作类别答案。
- 双相机首图stamp2.885→本源prior odom2.88（.005秒），末关联30.218→30.2（.018秒）；外参/投影/识别未验证。
- 响应分析 `runs/analysis/20261112_survey_v12_response/response.json` 与同目录PNG/SVG。命令.45–.5档→.25–.3档时，观测header姿态角速中位从.08346降到.011405rad/s；末3墙钟秒中位.009962rad/仿真秒。无因果增益/死区结论；恒速20仿真秒算术情景不可当墙钟预测。
- 台账 `runs/records/ground_survey_v12_2bb54a18.json`；报告 `docs/platform_ground_survey_v12_20261112.md`。
- 下一版显式.35最低请求角速/45秒转向/60秒总预算的只读审查已通过151相关测试，审查 `docs/control_review_profile_20261112.md`；保持.2age/.06位移/1.8rad/最多3恢复及到角或越过锁停。参数联合变化不能拆成单一因果收益；长程证据cap500轨迹/1200命令可能掉行，必须如实保留。
- 主代理已提交v13：`e8cd8e60-fddb-4fdc-9b65-25349077328a`，raw=`callback_survey_v13.jsonl`，manifest=`survey_v13_artifacts.json`。当前卡不预设该局终态，须以主代理保存的后续快照为准。

## 上一轮：v11 地面原地观察练习

- 平台详情 ID：`1da890da-6b9e-4701-8efb-934da9850219`；runId：`run_20261112_014851_4596f50c`；平台编号 `#84264286`。
- 终态：`runs/evidence/20261112/survey_v11_list_final.json` 第一条，**failed / score=null / durationMs=59393**，更新时间 `2026-11-12 01:52:21.713795+08`，原因仍是“流程未检测到完整结束标志（未提取到成绩）”。
- 源码：`dist/inspection_ground_survey_v11_src.zip`，78,135 B，SHA256 `dff72b3926ee14e161ec2165e2d9908ecea207bb4ee4b7ca53d1fcd20a39db81`；脚本：`dist/run_ground_survey_v11.sh`，2,845 B，SHA256 `022494c64d58e7a16062688afbd3aec908f71d0af29b79bf8587064f58b461e0`。产物 manifest 已复核，两文件保持上传版本不变。
- raw：`runs/evidence/20261112/callback_survey_v11.jsonl`，220,299 B，SHA256 `e926f696db4d4ff2939dc666e63eac334bc1e0b0f112b14bf1813ecd2465a370`，18 行，末尾 launcher_exit=0。
- analysis：`runs/analysis/20261112_survey_v11/summary.json`，**collected 只表示诊断证据完整**，不是比赛成功。独立轨迹85行、命令228行、native四图及full snapshot完整，无parse/diagnostic/conflict错误。
- 探针快照9.190 s；draw `ground_2,ground_1,ground_4`；TURN期累积26.5331°，最终odom yaw约−27.15847°（含停止前后观测变化）。候选目标约−90°，未到角。
- STOP发生于7.1874 s：scan墙钟接收年龄.2481268 s超过.2 s门禁，odom仍fresh；随后数据恢复仍不重启。本局正确执行了永久停止设计。
- 双相机首图stamp2.815关联各自prior odom2.8（.015 s）；last_associated图stamp11.483关联各自prior odom11.48（.003 s）。四图曝光关联均通过接收证据检查，但外参/投影/识别仍未验证。
- 核心快照119非零、108零速只表示发布调用。未发识别、到点、飞行、照射或完成消息，尚无单点或完整任务得分。
- 台账：`runs/records/ground_survey_v11_1da890da.json`；报告：`docs/platform_ground_survey_v11_20261112.md`。台账验证14条、0重复、files_verified=true。
- 下一版PAUSE已冻结并独立只读复核通过：143专项passed，记录 `docs/control_review_pause_20261112.md`。允许进入新的有界地面原地观察练习，尚非完整任务/计分提交；主代理正在运行修后全套并准备独立v12通道。不得覆盖v11/v10或旧包。

## 上一轮：v10 地面原地观察练习

- 平台详情 ID：`fbcdc168-a118-4729-8deb-bf5a4ecfb759`。
- runId：`run_20261112_013814_4b96e433`。
- 最新有效状态来源：`runs/evidence/20261112/survey_v10_list_after_run.json` 的 `data.list[0]`：**`failed`、`score=null`、`durationMs=96426`**，原因“流程未检测到完整结束标志（未提取到成绩）”，更新时间 `2026-11-12 01:41:27.111118+08`。较早 detail 的 `submitted`/空 runId 与早期 list 的 `queued` 均已过期。
- 源码：`dist/inspection_ground_survey_v10_src.zip`，77,417 B，SHA256 `97d50a28069998c2341600136ac78d856821d3bb39bf17e3838cfdca0770ac66`。
- 脚本：`dist/run_ground_survey_v10.sh`，2,857 B，SHA256 `bc1ac8322db9c8437fe59aacc44c23626ff278e0ce12d4c8cbd8146ee0bc8e9a`。
- 产物台账：`runs/evidence/20261112/survey_v10_artifacts.json`。
- 此包仅发布有界 `/cmd_vel` 角速度，线速度恒为 0；不发送识别、到点、飞行、激光、任务完成或计分消息。
- 地面点号到静态候选地图的映射仍为 `candidate_unverified`；它只用于观察方向，不能作为目标类别或裁判答案。
- **回调文件沿用旧通道名**：`runs/evidence/20261112/callback_survey_v9.jsonl` 只归属本次 v10，绝不能记为 v9 实测或与其他局合并。
- raw 304,715 B，SHA256 `88a450ae27ca8dc63bdc376dfe0ea4fe99154623da76c40d05c639eb0fa649f9`，44 行且末尾 launcher_exit=0。已由主代理解析到 `runs/analysis/20261112_survey_v10/summary.json`，不要覆盖或重复生成同一目录。
- 解析状态 `partial`：独立 compact trajectory 234 行、actual_commands 553 行、4 张 800×800 native JPEG 均完整；无 parse error、无 conflict、生命周期无缺项。冗余 fallback 的 `survey.trajectory_10hz` 超过 64KB，显式记入 evidence_errors，因此总体仍不能标为 collected。
- 探针时长25.651 s、draw为 `ground_2,ground_0,ground_3`；候选目标约−90°，最终odom观测约−8.51967°，没有完成转向。STOP时仿真clock−odom=0.26 s，正确触发0.2 s门禁；双相机末图缺少有效先前位姿关联。
- 本轮台账 `runs/records/ground_survey_v10_fbcdc168.json`；报告 `docs/platform_ground_survey_v10_20261112.md`。台账验证为13条、0重复、files_verified=true。
- 此处记录的v10接收服务已属历史，不据旧PID/端口重启或停止进程；后续通道归属由主代理核对。

## 续接后的执行顺序

1. 主代理完成修后全套检查，再用新版本包/独立诊断通道上传有界观察练习。打包后核对审查hash与manifest，不覆盖任何旧包。
2. 逐局检查PAUSE实际回传：临时过期先零速；只有稳定、新鲜、安全后才能继续原目标；非法/越界/超速/总时限/总旋转永久锁停，恢复不能重置预算。新补丁已在回调锁存瞬时有效数值越界，仍须检查真实平台证据。
3. v11已消除v10冗余大字段回传问题，保持compact/native分开传输。逐张查看真实图像；曝光关联有效也不能据此声称目标识别或任务得分。
4. 在门禁保持有效的前提下继续完整观察；针对真实图像验证单点识别并取得第一项平台分，再扩大至三个地面点、穿窗、空中照射与降落。
5. 正式视频与技术文档只采用真实平台证据；现有草稿和开发演示仍不能当作完整任务录像或有效成绩。

## 独立控制与回传审查（v10）

审查已读取 survey、bounded adapter、telemetry 与对应测试；已核对 v10 ZIP 中三个源文件与审查时工作区逐字节一致。

### 已复现的问题与 v11 复核

1. **活动期非法地面传感器样本未立即锁存 STOP。** `bounded_ground_probe.py` 的 `update_odom`、`update_scan` 异常分支只清当前样本/记录 reason。如果非法样本后、下一次 `step` 或 watchdog 检查前出现合法样本，`GroundSurveyGuard` 会继续输出旋转。最小复现：进入 TURN → NaN odom（或 NaN scan）→ 下一合法样本 → `step` 返回 `angular_z=-0.15`。应仅在活动 PULSE/TURN 中立即 `stop`；WAIT 可继续等待有效证据。回归测试应覆盖“非法→合法→下一 step 仍为零速”。本条为代码缺口，尚无证据证明它在 v10 实测中触发。
2. **非法 UAV odom 后仍可关联旧曝光位姿。** `ground_observation_survey.py` 的 `update_uav_odom` 异常分支未清 `uav_odom_history`。已复现：合法 t=10 → wrong child_frame t=10.01 → image t=10.02，曝光 `association_error=None`，仍关联 t=10。仅影响曝光几何证据，不控制 UAV。应清该源历史/保留无效原因，待合法源恢复后再建立关联。

v11 冻结候选源码已修复上述两条：PULSE/TURN 中非法 ground odom/scan 立即 STOP；两源曝光历史各自清理并记录 source_error，合法非倒退样本到来后恢复。原 v10 包保留原行为。v11 新增 `last_associated` 原图阶段，离线分析器已补接并维持真实 header/pose 再校验，含有效和无效关联两项回归测试。

**v11 尚有独立复核发现、已交主代理修复的控制缺口：** 活动期“数值合法但超速/位移越界”的 odom 只在 step/watchdog 查询当前样本，能被下一个正常样本覆盖。已复现先 yaw_speed=.56 后 .1，下一 step 仍 TURN/−.5；位移先 x=.061 再 x=0 同理，虽然 max_observed_displacement_m 已为 .061。需在活动 odom 回调对超速、位移（并涵盖 tilt）立即锁停或保留可触发的峰值。不要把已有测试全绿当作该缺口已修复。

该缺口在拟v12的PAUSE补丁中已修复：回调锁存物理超限，TURN/PAUSE累计角与到角也在回调锁停。复核又发现无锁PAUSE赋值可覆盖并发硬STOP，控制代理已加RLock并以并发回归先红后绿修复；独立143专项通过。v11已上传包不回写这些改动。最终审查hash与minor旧reason保守清一次驻留边界详见 `docs/control_review_pause_20261112.md`。

### 已核实的边界

- 仅 `/cmd_vel` 有实际发送路径，其他已声明输出保持静默。
- v10 survey请求角速度上限.15 rad/s，v11有界实验上限调整为.5 rad/s，观测角速>.55 rad/s需停机；25 s单次转向、1.8 rad累计旋转、.06 m总位移、.2 s新鲜度边界保留。候选半径.447 m的尖端线速计算不是官方尺寸或整机安全认证。
- `/clock`、odom、scan 使用接收墙钟与仿真时间双重有效性门禁；独立 watchdog 在 ROS spin/graph 查询阻塞时仍尝试发送零速。
- 图查询不持有命令发布锁；慢循环后从当前墙钟调度，不补发追赶突发命令。
- 回传在停止后进行，core、survey metadata、series 和 native JPEG 分开；分片有索引/总数/SHA256，分析器显式保留缺失与冲突。
- 回传具有60秒发送调度预算，但从事件构造完成后才开始，urllib timeout也不是整个函数的硬墙钟期限；材料不能称整个report_final严格限时60秒。网络/编码失败可能造成不完整证据，不得当作任务成功。退出码与 `FINISHED` 也不能替代官方计分。

## 已有测试与工具

- 主代理最新全套：794 passed in 46.72 s，JUnit `dist/integration_survey_current.xml`（修复上述问题后需重新跑相关专项，再更新全套证据）。
- survey 分析器专项：19 passed。
- evidence/camera/survey/bounded 六组专项：162 passed。
- v11 `last_associated` 兼容后，分析器专项21 passed；同六组专项更新为176 passed。主代理须在控制补丁完成后再次运行全套并更新证据。
- PAUSE+回调硬停+RLock竞态修复后，独立控制/启动143专项passed in4.57s；修后全套结果由主代理记录。
- survey 分析器：`tools/analyze_ground_survey_evidence.py`。
- ground v6/v7 分析器：`tools/analyze_ground_motion_evidence.py`。
- camera ROI 分析器：`tools/analyze_platform_camera_frames.py`。
- 台账工具：`python tools/run_records.py validate runs/records --verify-files`。

## 已有真实证据与缺口

- v7：`2208e026-e408-4119-8737-d9cab6c4a3b4`，`run_20261112_011054_34edb480`，平台 `failed`、`score=null`，65.212 s。记录了 20 次非零发布与 110 次零速；这只表示发布调用，不证明响应或得分。
- v7 回调：`runs/evidence/20261112/callback_ground_v7.jsonl`；分析：`runs/analysis/20261112_ground_v7/summary.json`；台账：`runs/records/ground_v7_2208e026.json`。
- v7 4 张真实 JPEG 的离线 ROI：OpenCV 0 候选，CLIP 10/10 拒识。地面图接近均匀绿色；UAV 图中疑似目标只有裁切窄片，尚不足以识别。
- v5 技术方案 PDF 仍为草稿：`submission/城市应急全域巡检技术方案-草稿-v5.pdf`，不包含官方完成成绩或完整比赛录像。
- 公开静态图库 8 图现均完整；详见 `docs/static_image_review_20261112.md`。`inspection_sticker_small.png` 曾截断，后从公开 Git blob 完整重取为 4,510,093 B，SHA256 `726510e8880b0441eac3d3f2c77fdf806040f77beeb625ef06cb0bde22f57070`，PNG 完整解码已复核。它仍是整场地图，不是目标类别参考；现有完整图未补齐正式七类目标图库。
- 目前没有任何一局的官方任务完成与有效成绩证据；仍须跑通单点计分后再扩大任务范围。
