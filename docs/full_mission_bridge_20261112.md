# 从首点练习接到完整全域巡检的最小接线审查

日期：2026-11-12。初版只读检查本队源码、已保存的本队平台证据、官方公开规则及已下载的公开静态环境资料。后续按实际 v25 证据更新结论，并另外完成包络校验最小修复，见 `docs/uav_envelope_fix_20261112.md`；本文未操作网站、启动仿真、发送 ROS 消息，也没有读取裁判实现、随机化分配、隐藏答案或他队任务程序。

## 结论

现有 `MissionController` 已经有完整的三地面点、返航、窗户、两空中点、降落的阶段编排；现有 v23／单点分支又已有真实地面控制与异步识别接线。**最短路线是连接这两条已有实现，并补两个实质缺口：任意当前起点的连续地面执行，以及“前视定位→飞到下视灯悬停点→照射”的空中阶段。** 无须重写第三套完整状态机，也无须等待官方 ZIP 才继续做候选地图和接口接线。

**v25 已将首点真实闭环并获得官方 10.00 分**：三帧在线识别、一次话术、原始 `recognition_status=ground_0`、真实入区停稳及一次到点声明，官方最终 `success / 10.00 / 275.179s`。该有限实验没有发布全任务完成消息，说明平台至少存在这一路自然结算；不能再把 `mission_completed` 当成取得任何分数的必要条件。v23 外观察与 v24b 识别等待失败仍是独立历史事实，不能改写。下一步应连接连续地面执行并逐项补齐飞行能力；完整完成／主动放弃的精确合同仍需独立确认。

## 当前证据能支持到哪里

| 层次 | 真实支持 | 仍未支持 |
| --- | --- | --- |
| 官方流程 | `docs/referee_rules_v1.txt` 表1、表3、表5明确三地面点按公开抽签执行；入点前识别、入点后 `current_target`；窗前声明再识别、下降穿越；空中识别后1–3秒有效照射且高度≥0.8m；返航后、降落前声明 `landing_marker`。 | 识别 ACK 字符串载荷、比赛提前放弃／结算接口、`mission_completed` 接收语义。 |
| 平台 v23 | `run_20261112_051547_2766d9ab`：本队外观察路线完成，三张真实原图及各自前序 UAV odom齐全；四个边界 scan 共1440束与固定候选静态 world 相容。见 `docs/platform_ground_route_v23_independent_20261112.md`。 | 平台终态仍 `failed / score=null / 171.756s`；该包没发识别／到点／飞行／照射／全任务完成。离线三帧候选一致不等于裁判接受。 |
| 平台 v24b | `run_20261112_055257_72484c3b`，记录 ID `e51c52ce-2605-4a4b-acb2-ea9c4a8bbfee`；`single_point_v24b_list_after_run.json` 为 `failed / score=null / 128.875s`。本队 summary 有三张 CAPTURE，最后 `FINISHED`、`reason=single_point_uav_image_stale`、phase=`RECOGNIZE_WAIT`。 | `recognition=null`、`match_results=[]`、`attempts={}`、`in_entry=false`。未证明在线识别→上报→进入→声明；summary 顶层 `partial` 也原样保留。 |
| 平台 v25 | `run_20261112_060713_501f412f`，官方记录 `c4202874-407a-4361-844a-0f95ba7b1d7f`；三帧在线匹配、一次识别发布、原始状态 `ground_0`、真实入点停稳、一次 `current_target`、本队 exit0。官方最终 `success / score=10.00 / durationMs=275179`，赛后 ground_0 到达并计分+10。 | 只证明这次有限首点取得10分，没有证明连续三点、飞行、窗、照射或完整任务完成；`summary` 的本地 `partial/unverified` 原文不因官方结果而重写。 |
| 当前本地单点源 | v25 使用 `CURRENT_CAMERA_MAX_AGE_S=.50`，依据 v24b 相机周期；保留 odom/scan 原0.20秒与曝光 prior 原0.05秒门。 | v25 的成功不改变 v24b 的 `failed/null`；后续源变动需重新做版本绑定。 |
| 本地完整控制器 | `mission.py:_build_stages/_add_window` 已表达完整顺序，`official_protocol.py` 已生成正确表5话术；`mission_node.py` 已有消息适配、停止、看门狗与正常退出。 | 这些阶段尚未在本队当前平台完成全流程；其地面控制器与实时输入队列并非 v23 已验收实现。 |
| 几何模块 | `geometry_runtime.py`／`runtime.py:pose_at_exposure` 已保存原始完整 odom并提供有界前序曝光匹配；`target_geometry.py` 已有 `localize_target`、`hover_for_downlight`、`assess_illumination`。 | 业务代码没有调用三项目标几何函数；没有实际空中 TargetFix、灯轴／测距适配、迁移或有效照射证据。纯数学测试不等于平台动作或得分。 |

v24b 来源：`runs/evidence/20261112/single_point_v24b_list_after_run.json`、`runs/analysis/20261112_single_point_v24b/summary.json`。v25 来源：`runs/evidence/20261112/single_point_v25_list_final.json`、`runs/evidence/20261112/callback_single_point_v25.jsonl`、`runs/analysis/20261112_single_point_v25_independent/summary.json`。本文没有改写这些原始文件。

## 已有公开材料足以做的工作

已有固定来源 `ranmengjia/ICAN@82b041043849a5d86e085390bf0df787366d98cd` 的两份接口说明、`inspection_task1.world`、场地图和来源 manifest；`docs/static_resource_contract_20261111.md` 已记录来源与哈希。world SHA256 为 `595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1`。该公开静态副本没有证明等于当前官方版本，但足够生成、计算并在平台传感器中逐段核验候选计划。不要再把“拿不到 ZIP”当作所有接线都不能进行的原因。

| 对象 | 已有公开候选值／合同 | 下一步真正要核验的内容 |
| --- | --- | --- |
| 地面控制 | `/cmd_vel`，`geometry_msgs/msg/Twist`；v23 已有真实前进、转向和停车响应。 | 任意点间连续航段、实际偏航下的响应及末姿态；不能用起点+x短直行推定所有方向相同。 |
| 飞控 | 公开接口说明是 `/uav/cmd_vel` 的 `linear.x/y/z`、`angular.z`，持续正z上升、零速度保持、负z下降；不需要先猜 Trigger 起降服务。 | 当前平台实际 body/world 坐标合同、正z响应、零命令停止／悬停、实际速度和停距。当前只有输出端 matched 证据，没有非零飞行实测。 |
| 窗1／窗2 | 候选中心 `(1.925,-.3275)`／`(1.925,-2.3725)`；窗体 x厚0.16m；洞宽1.00m，垂直开口 z∈[.40,1.15]。目标板分别在西侧 x=1.815／东侧 x=2.035、z=1.40。 | 当前点云／图像和 odom中的洞口、机体水平／上下包络与余量、下降及穿越响应；窗1从西往东、窗2从东往西只是静态候选路线。 |
| 两空中点 | 候选板中心 `(3.080,-.320)`／`(3.080,-1.355)`、板盒顶面z=.012、尺寸 `.35×.30m`。 | 每局真实图像类别、曝光定位、下视灯中心命中与稳定性；不能从候选板位置或上一局类别填答案。 |
| 起降点 | 候选指挥中心 `(0,0)`；真实起点 odom约z=.055。 | 当前平台的落地／停止判据、返航定位与结算，不从候选 `min_z` 猜裁判判据。 |
| 灯光 | `/uav/laser_fire` Bool；v5 已实收 `/uav/laser_state` Bool false，1297次回调、RELIABLE/VOLATILE。 | `true→false`的真实响应、时延与持续更新；Bool是开关状态，尚未证明是中心命中或裁判接受。 |

机体外廓不能为穿窗缩小：公开模型有可见旋翼，现地面保守半径0.447m；窗洞的垂直开口只有0.75m。旧 `_validate_plan` 把同一水平半径当垂直余量造成假拒绝。该最小修复现已将**水平包络与机体原点上、下包络分开建模**，保留0.447m并计倾斜膨胀，同步点云与姿态运行门；缺显式包络时仍走旧保守球语义。显式包络还必须给出窗宽，并证明窗宽可容纳半径、净空与实际控制容差；候选1m窗会拒绝默认0.08m容差。见 `docs/uav_envelope_fix_20261112.md`。这仅是本地配置／运行门修复，未证明当前平台窗洞或飞控可通过。

## 最少改动的接线顺序

### 1. 保留完整阶段编排，接入已验收地面执行器

复用 `MissionController._build_stages` 的任务顺序、官方编码器和 `mission_node` 的输出绑定；**地面阶段由 v23 的 `GroundObservationRouteGuard` 受限航段内核执行**，不要直接切回 `MissionController._velocity` 的旧地面同比例控制。后者允许同时平移和转向，未包含本次已实测的双档校准、窄转角窗、逐腿停车累计路径门与 PAUSE恢复合同。

当前 `GroundSinglePointGuard` 只处理首ID；`plan_ground_observation_route._initial_pose` 强制距原点≤.05m、yaw≤.05rad，单点结束又进入 STOP／FINISHED。因此**不能把当前入口连续调用三遍**，不能把上一点中心冒充新起点，也不能每点重置总路径／转角／总时间。

最小扩展是让现有航段执行内核接收显式 `segments + final_yaw + budgets`，由同一任务监督器持有一次 session、冻结公开三点顺序、实际初始 pose与全程累计账本。每点只清局部航段和识别 epoch；worker context改为 `(session, active_point_id, point_epoch)`，每点的原图、三帧一致性、K、prior pose、模板 provenance和一次发送锁存仍复用单点实现。现有 `frozen_first_ID` 可在兼容适配层保留字段名，但其语义必须清楚绑定当前活动点，不能继续永远指向抽签第一个。

任务接口只需明确：当前阶段请求一段地面路线／一次识别／一次到点声明；执行器返回“已到观测位／已建立当前视觉证据／发送返回／已入区停稳／失败”。只有执行器的实测完成才能推进高层阶段。新任务总预算按整条已冻结候选路线及真实速度响应制定，不能在已有单点3m预算走完时临时扩容。3m、6rad、300s是本队单点实验合同，不是官方比赛规则。

`MissionController` 的每点 `via` 是静态列表；随机前一点不同，固定 `via` 不自动保证从三个实际起点均安全。最小候选过渡可沿已核对左半场通道 x≈.6组合路线，并从**真实当前pose**计算退出上一点→当前观察位的各段；必须逐段做静态扫掠与当前scan门禁，不能简单两点直连。无需每点回到(0,0)重跑首点程序；那会增加路程并掩盖累计预算问题。

### 2. 复用当前单点视觉／声明闭环，补足所有五点覆盖

每点仍执行：`外观察到位且静稳→三帧当前图绑定→唯一类别→一次表5地面话术→明确transport等待→地面入区→停车静稳→一次当前ID声明`。transport等待只代表本队发布行为，不能插入假 ACK。地面三个点及地面返航期间，非零 `/uav/cmd_vel` 必须始终禁止。

当前单点仅 `ground_0/1/4`允许进入；对 `ground_2/3`明示拒绝。已有 `docs/ground23_candidate_geometry_20261112.md` 给出**未执行的候选**，不必重新设计：g3观察后直达 `(1.115,-1.925)`并末朝南，含0.30m余量约2.962m；g2可经 `(.6,-2.62)→(0,-2.69)`并末朝西，含余量约4.124m。原g2中心轴线的最小laser距离0.57m，过不了原0.641m环扫门，不能仅提高里程cap。

以上候选新增末朝向对齐及交接，当前代码还没有实现；不得把离线yaw视为已经执行。候选全程最小净空0.650m比0.641m门只高9mm，不能承诺在现2cm误差容限下必过。也不能声称整机完全入灰色区域：完整旋翼外廓最小投影宽≥0.67m，而任务板最大边仅0.55m；本队最多分别证明中心／含轮底盘入区，官方得分只能由真实裁判结果确认。

### 3. 先绑定真实飞行和自由空间，再复用窗户阶段

保留完整控制器的 `RETURN_GROUND→TAKEOFF→WINDOW_1_*` 顺序。依据公开速度接口，优先验证 `flight_mode=velocity`；没有已确认 Trigger 合同，不需要增加 takeoff／land／flight_abort假接口。

`runtime.py:motion_clear` 的现pointcloud模式在 `abs(omega)>1e-8`时一律拒绝；前向点云也不覆盖起飞上方、垂直下降、后方和整机停止扫掠体。它不是填上topic就能执行。最小修补应在现有净空入口组合**当前水平scan／前向点云／下视测距 + 经当前平台核验的静态起降柱和窗洞自由空间**，给出每类动作的证据；保留未知方向拒绝。不应删除原拒绝分支或把下视三束 `/uav/laser/scan`伪装成水平 `uav_scan`。同一模型上的 `/scan`可作为空中水平补充候选，但必须用飞行中的当前header／pose实测，不能把地面scan证据直接外推。

窗户复用 `_add_window` 已有的 `APPROACH→DECLARE→RECOGNIZE→DESCEND→CROSS`。仅补真实几何／外廓、窗口视觉和飞行执行器；声明顺序已有官方来源。窗口识别与当前地面平面matcher不是同一个平面合同：现`ground_plane_matcher`只处理地平面，不能直接拿其IPM去识别竖直窗上图片。复用现 `vision_core.TemplateMatcher` 的参考图和原图单应候选接口，或让现异步worker接受显式窗口平面模式；不必再造整套视觉节点，但必须用真实窗口帧验收两类拒识／识别。

### 4. 只补空中识别到照射之间缺失的两个阶段

当前 `AIR_NAV→AIR_RECOGNIZE→AIR_INSPECT`共用一个 `point.pose`，`_illuminate_reported`还要求前视目标持续居中。公开候选相机前伸.390m且前向，灯向下；机器人位于地面目标上方时该目标在前视相机后下方。**这个结构不能仅靠配置修好。**

复用现成几何函数，最小阶段链改为：

1. `AIR_RECOGNIZE/LOCALIZE`：在前视可见观察位获得真实类别；用原曝光图、K、完整前序pose和明确外参生成带point／epoch／误差／年龄的`TargetFix`，一次发送空中话术。
2. `AIR_MOVE_TO_LIGHT_POSE`：`hover_for_downlight`算补偿灯偏置的机体悬停点；先经已有／补充路径净空门，移动期间灯关闭。
3. `AIR_ILLUMINATE`：当前完整pose、缓存但未过期的TargetFix、独立有效下视range、真实灯反馈交给`assess_illumination`；允许目标离开前视画面，只累计连续合格的1–3秒，条件丢失立即关灯和清累计。随后明确关灯再去第二点。

不要使用report产生时的最新pose替代曝光pose，不把confidence／像素重投影阈值填成米制误差0，不刷新旧TargetFix时间。`runtime.py:pose_at_exposure`已可复用；待补的是从原始证据构造`StampedPose3D/CameraObservation/StampedRange/StampedBool`的适配及可证误差界。

### 5. 复用窗2、返航、降落；独立验证收尾

复用既有 `WINDOW_2_*→RECOVER_AFTER_WINDOW_2→RETURN_UAV→LAND_DECLARE→LAND`。按官方顺序先在返航终点发布 `landing_marker`，再真实下降、停稳；停止交付、进程正常退出和官方成绩分别记录。完整完成才可进入现有候选完成消息路径。不能因为某个有限实验已安全停止，就提前调用 `RunOutcome.complete_after_stop`发送全任务完成。

## 必须先实测的最少节点

以下是四个**能力验收节点**，不是要求固定新增四局；满足安全前提后可在同一真实运行依次验收。结算合同核查可并行进行，不应等全部动作都接完才发现无法提成绩。

| 节点 | 最小实际动作与输入 | 通过所需证据 | 本地／不通过仍能做的工作 |
| --- | --- | --- | --- |
| A：首点闭环 | v25 已完成一次识别→上报→允许入区→停车→声明。 | 三帧及各自证据、在线worker结果、准确一次话术、真实入区停稳与一次声明；原始 `ground_0` 状态及官方10.00分已覆盖本次有限首点。 | A本次通过；v24b仍未通过。接线B连续地面任务，不以重复同一首点代替。 |
| B：连续地面与飞行交接 | 同一session按公开顺序连续三点并返航；覆盖g2/g3新路线和末对齐后，才放开其准入。 | 逐点独立视觉和一次上报／声明；全程累计量不重置；点间无非零UAV命令；返航真实pose。 | 不同draw应按可执行能力真实停止或跳出为未完成，不能换抽签／猜类别；不靠重复首点冒充三点。 |
| C：飞行／窗／照射基础 | 在静态与当前传感器已确认的起降柱内做有界正z、停止、下降；在安全区核对横向与yaw合同；保留完整点云、下视range、灯true／false与曝光pose。随后实测第一个窗的声明／识别／下降／穿越及第一空中点定位→悬停→照射。 | 真实方向、速度、停距、上下包络；窗口几何和实际下降跨面；TargetFix来源、激光响应、合格高度和1–3秒连续稳定中心命中。 | 几何三函数、窗口包络、真实tick参数组合回归及拒绝路径都可先做；不删门禁放行。 |
| D：完整串联与结算 | 复用能力覆盖第二空中点、窗2、返航、降落前声明、真实落地和停稳，再完成可信收尾。 | 两窗两空中点和落地的原始轨迹／图像／灯记录；真实平台终态、结构化逐项诊断与非null分数。 | 完成日志／源码哈希／视频数据接线可预备；`COMPLETE`、exit0、发布返回和网页默认0分均不当官方成功。 |

现有单点相关本地独审分别见 `docs/control_review_ground_single_point_20261112.md`、`docs/control_review_ground_single_point_transport_20261112.md`；其报告明确只验证本地控制、源绑定和模拟ROS，不是A至D的替代物。初版只读审查没有重跑测试；后续包络修复的专项红绿与源码快照单独见 `docs/uav_envelope_fix_20261112.md`。

## 完成标志与“主动放弃／提前结算”合同

1. **官方V1能确认的巡检收尾只有降落前 `cruise_event=landing_marker`和真实降落。** 表1的 `/referee/finish`、`finish:return_start`明确属于陆地障碍物清除，不能给巡检借用。官方注意事项只说评测结束后提供结构化诊断，没有给比赛提前停止／放弃话题、schema或结算流程。
2. **当前 `mission_completed`来自论坛线索，尚未验证。** `mission_node.py:completion_status_payload`自身注释为“Completion payload reported in the platform forum; transport is unverified”，默认发布话题 `/inspection/mission_status`。`docs/terminal_marker_fix_20261106.md`引用[巡检的结束条件是什么？](https://www.splenach.com/ros-simulation/forum/6999b2f6-69c7-4c53-bedf-e6a2001eefe5)，并明确未证实作者身份／官方协议。控制器仅在全部阶段完成、LAND通过、停止发布成功后走该候选路径；本机用例仅证明该条件和退出等待。
3. **本次已保存资料未找到合法的比赛提前结算／abort接口证据。** 核查范围为官方规则、`.firecrawl/ros-evaluation-notes.txt`、巡检指南及11/12刷新副本、`docs/runner_contract_current.md`、`docs/protocol_gap_current.md`和上述论坛线索说明。若官方另有“允许主动放弃”的文字，允许这种行为本身仍不等于ROS/API接收合同；本次不能据此猜消息名、发布`mission_aborted`或把首点实验标成全任务完成。
4. **两个同名概念不能误用。** 前端 `evaluation/upload` 的 `mode=abort`是取消分片上传，与已运行比赛无关；本队profile里的`flight_abort`是可选Trigger飞行紧急停止配置键，当前没有平台确认接口，且停止运动不代表结算比赛。`RunOutcome.fail`只让本队安全停止并退出，也不自动要求官方结算。
5. **动作和结算要保留独立状态。** v25 现有证据可写 `local_ground_transport_complete=true, official_acceptance=true, official_score=10.00, full_mission_complete=false`。它未发完整完成消息而自然得到官方有限首点分，已反驳“只有完整标志才能拿任何分”的假设；但还不能推导所有退出路径或主动放弃接口均相同。仍需补：主动放弃精确接口及适用赛项、其他退出路径是否保留已得分、完整任务自然结束条件、终态消息在完整任务中是否必要、逐项诊断可得时机。不能仅由publish返回推断。

## 当前明确未验证的接口清单

| 接口／关联 | 已知 | 缺失的精确部分 |
| --- | --- | --- |
| `/referee/recognition_status` | v25 识别后实收原文 `ground_0`，随后真实入区并获得该点10分。 | 跨point/draw关联、拒绝值、时效/epoch及重试语义；官方话术不带本队UUID，不能套现通用event_ack解析器。 |
| `/inspection/detections`、`current_target` | v25 已有一次话术、一次入区后声明及官方 ground_0 计分证据。 | 连续不同点、重复发送/重试幂等语义；不得将单次成功泛化为完整任务。 |
| `/uav/cmd_vel` | Twist公开速度接口、桥接匹配。 | 非零实际响应、body/world、z/yaw、零命令悬停／制动；Trigger模式尚无必要或证据。 |
| `/uav/laser_state`与下视测距 | Bool false和三束scan已有真实回调；静态laser TF已有候选/部分观测。 | true响应/延迟、有效非inf range与独立外参、完整足点平面和命中语义。 |
| 当前平台相机/点云/灯/外廓 | K和地面候选投影、部分TF已相容；公开world有静态值。 | 当前平台空中帧链、真实窗与上/下包络、运动中同步误差、定位误差上界；不能默认缺失误差为0。 |
| `/inspection/mission_status` `mission_completed` | 本队已有候选发布和退出实现，论坛线索。 | 当前平台订阅/桥接、消息真实接受、它与自然结算/落地的关系。 |
| 比赛放弃／提前结算 | 本次缓存范围无精确合同。 | 精确公开topic/service/API、载荷、先决条件、计分/诊断与退出语义。不得拿上传abort或飞行abort替代。 |

## 本轮审查源快照

以下只用于将本文与并行开发区分，哈希不是平台上传绑定；共享源后续变化须按新包manifest复核。

| 文件 | 本次读取SHA256 |
| --- | --- |
| `mission.py` | `81ddf73cdc545688810e4b21db4985e541c88a7089e243dcc45d28099b66e1c4` |
| `mission_node.py` | `23daea54572bd2f4624f9eacc555c166386724388ef3b4646dfe29b58b0a64b3` |
| `runtime.py` | `1472dfaab7380de11ee4f93c90870d85116c35481a879c27ac4e781b4dcd8682` |
| `ground_single_point.py` | `902424ca5ba3ec208a0028645d377f4ce92ff77e5ee56205c87fac95a9b349cb` |
| `target_geometry.py` | `716fdee7bc72d563c1be4102c859ee587fc011293aeec98f901b9eaa1f2faea6` |
| `referee_rules_v1.txt` | `45c6eaf7c81777af2c6f47a3cd677a3c98c3f3285e20bed5297f08df36d2931e` |

