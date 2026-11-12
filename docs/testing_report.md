# 全域巡检工程测试报告

更新日期：2026-11-12。本报告保留历史记录，并在前部列出最新增量验证。

## 2026-11-12 官方平台只读探针 v5 实收与终态

- `run_20261111_222348_ef5c04b0` 已取得5条自有网络诊断：launcher启动、完整快照、两张JPEG、launcher退出码0。未收到 `probe_started`、`probe_finished` 事件；完整快照和14类真实公开消息仍证明探针节点运行。分析状态保留 `partial`、`official_scoring=unverified`，无解析错误。
- 实收两路800×800 rgb8相机和CameraInfo、两路odom、TF/静态TF、clock、地面scan、UAV窄束scan、点云、`/uav/laser_state=false` 和公开地面抽签。快照上报累计回调数与保存的代表样本数分开，不用60秒采样设置推断仿真频率。两张360×360 JPEG只作诊断，保留实际字节与哈希，不是完整任务录像或视觉准确率结果。
- 本轮公开抽签为 `ground_1,ground_4,ground_0`，不是图片答案，也不能据此推出全五点集合或坐标映射。识别状态仍无实际data；控制/识别/声明输出端点快照为0订阅，仅记录该时刻现象。
- 平台最终证据 `runs/evidence/20261112/probe_v5_final.json` 为 `failed`、`score:null`、`durationMs:112172`；更新时间2026-11-12 00:17:06 +08。只读探针未执行比赛任务，未发控制、识别、完成或计分消息；exit0不等于任务通过。查询时服务down不回推为失败根因。
- 回调原文SHA256为 `2bb334edc51f1b9207bedf22323d0b15c03280157e3fca148355e03e8ce9723c`。详情、内参、frame、快照计数、JPEG哈希与剩余边界见 [v5平台探针报告](platform_probe_v5_20261112.md)。最新本地完整检查仍以实际JUnit为准，本次证据刷新未产生新业务测试结论。

## 2026-11-11 22:51 本地修复与平台 v5 提交

- 最新完整检查为 `577 passed in 40.77s`，原始JUnit为 `dist/current_tests.xml`；开发源码包58208B，构建仍明确 `configuration_ready=false`、`platform_run_verified=false`。10条台账校验通过，上传的v4/v5文件保持原始哈希。
- v5练习记录为 `run_20261111_222348_ef5c04b0`，详情ID `a1485e1e-27a0-45b0-bd2f-d3c313fdef6a`；60秒只读采样、最终汇总和独立JPEG回传。提交及文件哈希存于 `runs/evidence/20261111/probe_v5_*`，当时仍排队，不计为任务成功。
- 诊断修复专项108项；新增本地证据分析器16项；材料生成相关24项。真实JPEG保持原字节和哈希；非法或缺少的回调不推断为ROS运行失败，也不生成替代画面。
- 新增纯数学 `target_geometry.py` 的51项专项与独立复核通过；最大射线倾角包含姿态、外参和时间运动角误差，定位漂移后重新检查硬误差上限。该模块未接控制器，实际相机标定、曝光位姿、灯/测距合同和空域门禁仍待接通。复核见 `target_geometry_review_20261111.md`。
- 技术PDF草稿已引用指定JUnit与21条v4真实回调，5页渲染已逐页查看；它没有官方完整任务得分或录像。旧MP4仍是人工开发演示。

## 2026-11-11 官方平台只读探针 v4

- 练习测评 `run_20261111_155937_44feef8e` 已取得 21 条自有诊断回调：上传 shell 启动、rclpy 探针启动、公开图、14 条订阅创建、2 条真实 `/odom`、1 条 `/tf` 与脚本退出码 0。直接从源码运行不证明 colcon 编译成功；详情、事件计数、QoS 和哈希见 [平台探针报告](platform_probe_v4_20261111.md)。
- 平台最终快照 `runs/evidence/20261111/probe_detail_latest.json` 为 `failed`、`score:null`、`durationMs:66337`，16:59:38 +08 查询时服务为 `up`。本次探针没有发布控制、识别或完成消息，也没有执行比赛任务。不能把“未提取到成绩”改写成代码未启动，不能把退出码 0 改写成任务通过。
- 回传 FIFO 存在积压：探针启动至首个 TF 样本的生成跨度约 0.086 秒，对应接收跨度约 28.83 秒；进程结束前只短暂等待发送线程。尚未收到的相机、抽签、状态和最终摘要不能据此判为 ROS 回调未发生。
- v4诊断阶段曾保存起始时间为2026-11-11 15:53:47 +08、456项通过的本地JUnit；当前该文件已由上节最新真实检查更新。历史通过数不代表后续源码版本。纯 Python 测试仍不证明运动、识别、裁判计分或正式录像。

## 2026-11-11 账号内核验、台账与识别合同修复

- 账号内求助帖详情已核实：状态为 `rejected`，审核时间为2026-11-10 15:40:21 +08。审核说明：“工作空间等等文件都在官方的百度网盘连接中。测评侧的构建、运行日志等等内容都是不展示的。”这是审核回复，公开评论数仍为0不能解读成“没有任何回复”。证据：`runs/evidence/20261111/support_post.json`。
- 这条回复解释了构建/运行日志的展示边界；它不能证明用户脚本已运行，也不能证明未运行或平台故障。当前未取得实际官方百度网盘分享链接。今日课程API仍只有视频，`documentKey`、`documentContent`、`documentFileName`、lesson `content`为空，见`runs/evidence/20261111/inspection_course.json`。
- 已找到两份公开第三方工作区候选，保存了固定commit、文件哈希、静态地图/接口说明与七类候选参考图。它们未获证据认定为当前官方环境，也未替代官方裁判或隐藏随机化信息；来源和限制见`docs/resource_search_20261111.md`。
- 本队历史API返回8条唯一测评，全部 `failed + score=null`；已导入`runs/records/`。4条探针单列排除，4条任务类提交保守纳入分母，证实完整完成为0、未知完成情况为4，因此当前台账完成统计为0/4。kind只保留提交类别/意图，其中诊断脚本不证明真实完整任务执行；不能把此比例作为四场已执行算法试验的可靠性估计。最低有效总分为`null`、有效耗时n=0，逐项分、人工分、旧源码与脚本哈希都保持未知。分类依据见`runs/records/README.md`。
- 今日早期历史与详情快照返回`serverDown=true`、`serverStatus=down`，不把查询时服务状态回推为旧失败根因。证据：`runs/evidence/20261111/evaluation_history.json`、`latest_probe_detail.json`。后续 v4 最终快照已返回服务 `up`，见本报告首节。
- 识别确认已支持明确的两种合同：默认`event_ack`；只有profile显式启用`transport_only`时，节点不订阅或等待未证实存在的识别ACK接口。仅在真实话术发布返回成功、等待正时长且视觉/姿态仍有效后推进，不填充ACK集合、不宣称裁判认可。运行状态持续标记`official_scoring=unverified`。
- 台账专测39项；识别确认新增12项，包含无ACK的完整离线流程和发布异常回归。该次完整代码测试为 **352 passed in 20.86s**，只证明本地代码行为，不能证明当前官方ROS/Gazebo运行、得分或真实录像完成。

## 2026-11-11 早期启动器与复赛准备（历史增量）

- 正式启动脚本的 overlay 查找改为：显式 `INSPECTION_WORKSPACE` → 脚本父目录 → 脚本目录 → 调用方当前目录。每次 source 成功后，只有能解析本队 `inspection_mission` 才结束查找；真实 source 失败仍保留原退出码。
- 测试先复现 2 项失败（选错 cwd 的配置、遇到缺包 overlay 后未继续查找），再修复。相关启动器/探针测试 **23 passed**；全套 **296 passed in 21.06s**；强化 source 失败测试后，启动器测试 **12 passed**。`bash -n` 通过。
- `python tools/preflight.py .`：**PASS**。
- `run_scenario(demonstration_plan(protocol='inspection_v1'))`：人工地图、传感器和确认下完成 **727 步、7 份识别报告**，终态 `COMPLETE`；不是官方 ROS/Gazebo 运行或比赛成绩。
- 当时重新读取现有官方课程：无示例附件，已知V1.0判分正文未变。匿名帖子详情返回HTTP 403；这项早期限制已由上节账号内快照补充，不能继续写成“尚无平台回复”。
- 该次启动器修复未发起平台测评，未消耗正式提交机会；当时`configuration_ready:false`、`platform_run_verified:false`。修复启动路径本身不能证明真实平台启动或得分。

## 2026-11-10 增量验证

- `python -m pytest tests -q`：**290 passed in 16.82s**。
- `python -m compileall -q src tools tests`：通过。
- `python tools/preflight.py .`：**PASS**。
- `python tools/mission_scenario.py`：离线运动学流程 680 步、7 份识别报告，到达本地 `COMPLETE`；不代表官方 ROS/Gazebo 场景或比赛成绩。
- 新增故障回归：机器人偏离观察点且冻结时，重捕获导航在 `no_progress_timeout_s` 后进入零速故障停车，不再等到整阶段超时。
- 窗口下降校验不再把 0.2 m 当作官方门槛。判分细则仅要求“识别成功且下降穿越”；本地仍要求观测到真实下降，并校验机器人净空与定位容差。
- **本轮未运行 ROS/Gazebo，也未在网站提交测评**；`dist/readiness.json` 继续保持 `platform_run_verified:false`。

## 2026-11-06 基线结果（历史记录）

## 本轮结果

- `python -m pytest tests -q`：**253 passed in 10.55s**。
- `python -m compileall -q src tools tests`：通过。
- `python tools/preflight.py .`：**PASS**。
- `run_scenario(demonstration_plan(protocol='inspection_v1'))`：人工地图、人工传感器和人工确认下完成，727 步、7 份识别报告、6 次声明；不是官方 ROS/Gazebo 场景，也不是比赛成绩。
- `python tools/build_submission.py`：生成开发包，源码 ZIP **41,413 bytes**，20 个条目，含 `src/inspection_mission/package.xml` 和 `official_protocol.py`，无缓存。
- 启动脚本为 LF、无 BOM；仅启动本队节点，保留最早启动标记和真实退出码。
- `dist/readiness.json`：`configuration_ready:false`、`platform_run_verified:false`。

源码 ZIP SHA256：`5be8edea774d370253063d4b91f73883d2dfd49dd3de04ceeb46779a9ce4a820`。

启动脚本 SHA256：`53aa0c6a619ae635e98cc241318d8338c36f259d83eddd6a5c58a73024bbccb7`。

## 已修复的行为

1. 五个地面候选点中抽取三个；官方 String.data CSV 订单严格解析，拒绝重复、未知、空项和错误数量。
2. 根据实测单一类别生成官方表5的12条话术；拒绝未知类别、组合猜测和别名猜测。
3. 内部视觉数据迁至 `/inspection_mission/observations`、内部状态迁至 `/inspection_mission/status`；裁判 `/inspection/detections` 专用于识别话术。
4. inspection_v1 地面观察、识别确认、驶入点内、到点声明依次执行；窗口识别前声明，降落下降前声明。
5. 空中先识别确认，再照射；失去视觉证据、偏心、漂移、身份变化或歧义时灭灯并重新识别确认；照射阶段不重复上报。
6. 从首次开灯命令独立计时，达到3秒而有效证据不足时灭灯并明确失败；反馈延迟和中断不延长同一次照射。
7. 正式模式校验不超过0.5 m/s的地面、空中与自动起降速度；固定错误声明值、错误CSV绑定、缺少五个真实候选ID都会阻断启动。
8. 无初始里程计时等待；返航和降落必须实际到达目标位姿；不能关闭识别闭环。非法航点、NaN/Inf、已废弃的全局高度参数，以及无CameraInfo却配置物理测距尺寸均明确失败。

## 仍不能证明的事项

本机无可用 ROS2 Humble/rclpy 或官方巡检工作空间。测试中的边界替身不能证明真实节点启动、DDS/QoS通信、机器人运动、官方图片识别或裁判接受。独立的平台 v4/v5 探针已证明上传脚本、探针节点运行和真实公开传感器接收；v5新增双相机、CameraInfo、公开抽签等14类样本，但没有验证完整业务节点、机器人运动或裁判得分。

已取得官方判分细则，并保存为 `docs/referee_rules_v1.txt`；当前课程页和课程详情数据未发现 demo_inspection_ws.zip 下载附件。用户本轮明确表示目前无法取得该资料。

当前已经取得第三方候选地图、参考图和接口文本，以及v5真实相机/CameraInfo、里程计、静态TF、激光和点云样本。仍需核实地图/观察位/点内位姿、图库覆盖、控制坐标系与接收端、光学外参、完整传感器覆盖、灯光开启/命中语义和正常结算合同。`/referee/recognition_status` 为 String，但实际 data 和关联语义仍未取得；官方话术没有定义本队报告 UUID 的回传。显式 transport_only 适配的本地发布成功不能替代裁判接受或计分证据。阶段连接条件已拆分，一次声明交付与最终退出流程仍需平台实际证据。

本次导入的8条历史中，11月6日的只读探针`run_20261106_031856_3b37ac97`只有裁判启动日志、无有效成绩；平台审核已说明用户构建/运行日志不展示。11月11日 v4 新探针的独立启动与传感器证据见本报告首节；其平台终态仍无有效成绩。新记录没有覆盖历史或被改写成完成标记、确认、得分。

剩余合同和所需证据见 `docs/protocol_gap_current.md`；平台日志问题说明见 `docs/platform_support_request.md`。
