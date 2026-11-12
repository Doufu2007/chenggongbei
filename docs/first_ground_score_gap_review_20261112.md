# 首个真实地面计分闭环：只读复核

2026-11-12。仅核查本队公开传感器/图、公开静态资源与现有源码；本轮没有运行控制、修改控制源、读取裁判实现/随机答案或进入私有目录。平台“仿真/AI/算法”模块均未开放，不把它们列作运行依赖。

## 先改正当前判断

**主要缺口已不是“源码没运行、输出桥不存在”**。本队 v15 已有真实地面转向/位移和六个输出的匹配订阅者；当前缺口是可靠运行到观察位、真实图像识别、物理点位对应和一次实际接受/计分。完整 `mission_node` 目前仍是严格拒绝空配置的通用实现，尚不能替代已实测的有界地面控制器。

证据：`runs/analysis/20261112_approach_v15/summary.json`；原始 `runs/evidence/20261112/callback_approach_v15.jsonl`，SHA256 `0e205c23a3ed85392c7cff019788a644da1eb2f22261f8efe16719bc862a8a8f`。本局抽签 `ground_1,ground_2,ground_4`；转向后 `drive_yaw≈-1.5464rad`，前向投影 `.1206299m`，结束原因为 `approach_drive_wall_time_limit`，未进入CAPTURE，更未取得计分。

## 哪些已经有真实证据

| 接口/事实 | v15 可确认内容 | 仍不能据此宣布的内容 |
| --- | --- | --- |
| `/cmd_vel` | Twist；桥 `/ican_inspection_public_bridge_141`，RELIABLE/VOLATILE，实际 matched=1；同局非零地面命令、转向及位移 | 全速响应、米级路径、制动上界或所有方向控制增益 |
| `/uav/cmd_vel` | Twist，同桥同QoS、matched=1 | 飞行响应、body/world坐标、起降与速度增益；该局严格未发布 |
| `/inspection/detections` | String，同桥同QoS、matched=1 | 话术已接受、类别正确、报告与点位关联；该局未发布 |
| `/referee/current_target`、`/referee/cruise_event` | String，同桥同QoS、matched=1 | 到点/穿窗/降落声明已执行或已计分；该局未发布 |
| `/uav/laser_fire` | Bool，同桥同QoS、matched=1 | 灯光方向/实效、状态回传或裁判照射接受；该局未发布 |
| 输入 | 真 `/clock`、`/odom`、`/uav/odom`、360束 `/scan`、两路800×800图/CameraInfo、本场CSV抽签 | camera/laser完整TF、地图ID语义、识别ACK载荷与全程连续freshness |

因此可以从以上事实创建**新的、范围明确的地面运行绑定**；不要继续要求用户重复提供已确认的话题名，也不能把六个 matched=1 当作六个动作已验证。

## 相机和板号是否有逻辑错误

1. **默认地面视觉来源确实不适合当前实图。** v15 原图 `native_L000019_5514907183cfdd93.jpg` 仍整幅绿色；同局 `native_L000017_e352042ee1288004.jpg` 的 UAV 图能看见地面场景和任务板。两图位于 `runs/analysis/20261112_approach_v15/`。这不证明绿色来自下视、遮挡或渲染故障，也不能通过修改机器人模型修复官方场景。
2. **相机名为UAV不等于使用飞行。** v15 两路TF和同期odom在地面阶段随同运动；可保持仅 `/cmd_vel` 运动，地面识别显式配置 `camera_source="uav"`、`camera_frame_id="uav/camera_link"`。现有接线已实现，见 `docs/ground_camera_binding_20261112.md` 与 `mission.py:450`。保留真实source/frame/stamp，不能把UAV topic塞进ground槽重标来源。现有 `_Stage.vehicle` 和上报 `kind` 保持ground。
3. **两路都是候选前视，尚无“已实测下视相机”的合同。** 公开固定world中 ground sensor pose为`(.16,0,.11,0,0,0)`，UAV为`(.390,0,.130,0,0,0)`；K≈`fx=fy=476.703,cx=cy=400.5`。这只是候选外参；真实TF只留有机体与下视laser，不能把CameraInfo的 `P[3]=-fx*.07`当机体外参。见 `external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world:286`。
4. **不存在已证实的“编号应减1”。** 公开布局/模型有0..4，真实抽签确实曾包含ground_0；候选 `CANDIDATE_POINTS` 使用同号0..4。默认 `mission_plan.json` 的ground_1..5仍是明确占位，必然漏掉已出现的合法ground_0，不能用它实跑。但同号ID到候选world的空间映射尚未由公开点位合同/现场可见编号完成独立确认。类别也不是板号的固定函数：每局只依据当前图像，不从ID选类别。
5. **“到点中心才要求前视图居中”会制造几何死锁。** 目前官方流程实现已经采用外观察位 `approach→recognize→enter→declare`（`mission.py:298`），顺序正确，应保留。`_target_detection/_inspect` 还要求归一化中心XY均在默认`.12`内；当前yaw视觉伺服只能改水平中心，不能抬低镜头修垂直偏差。外观察位必须从真实图选择，使完整板满足视野/中心条件，不能让控制器在板正上方等待永远无法居中的前视图，也不能靠降低识别门槛宣称成功。

## 完整任务软件还有哪些空缺

- `official_protocol.py:17` 已能把五种真实地面类别编码为官方表5话术；它不是分类器。`vision_core.py:81` 有ORB模板定位，`vision_node.py:125`调用通路可提供带真实source/frame/stamp的检测，但当前 `inspection.yaml` 两图topic均空、模板字典为空；QR/ArUco不能代替五类照片识别。
- `mission.py:298` 已构造三地面点、返程、两窗、两空中点照射、返航降落顺序；窗户先声明后识别，地面先识别后进入、到点后声明，话术 `kind` 与相机source分开，基本协议次序没有发现逆序问题。
- `interface_profile.json` 仍 `verified=false`，cmd/frames/scan/flight_mode/light_feedback未绑定；`mission_plan.json` 的全部pose/approach/包络为null。`protocol.py:60`会立即拒绝，不能翻一个verified开关绕过实际缺口。
- 完整profile验证还要求飞行、UAV障碍与灯反馈，即使只想先跑地面。最短首点验收应由**范围明确的地面单点节点/配置**复用已实测安全控制与纯话术编码，不虚填空中字段来强行启动完整任务。
- `/referee/recognition_status`已知实际String，但仍没有一次可解析消息来确认事件ID/接受语义。源码默认`event_ack`不能凭空设置字段；已有显式`transport_only`可把“发送后等待”作为本地推进策略，**不是官方接受**。后续若实际ACK出现，再按原样建立解析与关联。
- `mission_node.py:21`的`mission_completed`仅为平台论坛描述、传输未验证；官方V1巡航没有给同名结束话题。局部首点成功不发送全任务完成，不伪造裁判结束标志；分数提取/结算仍须以平台真实结果验证。

## 最短闭环：每一步只增加一个可验证结果

1. **先到首点外观察位。** 从本局公开CSV取第一ID；沿独立候选路线以既有freshness/净空/预算闭环行驶，保存实测轨迹和scan残差；停止后取得至少3张不同曝光stamp、各自真实前序UAV pose的完整原图。v16首先验证同步修复，不加发识别/声明。若运行超时，按实际暂停/速度/路径预算定位，不以增加全局超时掩盖原因。
2. **先离线读准这局实际板。** 在图像内定位完整四角/ROI，按板平面纠偏，和已下载公开模板作同条件比对，保留第一/第二候选差距、多帧一致性和拒识；缺像素或两类难分就调整下一局观察位。当前观察可用于设计，**不能把上一局识别结果写死到下一局**。需要确定该物理板对应首ID，不能轮流发布ID/类别试分。
3. **下一独立单点版本在运行时识别。** 仅当本局首ID、外观察位与唯一fresh类别关联清楚，才调用 `recognition_phrase("ground", measured_category)`并发布一次合法`String.data`，保存原图、类别、话术和时间。保持非零UAV命令为零；不使用ACK字段猜测。若采用transport_only，日志明确只证明本队发送。
4. **进入该点并停止后声明。** 从已核对的外观察位进入该实际区域，实测停车后向 `/referee/current_target`发本局首ID；不在观察位提前声明到点。继续收公开识别状态与平台日志、保存声明前后odom，按当前已知合同结束本次有限实验。
5. **只以裁判证据判首点成功。** `10分/点`来自官方V1表3，不是本队预设成绩。拿到对应项被接受/计分或终局真实分数才记为首点计分成功。若平台仍“未提取到成绩”，应并列记录本地动作/发送证据与官方结算缺失；优先补结算合同，不把局部成功标全任务完成。

参考官方证据：`docs/referee_rules_v1.html`、`docs/referee_rules_v1.txt`，课程 https://www.splenach.com/courses/129d6e75-f19a-4c97-9ec6-0853695a4c48 。地面计分明确为“正确识别+进入点位”，导航中禁止非零 `/uav/cmd_vel`，全赛限速`.5m/s`；首点实验不需要先等穿窗/飞行/灯光全部接通。

## 从首点到整场仍必须实际测的项目

首点过程复用至三点且保持抽签顺序；每个点均留独立图/话术/位姿/计分。随后单独验证UAV命令坐标与起降、完整3D障碍/窗净空、窗前识别及实际下降穿越、灯开关/光束轴标定/`.8m`以上稳定中心照射`1–3s`、返航降落声明和结算。现有单元测试、候选几何和matched=1都不能替代这些响应。正式视频/PDF只能使用这些真实记录更新。
