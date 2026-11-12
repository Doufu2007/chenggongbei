# 三地面点返航与飞行接续：下一最小验收

2026-11-12。只读审计生产源码、`full_mission_bridge_20261112.md`、`flight_fov_diagnosis_20261112.md`、`active_state_20261112.md` 最新两段及固定公开候选世界。只新增本报告和离线分析文件；未改生产、未操作浏览器、未启动仿真、未读取私有裁判或隐藏答案。v38 正在验证、v37 高度实验由其他代理实施；本文不将其预写为通过。

**最近的接线缺口是第三地面点之后的真实返航。** v38 仍继承 `GroundThreePointAxisGuard._handoff`：`point_index==2` 时归档后直接 `stop('three_ground_transport_complete_official_score_unverified')`。完整 `MissionController` 虽有 `RETURN_GROUND→TAKEOFF`，它没有驱动当前三点 guard 的这条实际路径。先以独立扩展验收“三点→同一账本返航→双钟静稳”，再交给飞行；不重复 v37 的高度实验。

## 可直接复用的函数与最小改动边界

| 实际接口 | 已有能力及接续边界 |
| --- | --- |
| `ground_three_point_axis_plan._AxisBuilder`、`ground_three_point_plan._turn_clearance/_drive_clearance` | 已产生分离转向、正向轴段、`reverse_axis`，按原雷达前伸和 `.641m` 门检查静态候选。`plan_ground_leg` 仅接受任务ID，没有 home 路线接口；不能伪造一个任务ID作返航。 |
| `GroundThreePointAxisGuard._handoff`、父类 `_archive_point/_load_operations/_remaining_plan_fault` | 第三点声明发送返回且等待原2s后归档，再从**当前真实** `(x,y,yaw)` 规划返航，加载独立 `role='return'` 的 operations。沿用同一 guard、session、`started_wall`、`global_origin`、累计 path/angle、turn/drive计时及恢复账本。 |
| `GroundThreePointGuard._start_operation/_finish_operation` | 已顺序执行和记录每段实际起止；但 operations 耗尽时，`observation` 进 CAPTURE，其他 role 一律按旧任务点检查后进 DECLARE_WAIT。必须显式拦截 return 耗尽，进入返航静稳，不能落进旧分支重复声明。 |
| `GroundThreePointRecoveryBudgetGuard`、`create_camera_recovery_three_point_node(..., guard_factory=...)` | 可独立扩展类并注入现已审查的发布/回调适配。新 limits 需独立验证完整任务新预算，不能调用父类固定12.5m/28rad/1200s校验后再改字段，也不改冻结v38常量。 |
| `GroundBoundedApproachGuard._stable_dwell`、现发布边界/看门狗 | 已有源stamp和墙钟同时≥1s，速度≤.01m/s、角速≤.03rad/s、锚点平移≤.005m、四元数角变化≤.02rad；可用于返航停稳，但必须同时核查当前新鲜性、终点、预算与真实零命令返回。 |
| `MissionController._build_stages/_add_window`、`official_protocol` | 完整飞行顺序和声明编码可保留；不能切回未经本次路线验收的旧地面比例控制，也不能把guard的STOP当全任务完成。 |
| `protocol.planar_velocity`、`mission_node._publish` | 已有 world→body 的可配置转换。缺的是平台实测选择依据与增益/停距，不是缺一个旋转公式。未绑定的 `interface_profile.json` 仍 `verified=false`。 |
| `RuntimeInputs.pose_at_exposure`、`TemplateMatcher.detect`、`perception.camera_target_position` | 有原始前序曝光pose、原图参考匹配四角和已知矩形PnP；窗户应走竖直矩形观测，不能复用地平面IPM。还缺实际原图/K/D、frame/TF、物理尺寸与米制误差绑定。 |
| `target_geometry.localize_target/hover_for_downlight/assess_illumination` | 可复用空中点定位→灯位悬停→照射的三步计算；前者必须有曝光位姿，后两者只处理水平目标平面，不能拿去定位竖直窗框，也不证明沿途净空。 |

返航阶段禁用新识别任务、类别上报和 `current_target` 声明，旧worker context置失效；保留原三点证据和发送锁存。地面返航完成前所有 `/uav/cmd_vel` 保持全零、灯关闭。最小独立版本返航后仅停稳退出，不自动起飞、不发 `mission_completed`。

## 60种公开顺序的返航几何与独立预算

固定公开世界SHA256 `595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1`。每个名义第三点终姿态接：

`实际终点→(.6,实际y)→(.6,-.2)→朝南倒车至(.6,0)→朝西正行至(0,0)→沿南侧半圆转至yaw=0`。

各转向均由原 builder 选可行弧、分成≤π/2子转角。不能在 ground_2 南边界直接转朝南倒向北：朝南的雷达前伸会越过原 `.641m` 环扫门；先正常向北至 `(.6,-.2)` 才转向。也不能一直朝北开到y=0：北墙与前伸雷达只剩约 `.57m`。上述中间锚点处理了这两个确定冲突。

| 60种三点有序draw加返航 | 精确离线结果 |
| --- | --- |
| 最大名义路径 | **13.990000000000m**，`ground_2→ground_4→ground_3` |
| 最大名义累计转角 | **37.752432539827rad**，`ground_1→ground_3→ground_2` |
| 最大操作数 | 44；最多17个drive、27个turn |
| 全顺序最小名义laser净距 | **.650m**，高于原`.641m`门9mm |
| v25经验时长外推最大 | **1352.939406433s**，`ground_2→ground_1→ground_4`；计算`36.0058×path+22.9369×angle`，只是事前估算 |

建议下一独立实验固定总cap：**path=15.0m，rotation=40.0rad，global=1800s，turn累计=1000s，drive累计=650s**。相对上述名义最大分别保留1.01m、2.247567460173rad及447.060593567s；最多17段drive/27段turn的真实误差、静稳、识别与等待都须计入实际永久账本。余量是有限停止预算，不是对真实误差的连续保证；超过任一项即停，不临场扩容。原v38保持12.5m/28rad/1200s，不在原版本追加返航。

其他合同全部保留：地面`.447m`外廓、`.02m`净距、`.641m`环扫门，原局部`.06m`扫掠/`.03m`scan anchor/`.03m`停车余量，源与arrival`.20s`、曝光prior`.05s`、相机`.50s`、clock lag`.12s`，原速度/跟踪/倾角/单转45s等门。恢复仍全局32次、单次5s、累计60s，三点与返航不重置。平台外层超时未知，1800s本地cap不等于官网承诺运行1800s。

误差反例必须保留：在原终点半径`.02m`允许圆内取中心及四个轴向边界、yaw取`{- .025,0,+.025}`，共75个样本，72个候选可规划；ground_2向南偏`.02m`（y=-2.71）时三个yaw都被原初始净空拒绝（`.640m<.641m`）。因此“60/60名义通过”不能写成“2cm全误差通过”。实际返航每段仍须用当前pose和当前scan复核；9mm静态余量不够覆盖全部2cm横向跟踪容差时，应正确拒绝。

可复算文件：`runs/analysis/20261112_ground_return_plan/audit_return_geometry.py`；输出 `geometry_audit.json` 含60行结果、五个返航operation列表、75个误差样本和所读生产源码SHA256。命令：

```text
python runs/analysis/20261112_ground_return_plan/audit_return_geometry.py
```

## 返航后仍需真实观测的飞行与视觉约束

1. **水平控制合同。** 在保留固定静态空间/外廓/freshness的独立有限航段中，记录请求、实际DDS返回、原始odom与归零后的位移/累计路径/速度收敛。yaw≈0时body/world两种假设相同，单轴脉冲只能验方向和停止，不能确定坐标系；必须在合格非零yaw、且两预测差大于实测噪声时再作区分。非零yaw本身也需可观测旋转空间，不能删除点云偏航拒绝。
2. **高度与净空。** v37覆盖固定初始柱和完整云采样，不覆盖水平航段。复用 `CruiseCloudCapture` 的完整bytes/header、前序pose及真实TF；缺TF明确记缺。通用 `motion_clear` 仍要求当前云与pose严格同stamp，前向孔径也不覆盖全部近场；现`R+v×timeout`不含已量测制动项。不能仅改端盖距离、延长超时或将scan的+inf当未知空间自由。
3. **窗口观测位。** 公开候选窗1中心`(1.925,-.3275)`，洞口z`.40–1.15m`、宽1m；目标矩形约`.36×.22m`，位于`(1.815,-.3275,1.40)`。前视相机候选机体偏置`(.390,0,.130)`、水平约80°、800×800。这些只用于预测取景位；须实际图像/CameraInfo/TF和曝光pose核对，不能从候选纹理推类别。
4. **窗体与灯目标不同平面。** 窗图像中心/角点/平面应以当前竖直目标几何验收。`camera_target_position`只返回相机系中心，未自动提供窗框开口位置或可信米制界。空中地面板才适合`localize_target→hover_for_downlight→assess_illumination`；照射阶段无需维持前视居中，但必须有未过期TargetFix、当前完整pose、独立下视range和真实灯反馈。现三函数业务仍未接通。
5. **穿窗实际精度。** 当前候选含倾角/余量水平半径约`.485296542m`；1m洞加`.01m`中心容差后每侧只余约4.7mm。默认`.08m`容差应拒绝。地面2cm或高度保持成功都不能外推为水平窗孔精度，不能缩小旋翼或外廓来放行。

## 下一项最小有判别力的真实实验与验收

**下一局只扩展“三点成功后真实返航”，保持无人机全零。** 它直接区分“当前三点终态无法接续”和“同一任务账本可返回合法起飞位”，不会把尚未验收的水平飞行、窗识别或照射混进同一个结果。v38正在运行的原包不改，独立扩展需先专项回归和非作者审查。

通过必须同时满足：

- 三点各有本局独立三帧、准确一次识别和准确一次到点transport；从第三点真实终姿态生成并实际执行返航operations，途中新识别/声明数为0，所有飞行命令和灯始终为0/false。
- 原路径、转角、时间、恢复次数/累计暂停在第三点→返航边界连续，无新guard、无重锚、无预算清零；源龄/arrival/净空/姿态/局部停车门非零命令违规数为0。所有静态规划拒绝均真实停止，不能替换抽签或继续猜路径。
- 最后真实`hypot(x,y)≤.02m`、`|wrap(yaw)|≤.025rad`，同一新鲜稳定段内源stamp与墙钟都≥1s；速度≤.01m/s、角速≤.03rad/s、锚点平移≤.005m和姿态变化≤.02rad。任何坏帧重置驻留；最后归零发布返回、watchdog和进程退出单独留证。
- 回传完整性、源码/脚本/包SHA与真实轨迹独立重算通过；官网终态和分数另存原始证据。本地返航通过只解锁后续飞行交接，不等于完整巡检、穿窗或照射得分。

独立扩展验收后，飞行的下一缺口才是固定安全空间内短水平响应及同帧视觉数据；已有v37完整高度试验不在此重复。
