# 目标几何接线审查与最小实施顺序

更新：2026-11-12。本报告先只读核对源码与 [飞行合同审计](flight_contract_audit_20261111.md)，后按授权完成下述第一步本地接线与专项测试；未联网、未启动仿真。工程中没有 `controller.py`；实际任务控制器为 `mission.py:MissionController`，其位姿类型来自 `control.py:Pose2D`。当前平台实测目标是有界地面动作，不能将其外推为空中飞行、相机外参、地图或照射已验证。

## 当前结论

`target_geometry.py` 已提供 `localize_target`、`hover_for_downlight`、`assess_illumination`，但业务代码没有导入或调用它。纯数学模块尚未接通里程计、曝光位姿、视觉测距、路径门禁或任务阶段。最小可行顺序是：**先无损保留公开输入并提供有界曝光匹配，再绑定有证据的外参与误差，最后将空中识别、迁移和照射分成不同阶段。** 未取得合同前，仅能完成数据旁路和拒绝条件，不能声称完整几何飞行闭环。

## 数据链中的实施前断点

| 位置 | 当前实际行为 | 最小修复 |
| --- | --- | --- |
| `runtime.py:parse_pose/update_pose` | 检查 header 世界帧与四元数范数、倾角，再只保留 xyz/yaw/stamp。`child_frame_id`、完整四元数、twist 被丢弃；只有最新位姿。 | 保留现有 Pose2D 接口供地面控制，另建完整 odom 旁路，保留原 header/stamp、child frame、四元数与 twist。几何调用必须核对明确的 child frame；不能将 `base_footprint`、`base_link`、`uav/base_link` 直接视为相同。 |
| `vision_node.py:_camera_info_cb/_image_cb` | 缓存宽高、K、D、frame；图像用真实曝光 stamp 输出，PnP 可添加 `position_camera_m`。没有标定/尺寸版本、米制误差界、曝光位姿。 | 延续真实曝光时间；显式携带相机 frame、坐标约定、标定与尺寸版本、米制误差及关联物理点。没有这些字段时仍可做类别识别，但不能创建 TargetFix。重投影像素阈值与识别 confidence 都不能当作米制误差。 |
| `runtime.py:update_detections` | 保留 detection item 的额外字段，但只将外层 stamp/source 传入 item；外层 `frame_id`、图像尺寸、标定元数据没有留在 `frames`。 | 保留并校验明确传入的外层字段；将帧/版本归属传给几何适配，拒绝来源冲突。未知字段保留原样用于诊断，不猜测缺失值。 |
| `target_geometry.py:localize_target` | 要求曝光匹配的 `StampedPose3D`、时钟/epoch、外参与版本、保守误差界。 | 由 runtime 的短历史匹配曝光时间。禁止以最新 odom、处理时刻或固定单位四元数替代；未知误差不填 0。 |
| `mission.py:_inspect` | 识别报告 stamp/pose 为报告产生时刻的当前值，`_reported_aerial` 只保留 `(point_id,target_id)`。 | 报告时刻保留作传输上下文，另外保存曝光 TargetFix。报告时间、曝光时间、当前控制时间分开命名；TargetFix 绑定本局物理点、类别和版本。 |
| `runtime.py:update_light` | Bool 解码后用本地接收时刻作为 `light_stamp`，没有传感器源时间、epoch、反馈版本。 | 反馈无 header 时显式记录接收时间依据，须有传输延迟上界才能构造可用的 StampedBool。不能把命令发送或本地计时当作灯已实际开启/命中。 |
| 下视测距 | 当前 scan 缓存没有独立 `StampedRange` 适配，水平避障与向下测距用途未分开。 | 真实确认后将下视束的有效距离、量程、原 stamp、frame 与独立外参转为 StampedRange；不能把一条 range 当机体高度，也不能将下视窄束改名为水平雷达。 |

完整四元数的保留与允许三维运动是不同层次：旁路应保留真实姿态，现有平面运动控制仍须维持倾角限制。`target_geometry` 的单位四元数校验比旧 runtime 的范数容差严格，接线时应明确拒绝或经有界、可追溯的规范化；不能把旧 Pose2D 的 yaw 重新生成四元数冒充原姿态。

## 曝光位姿的最小合同

为每个来源保留有限条数、有限时间跨度的 odom 历史，记录世界 frame、真实 child frame、原 stamp、完整四元数、twist 与运行 epoch。无效更新、时钟倒退或 frame 改变应使旧几何证据失效；不同 run/epoch 的数据不能拼接。

第一版采用**曝光时间之前最近一条**合格姿态，要求 `0 <= exposure_stamp - pose_stamp <= max_skew`，同时满足历史保留跨度和明确时钟域。没有满足条件的姿态就返回明确拒绝原因；不取未来姿态、不外推、不重新打时间戳。`localize_target` 已计算残余偏差对应的平移/旋转误差，因此调用者还必须提供覆盖该时间段的运动速度上界，不能只拿一帧 twist 当保证。

若之后增加插值，应显式选择同 frame/epoch 的前后包围样本、平移插值与四元数 SLERP，并保留原样本时间和额外误差；该功能无需阻塞第一版。保留 API 建议为 `pose_at_exposure(source, stamp, expected_child_frame, max_skew)`，未确认 child frame 或无有效样本时拒绝。返回的是历史证据，尚不能直接视为 TargetFix。

## 识别后飞到目标上方，再照射

现有 `_build_stages` 对空中点的 `AIR_NAV`、`AIR_RECOGNIZE`、`AIR_INSPECT` 使用同一个固定 `point.pose`。`_illuminate_reported` 又要求前视图像持续新鲜且中心对准；这与下视灯位于地面目标上方的几何关系冲突。`target_geometry` 允许使用未过期的缓存世界定位，正是解决该矛盾的计算基础，但它不会自动生成安全路径。

后续最小阶段链应为：

1. `AIR_RECOGNIZE/LOCALIZE`：在可见观察位取得真实类别和合格 TargetFix，按实际识别合同上报；没有世界定位或确认条件就不推进。
2. `AIR_MOVE_TO_LIGHT_POSE`：用 `hover_for_downlight` 计算补偿灯偏置的机体悬停位姿。高度参数是世界 z，最小高度是相对目标平面的机体原点高度；不能直接把世界 z=0.8 或 range=0.8 当成满足规则。新目标经过实际路径/净空检查后才允许移动，全程灯关闭。
3. `AIR_ILLUMINATE`：使用当前完整位姿、原曝光 TargetFix、独立测距和真实反馈调用 `assess_illumination`。允许目标离开前视画面，仍必须满足身份/point/epoch、定位年龄、漂移、误差预算与悬停稳定性。只有 allowed 的连续时段计入 1–3 秒；丢失条件立即灭灯、清空本次累计，重新观察时生成新的真实定位，不能刷新旧时间。

`hover_for_downlight` 只计算名义悬停点；`assess_illumination` 只评估射线与水平目标平面的一致性。下视 range 的足点必须位于同一个已确认平面，目标平面与机器人高度参考也须有证据。这两个函数都不能证明沿途无碰撞或裁判已给分。原始识别话术不应因新增阶段而重复刷发。

## 按拟运动方向检查净空

已有 `mission_node._tick → RuntimeInputs.motion_clear` 使用**本 tick 将要发布的命令**，地面线速度会按当前 yaw 转成世界方向；这部分应保留，不能退回固定前方检查。当前仍有以下边界：

| 动作/输入 | 当前边界 | 最小可执行处理 |
| --- | --- | --- |
| 有界地面直行 | scan 基于自身 stamp，但检查使用最新 yaw；只有 yaw_offset，没有雷达到机体的平移补偿。 | 先限定低速、短距离、直行且角速度为零。实际确认 scan frame/安装偏置和运动方向；用 scan 时刻姿态变换测点，或将可证明的时序误差与平移偏置加入保守扫掠包络。过期或方向覆盖不足即停。 |
| 地面同时转弯前进 | 目前线性 corridor 使用瞬时方向，不表示整个停止过程的弯曲轨迹。 | 第一版把旋转与平移拆成两个受门禁动作；随后才实现曲线扫掠或以保守包络覆盖制动期间曲率。不能用“检测了前方”宣称所有转弯方向安全。 |
| 纯偏航 | LaserScan 分支检查四个方向；前向点云分支拒绝任何偏航。 | 明确真实整机旋转包络与传感器覆盖。没有完整覆盖的旋转区就拒绝；不删点云偏航拒绝分支来让控制器动起来。 |
| UAV 垂直/后退/穿窗 | 前向点云不覆盖上/下/后方，当前端盖视场检验也无法证明全部近端扫掠体。 | 取得对应方向真实观测，或经过当前平台版本核对的静态自由空间证据，再融合水平 scan/前向点云/下视 range。当前未知时继续拒绝，不能借本轮地面动作结果放行。 |
| 停止距离 | 当前 `R + speed × timeout` 包含延迟项，但没有明确制动响应上界；水平半径被复用为三维圆柱半径。 | 用已确认时延与制动上界计算所需路径长度，分开水平外廓和上下包络。任何优化都不得缩小真实机体尺寸来绕过门禁。 |

净空拒绝应记录“实际障碍”“样本过期”“方向未观测”“外参未确认”等可区分原因；单一 bool 可以继续作控制门禁，但诊断必须保留原因。有限返回点只是障碍采样，三点在远处不自动证明整条三维通道为空。

## 可以立即做与仍需实测的边界

可以先实现：完整 odom 旁路、明确 child-frame 条件、无效/时钟回退清证据、有界曝光历史、检测外层 frame/版本字段保留，以及这些纯数据接口的拒绝路径。维持旧 Pose2D 调用方式和现有默认 `verified:false`，不触碰本轮有界地面动作的授权范围。

后续才能绑定：相机光学轴到真实 odom child 的变换、CameraInfo/尺寸版本、米制误差、灯/测距独立外参、反馈时间语义、当前场地图和运动自由空间。数据旁路 API 完成应报告“已保留完整公开输入并可按曝光匹配”，不能报告“飞到目标上方照射已实现”或“已获平台分数”。

建议下一轮验收依次为：旁路字段与拒绝条件 → 实际平台同一时钟域的图像/odom 关联 → 地面有界前进/停止及方向门禁 → 有证据的目标世界定位 → 有证据的空中安全迁移 → 照射与裁判计分。每一步保留输入时间、frame/版本、拒绝原因及真实结果，避免用离线数学通过代替平台验收。

## 第一步实施与专项结果

本轮新增 `geometry_runtime.py` 并接入 `runtime.py`：完整保存原 odom header（含原始 sec/nanosec）、四元数、child frame、twist 与输入副本；保留旧 `Pose2D` 和倾角门禁，没有放宽控制合同。短历史同时受条数和时间跨度限制，`pose_at_exposure` 只返回曝光之前最近的原始样本副本，要求调用者明确确认 child frame、最大偏差和最大年龄；不取未来样本、不插值、不外推、不重打时间。

坏姿态、无效 twist、stamp 倒退、同 stamp 变数据或 frame 改变会清该源几何历史和当前相机帧；观测到 runtime `now` 倒退时清双方历史、旁路和帧。检测保留显式外层 frame、标定/尺寸版本、坐标约定、clock/epoch、CameraInfo stamp、图像尺寸和米制误差字段，并拒绝非法值及内外层冲突；缺失字段保持缺失。

`python -m pytest tests/test_geometry_runtime.py tests/test_runtime.py -q`：**55 passed**。专项覆盖输入/旁路/查询返回值副本隔离、条数及跨度裁剪、前序曝光匹配、未确认 child、过旧/未来/skew 拒绝、异常后清证据、时钟从 pose/camera/query 任一入口倒退以及旧平面接口兼容。全套回归由主代理统一执行。

完成范围是**完整 odom 旁路和曝光匹配 API**。没有构造 `StampedPose3D` 或 `TargetFix`，没有生成缺失的 clock/epoch、外参或误差界，也没有接入飞行迁移、灯光、测距、净空或裁判计分。以上实现不能作为平台任务已完成的证据。
