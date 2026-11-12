# 巡检目标定位与下视照射实施计划

> **For agentic workers:** 使用 subagent-driven-development；用户已授权补全并自动执行，不追加审批。

**Goal:** 为“前视定位后飞到目标上方照射”提供可独立验证的几何与数据有效性计算。

**Architecture:** 新增纯 Python 数学模块，输入显式光学到机体外参、曝光时刻完整位姿和误差界，输出带来源和时效的世界目标定位。悬停求解和照射检查使用独立灯/测距外参。当前不接入运动控制器、不设置平台默认外参，也不修改 verified。

**Tech Stack:** Python 标准库、dataclass、pytest。

**Spec:** docs/flight_contract_audit_20261111.md；本计划中的接口及约束。

## Global Constraints

- T_AB 将 B 系坐标变换到 A 系；相机点明确使用 OpenCV 光学轴：x右、y下、z前。
- 使用曝光时刻完整四元数位姿，核对世界帧、机体帧、相机帧、时钟域和重置标识。不能用最新位姿替代历史匹配位姿。
- 外参、标定/尺寸版本及米制误差界显式必填；不把像素误差当米制保证。
- 未知输入、非有限数、非单位四元数/方向、目标在相机后方或光线后方、超时/错帧/重置均拒绝。
- 没有控制输出或 ROS 发布；数学通过不代表平台定位或计分成功。

## Review Focus

- 非零 roll/pitch 与偏航复合时，不能只旋转xy。
- 图像位姿同步误差应计入速度和角速度下的保守误差界。
- 灯与测距各自的安装位置和方向不能混用；距离不能当机体高度。
- 目标离开前视画面可使用尚有效的世界定位；过期或误差过大时不允许照射。
- 光线与目标平面近乎平行、方向向上、量程或观测与目标不一致时拒绝。

## Task 1 纯几何模块

**Files:** 新建 src/inspection_mission/inspection_mission/target_geometry.py、tests/test_target_geometry.py。

**Interfaces:** 定义 CameraObservation、RigidTransform、StampedPose3D、LocalizationLimits、TargetFix、LightExtrinsics、StampedRange、StampedBool、IlluminationLimits、HoverSolution、AimAssessment。提供 localize_target、hover_for_downlight、assess_illumination，详细类型字段由实现者围绕显式合同确定并文档化，避免重复字段。

- [ ] 先用手算光学轴交换、平移、90度偏航与非零倾角建立失败测试。
- [ ] 实现 p_W=R_WB(R_BC p_C+t_BC)+t_WB；验证版本、帧、时间和误差预算。
- [ ] 实现按目标世界高度平面求下视灯射线悬停位置，补偿灯偏置；验证方向与正向交点。
- [ ] 实现新鲜位姿、独立测距与灯反馈条件下的误差/命中检查；返回是否允许继续照射及明确原因。
- [ ] 覆盖同步/时效边界、重置、坏四元数、超差、测距过期/不一致、灯偏置和前视目标已离画面等测试。
- [ ] 跑专项，记录命令与结果；另一代理只读审查后修复实质问题。

## Task 2 后续平台接线（须取得真实合同后执行）

- [ ] 保留检测 frame_id 与相机坐标，接收曝光匹配完整里程计。
- [ ] 用实际标定、目标尺寸、相机/灯/测距外参绑定该模块。
- [ ] 接通“识别定位→悬停→照射”状态；新航点仍需路径和障碍检查。
- [ ] 用官方逐项计分、真实图像与录像验证，未验证前不修改平台就绪状态。

## 运行状态

- 已提交只读诊断 v5：run_20261111_222348_ef5c04b0；等待真实公开相机与抽签数据。
- 本计划 Task 1 只解决数学合同，不能解决当前垂直空间观测不足或取代平台任务验收。
