# 目标几何独立审查（2026-11-11）

范围：只读检查 `target_geometry.py`、专项测试及 `2026-11-11-target-geometry.md` 的 Task 1。当前检查点的专项命令 `python -m pytest tests/test_target_geometry.py -q` 返回 43 项通过。数学专项通过不代表平台定位、运动或计分成功。

已核对完整四元数坐标链、光学轴约定、版本/帧/时钟/重置检查、图像与位姿时间差的速度误差、独立灯和测距外参。测距按照位姿与测距时间差增加线速度和角速度误差；未将测距读数当作机体高度。

初审发现两个需要修复的约束边界：

1. **倾角只检查名义值。** 使用现有测试的 `illumination()` 数据，将灯外参绕 y 轴旋转 `0.19 rad`，其 `rotation_error_rad=0.02`，`max_down_tilt_rad=0.2`；重新计算名义悬停点，将当前位姿移到该点并令运动速度界为 0。`assess_illumination` 返回 `allowed=True`、`total_error_m≈0.03651498`。外参允许的实际倾角可达 `0.21 rad`，超过硬限制。应同时约束灯与独立测距射线的名义倾角加姿态/外参/时间引起的总角误差。
2. **累积漂移后未重新限制目标误差。** 使用现有测试的 `illumination()` 数据，设置 `max_target_error_m=0.001`、`localization_drift_mps=0.003`，并将 `max_total_error_m`、`hit_radius_m`、`max_range_residual_m` 各设为 `0.1`。目标曝光后 1 秒，累积目标误差为 `0.003 m`，检查仍返回 `allowed=True`、`total_error_m=0.006`。应在累加时效漂移后再次应用 `max_target_error_m` 硬限制。

修复后复查通过：灯与独立测距均检查名义倾角加位姿、外参及时间差引起的完整角误差；悬停求解在尚未取得实时位姿时也检查已知灯外参角误差。目标累加时效漂移后重新应用硬误差限制。新增 8 项回归覆盖灯/range 的外参、位姿、运动三种角误差来源，以及漂移和悬停限制。

复查命令 `python -m pytest tests/test_target_geometry.py -q` 返回 **51 passed in 0.18s**。另独立验证角边界：名义 `0.18 rad` 加角误差 `0.02 rad` 在 `0.2 rad` 限制下灯/range 均可通过；名义 `0.19 rad` 加相同误差分别被 `light_tilt_limit`、`range_tilt_limit` 拒绝。未发现新的实质问题。此结论仅限纯数学 Task 1，不代表平台传感器已标定、控制器已接线或官方任务已获分。
