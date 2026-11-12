# 三地面点纯 planner（2026-11-12）

新模块 `src/inspection_mission/inspection_mission/ground_three_point_plan.py` 只计算候选几何操作，不导入 ROS、不读文件、不依赖分析目录，也不产生识别、裁判或控制授权。

```python
plan_ground_leg(previous_id: str | None, target_id: str,
                actual_pose: tuple[float, float, float]) -> GroundLegPlan
plan_ground_entry(target_id: str,
                  actual_pose: tuple[float, float, float]) -> tuple[GroundOperation, ...]
```

`GroundLegPlan` 是 frozen dataclass，包含 `previous_id`、`target_id`、`initial_pose`、`observation_xy_m/yaw_rad`、`endpoint_xy_m/yaw_rad`、`path_m`、`rotation_rad`、`minimum_laser_distance_m`，以及 tuple 类型的 `to_observation_operations` 与 `entry_operations`。来源 SHA 和 `candidate_unverified` 状态随对象保留。

`GroundOperation` 也是 frozen dataclass：`kind` 为 `TURN_ONLY` 或 `DRIVE_TO_XY`；`anchor_xy_m` 为该操作起点；`target_xy_m` 在纯转向时为 None；`expected_start_yaw_rad`、`target_yaw_rad`、`signed_subturn_rad` 明确指定转向方向，不能再用目标位置的 atan2 代替纯转向。另含 `length_m`、`source_phase`、`minimum_laser_distance_m`、`settle_required_after=True`。兼容属性为 `heading`、`turn_direction`（CCW/CW/None）、`rotation_rad`。

每个 TURN_ONLY 不超过 π/2 且要求操作后停稳。入口执行前应在当前视觉授权成立且车辆仍在观察站时调用 `plan_ground_entry`，用真 pose 重算入口；函数本身不检查识别或观察站身份。静态不能许可的起点、弧或直线抛出 ValueError。名义 g2 离站自动选择北半圆，等价 ±π 及 3π 初始 yaw 都已验证；y=-2.71 的南偏例会拒绝。g0 保留已计分轴线，g4 观察位固定 (.35,-.10)。

`BOXES` 是来自公开候选 world、激光实际 z=.135m 平面的 10 个不可变矩形常量；来源 world SHA256 `595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1`。连续直线/圆弧按有限解析候选求最小距离，并与独立稠密 pose 采样交叉验证。静态几何许可不能取代实时完整 scan 环、freshness、速度、跟踪误差或全程累计预算。

名义最小激光间距 .650m，相对原 .641m 门仅 **9mm**。运行时 2cm 跟踪误差可能安全拒绝；此模块不保证三点可执行完成，也不改变 .447m 半径或 .114m 传感器偏置上界。

TDD 先观察 17 项因缺少模块而失败，再实现后通过。专项测试含全 60 顺序、不可变对象、实际起点、入口重算、非法输入、g2 双圆弧反例与独立稠密采样；连同旧 observation planner 测试共 35 项通过，py_compile 通过。25 条名义首段/换点的 path、rotation、最小净空与原设计报告一致至 1e-11，操作数量一致。机器清单位于 `runs/analysis/20261112_ground_three_point_time/planner_manifest.json`。

只新增此模块及专项测试，未改旧 single-point / route / matcher / guard，未上传或运行真实机器人。
