# 首点外观察路线：纯数学 API

```python
from inspection_mission.ground_observation_route_plan import plan_ground_observation_route

plan = plan_ground_observation_route("ground_2", (actual_x, actual_y, actual_yaw))
```

输入一个本局公开首 ID 和真实 `(x,y,yaw)`；仅接受有限数值、XY范数≤`.05m`、`|yaw|≤.05rad`。返回冻结 `GroundObservationRoutePlan`：

- `initial_pose`：真实输入起点；`waypoints_xy_m`：绝对候选地图目的点，不含起点。
- `segments`：各段 `start_xy_m/end_xy_m/heading_rad/length_m`；`target_xy_m/final_yaw_rad`：首板中心及末朝向。
- `total_path_m/planned_rotation_rad`：累计路长及含初始转向、段间转向、末朝向的累计角。
- `budgets`：路径=`计划+.30m`，原点峰值=`相对真实起点的航点最远距离+.15m`，累计角=`计划+.40rad`。240s及分段墙钟明确标为 `provisional_pending_platform_measurement`，不假设控制增益。
- `candidate_projection`：假设末位姿的四角、像素包围范围和800px内可见性；`target_board_clearance_m` 是机体中心到板矩形距离，尚未减机体半径。

固定地图/外参保持 `candidate_unverified`，投影为 `prediction_not_platform_observation`，计划为 `candidate_plan_not_execution`。调用只做标准库数学，不读写文件、不导入ROS/控制模块、不输出速度、类别、消息或成绩；消费者仍须独立验证真实传感器、局部净空、执行窗和运动预算。

验收：17项先因模块缺失失败，再17通过；与现有 `test_target_geometry.py` 合计 **68 passed**。父代理要求并行开发期不跑全套，本次未运行全套。

冻结源码 `src/inspection_mission/inspection_mission/ground_observation_route_plan.py` SHA256：`a7ba6bf9cce1af13b9997de5c4ec82b02074fde36e4ae4575fe48e4fd2f633c8`。

测试 `tests/test_ground_observation_route_plan.py` SHA256：`04aee318fb0f0c90b01f9fa2470a68bac2c757539319cb5a8acfb4a9d7bff90f`。
