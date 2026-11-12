# 地面单点：控制与进入阶段独立复核

2026-11-12。只读审查冻结 `ground_single_point.py`，SHA256 `7f1ee2e6a4dfaeb9f05a557be4ea398f6dc121cf2bda2708664e0606c38845f0`。审查未改生产源码、共享控制、profile、正式测试或实测回传；新文件仅位于独立审查目录及本文档。

独立反例通过 **14 passed / 17.70s**，命令：

```text
python -m pytest -q runs/review/20261112_single_point_control/review_control_cases.py --junitxml=runs/review/20261112_single_point_control/independent_final.xml
```

## 核对结果

- 对允许进入的 `ground_0/ground_1/ground_4`，观察后 `_begin_entry` 替换局部路线时，永久起点、已走路径与已转角不重置；整段进入轨迹中原路径/旋转/起点位移预算保持不变，累计量单调，最后停在候选区域。
- 用真实 guard 的 odom 解析器输入反复小幅正负偏航，净偏航仅约0.01rad；实际累计绝对转角到冻结上限时，返回 `approach_rotation_limit` 并全零停止。没有以计划转角或首末净转角替代实测累计量。
- 最终观察转向的 `TURN_SETTLE` 若实际位置已超过目标观察位2cm门，在重新完成静稳后、进入 `CAPTURE` 前以 `route_waypoint_error_limit` 停止；期间输出全零。独立转向漂移合同为6cm，超过该值则在 odom 回调中以 `approach_turn_displacement_limit` 立即锁停，不能混同两个门限。
- 三个允许首ID的进入末腿均经过 `DRIVE_SETTLE` 的双时钟静稳。切入 `DECLARE_WAIT` 的那一tick仍全零，保留末腿停止路径锚，清空直行局部状态；不启动额外最终转向。其后仅一次到点发送，等待结束仍全零，结束原因保留官方得分未核实。
- `DECLARE_WAIT` 即使已经完成静稳，也不能绕过当前传感器年龄：仅把调用墙钟推进0.201s、保留原传感器stamp，声明被拒且没有attempt。
- `REPORT_WAIT` 的 clock/odom/scan freshness故障均硬停。另用真实逐步推进clock/odom、停止发送scan的输入重现超过原年龄门：原scan stamp未改，以 `single_point_report_wait_sensor_freshness_lost:scan_stale_or_unsafe` 停止，不进入、不声明。已报告后恢复输入也不能复活该次进入流程。

## 审查断言修正

保留的 `independent_initial.xml` 为4 failed / 8 passed，失败来自独立断言预期：错误原因实际带原故障后缀；初版把最终观察位2cm门误当作转向漂移6cm门。修正测试后又明确将位移施加到“远离目标”的方向，确保测试输入实际超出2cm门。没有因此修改生产代码，也未把审查fixture错误记为产品缺陷。

本轮在冻结版本上没有发现可复现的控制/进入阻断。测试使用既有非单位执行器示例和合成传感器输入，验证控制合同；没有运行ROS、真实SIFT、平台仿真或裁判发布，不证明真实地面跟踪、官方类别、完整机身进入或实际计分。实时图像绑定与证据解析由另一路独立复核。
