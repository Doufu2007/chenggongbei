# 三点轴向候选 planner 独立审查

结论：**Approved（仅纯候选几何）**。未发现阻断项；禁止把此结论解释为执行器已支持倒车或平台完成三点。

被审源码：`src/inspection_mission/inspection_mission/ground_three_point_axis_plan.py`，SHA256 `e1bb65499f24eba7389b645dd29c10539bd7942fa51675d57912d95454deba8e`。

独立脚本与结果：`runs/review/ground_three_point_axis_20261112/independent_probe.py`、`independent_metrics.json`。

没有重复作者名义60顺序和每段501点检查。新增675个来自真实起始位姿的小扰动候选（XY各±3mm、yaw±.008rad），独立核验每次操作的前后锚连续、转角符号/终姿一致、每平移仅单世界轴、正反向投影符号、分段长度/转角总和、观察边界和最终位姿。675个均通过，包含135个显式反向段；非法南偏出发`(0,-2.71,pi)`被拒绝。

源码继续保留`candidate_unverified`状态和静态几何来源，只按公开ID选位，不读取隐藏类别。ground_2反向段使用独立`DRIVE_REVERSE_TO_XY`，没有伪装成旧执行器已支持的正向段。实测负X响应、制动、跟踪误差和实时scan仍是将来执行的必要门。

设计报告已明确最大累计转角26.703538rad超过旧运行25rad合同，故**不得直接接旧运行器**；应先完成方向响应及独立倒车实测，再评审明确的新固定预算。当前没有接ROS、没有上传此planner，名义最小环扫余量9mm不能证明实际成功率。
