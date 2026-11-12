# 独立轴向三地面点运行器设计

主控已明确授权本实现，不改冻结v25/v26或两份planner。v26引入名义斜线，真实运动方向不符；新运行器显式使用已独审`ground_three_point_axis_plan.py`（SHA `e1bb65499f24eba7389b645dd29c10539bd7942fa51675d57912d95454deba8e`）计算首点、视觉后entry和点间handoff，保留原观察位、终点中心/朝向、视觉/声明门及真实传感器最终停止权。

新增`GroundThreePointAxisGuard`与`AxisThreePointLimits`，继承v26的图像、epoch、任务声明、配对、双时钟及累计账本。新固定累计转角28rad（名义最大26.703538rad及预留）；其余仍为path12.5m、起点半径3m、总1200s、turn累计700s/单45s、drive累计500s。所有原净空、传感器年龄、速度、heading/cross-track、.03m局部停止预留、归零后累计.01m、停稳及宣告门保持。局内不扩容。

`DRIVE_REVERSE_TO_XY`是独立操作，目标距离保持正。机体朝向为`atan2(destination-current)+pi`，`drive_yaw`始终保存该机体朝向供原heading保护；仅反向段`measured_forward`取原投影的负值，cross-track绝对界不变；仅当前反向段线速度请求取负，仍按.03/.10慢速和到点减速协议。每次操作开始先清reverse flag，forward、turn、capture、handoff不能继承反向符号。TURN/ TURN_SETTLE在后继为反向时都用反向目标机体朝向重瞄并复核原候选转弧。

适配层直接通过`create_bounded_node`的guard factory创建新guard，复用原订阅、QoS、图查询、计时与watchdog；独立重建tick和真实视觉coordinator，不委托旧`original_tick`发布。全部图查询完成后再次查询真实DDS匹配，再按当前wall重新step并复核预算、source fresh、phase与幅度。实际callback→tick必须把反向命令发布到`/cmd_vel.linear.x`并记录有符号值。publish异常吸收STOP，独立尝试零命令；零发布异常不逃逸，watchdog后续仍能再次尝试零。真实coordinator→dispatch→String publisher计数保留，成功返回才增加returned。飞行、灯和完成输出继续静默；识别/到点仍仅由真实当前epoch和停稳门授权。异常/失联/预算耗尽后STOP不复活。

新增模块、LF启动脚本、专项测试与报告。先必要RED再实现；专项覆盖真实signed接缝、全三点handoff的axis接线、entry视觉门/过期epoch、预算与失联停止。只跑相关检查；完成冻结供另人独审，不打包上传。负X标定尚待平台确认，软件通过不代表反向实际可用或完整成功。
