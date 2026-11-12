# v23：首ground_1三曝光一致几何候选

冻结GroundPlaneMatcher对三张真实UAV CAPTURE各保留1个trapped_people公共参考注释候选。每个候选经自身原曝光前序pose转到world，均落在本局首ground_1的候选矩形内。三中心最大两两距离约0.331mm；对固定候选ground_1中心(0,-1.35)m有约3.2cm整体偏差，不能把模型外参或该ID几何关联当作官方测量真值。

| 原阶段 / 曝光stamp | 候选world xy(m) | inlier | 当前首ID区域 |
|---|---|---:|---|
| capture_0 / 98.724 | (.028929,-1.363209) | 100 | ground_1 |
| capture_1 / 99.058 | (.029068,-1.363310) | 108 | ground_1 |
| capture_2 / 99.391 | (.029241,-1.363319) | 106 | ground_1 |

这些是3张不同SHA、递增stamp且各自prior有效的真实曝光，不是同图旋转或测时重复。全部12模板分别逐帧匹配，共36个结果；没有板ID选类、人工ROI或阈值改变，另11模板每帧均拒识。原图、K/零畸变CameraInfo、source pose四元数/stamp、全部拒绝原因/模板hash/provenance/所有候选位置均在`report.json`。

输入来源`../20261112_route_v23_independent/summary.json`；实际公开顺序首ID为ground_1。只选择capture_0/1/2，不用first/last/缩略图补样本。冻结模块SHA仍为`1b9c9b3cd658f5192307161dbbe21954968e9680c98424250d2d0836e1dc710d`。

每图仅单次调用：match2602.68/2919.76/2733.39ms，模板一次冷加载7908.16ms；不把3帧当性能重复分布或准确率。`scores.csv`保留36行，3张BEV与3张原图overlay保留像素证据。

结论仅为当前首点的三曝光一致候选识别与空间关联。它不是官方类别真值、在线识别上报成功、裁判接受或得分；尚需控制层根据实时静止epoch/原图关联/当前传感器时序和实际官方反馈建立闭环。本次没有改生产源码/门槛或旧结果，没有发布或上传。
