# v20 原UAV图：冻结GroundPlaneMatcher离线核验

全部12模板均拒识，**没有可上报的几何候选，更没有本局首ID ground_2 的识别证据**。未改生产源码、门槛、旧五图或平台状态。

## 源图、实际位姿和K

- 输入来自已完成独立解析的 `../20261112_route_v20_independent/summary.json`。latest UAV native 与 last_associated 是同一张，按SHA/stamp去重后只处理1张，不用first或缩略图补样本。
- 原图 `native_L000013_13d20f2020c94981.jpg`，SHA256 `13d20f2020c94981`为文件名前缀，完整hash见report。曝光stamp65.379，frame `uav/camera_link`；前序UAV odom stamp65.36，差0.019s，原解析为valid_prior_pose_observation。
- 实际body位置 `(0.58684955,-0.01111298,0.05500007)m`，yaw `-1.50283189rad`，完整四元数与原pose均保留。没有使用计划观察位代替实际位置。
- 原始CameraInfo首末K一致：f=`476.7030836014194`，cx=cy=`400.5`，800×800，畸变项0。实际完整CameraInfo保留。
- 本局公开顺序为 `ground_2,ground_3,ground_1`。只完成首腿约+.591m，第二TURN超45s停止，尚未到ground_2最终观察位。

## 固定匹配与拒绝证据

模块SHA256 `1b9c9b3cd658f5192307161dbbe21954968e9680c98424250d2d0836e1dc710d`，与上次冻结一致。显式加载同12公共模板及原SHA/label/provenance，保持候选height=.173m、固定IPM与全部SIFT/几何门。

| 公共参考 | good / inlier | 不同query xy | template凸包覆盖 | inlier/good | 主要拒绝原因 |
|---|---:|---:|---:|---:|---|
| collapsed_ruins_02.jpg | 60 / 19 | 14 | 6.68% | .317 | template覆盖<15%；query凸包/投影仅6.61% |
| rescue_camp_02.jpg | 72 / 15 | 13 | 13.75% | .208 | template覆盖<15%；比例<.25 |
| trapped_people_01.png | 83 / 14 | 13 | 9.20% | .169 | 覆盖/比例不足；query凸包/投影7.66% |
| rescue_camp_01.png | 121 / 20 | 1 | 11.18% | .165 | 单点退化等多门失败 |

其余8模板也均失败，完整结果在`report.json`与12行`scores.csv`。同帧测时重复3次输出完全相同；它们不是3张新曝光或稳定多帧证据。

## 区域核对仅为拒绝诊断

匹配完成后，将每个H的中心按原曝光UAV四元数/位置转到world，保留到所有5个固定候选中心的距离以及是否在候选矩形内。参考类别不由ID选择，模型ID映射仍为candidate_unverified。

三个有凸BEV投影但被拒识的结果，分别落在候选ground_4、ground_0、ground_1附近：

| 被拒绝的参考H | 推断world xy(m) | 与本局首ground_2中心距离 |
|---|---|---:|
| collapsed_ruins_02 | (1.1230,-.7867) | 2.219m |
| rescue_camp_02 | (1.1215,-1.3682) | 1.741m |
| trapped_people_01 | (.0182,-1.3713) | 1.329m |

这些结果没有通过完整门，不能恢复成接受候选，也不能拿另一处板面替首点上报。按固定候选模型与实际pose，ground_2中心应在原图约`(563.03,437.11)`，镜头前深度2.2528m，较远；该预测仅用于几何核对，不能替代分类或类别真值。原图/BEV已经查看，没有以肉眼改门槛或人工裁剪。

## 本机实测耗时

Python3.13.7、OpenCV5.0.0、AMD64 Family25 Model116、本机单进程、OpenCV线程数1。计时包含同步整帧IPM/query SIFT/全部12模板BF+RANSAC+几何门；无ROS/worker调度测试。

- 模板冷读取+解码+SIFT：7602.67ms。
- 原JPEG读取+解码：2.99ms。
- 同一帧完整match：2759.99、2615.88、2522.65ms，中位2615.88ms。

后续worker设计应计入冷加载和秒级同步成本；这里不是可靠实时上界或ROS回调预算。未测试并发、子进程启动、内存峰值或CPU竞争。`analyze.py`可原样复现；`manifest.json`核验源图、源summary、模板清单、冻结模块和本目录产物。
