# 地面平面匹配器：独立只读审查

日期：2026-11-12。只读审查新增模块、测试、冻结开发报告和来源；独立反例只写入 `runs/review/20261112_ground_plane_matcher/`。未改生产源码、生产测试、实现代理输出、原始回传，也未调用网站、控制或裁判接口。

初审源码 SHA256：`2d40ee4fd667a7cdf7c3dd86310fa310849f1653a4ccd35aae798d93e4011ac0`；初审 `tests/test_ground_plane_matcher.py` SHA256：`0021610257f8e0a4507d28ccd78f665b5eae0d465ba6a7c0e599e72b23255c3c`。后续修复需重新核对版本。

## 已修复问题：相机平面后方的四角曾可作为接纳候选输出

初审存在一个可复现几何缺口。`verify_geometry` 检查 BEV 中的凸四边形与面积，但 `match` 在原图反投影前没有要求所有四角位于相机正深度侧，也没有因原图四边形非凸而拒识。局部可见图板的单应性可在 BEV 中合法外推到相机后方，使原图四角跨过 depth=0 无穷线，仍 `accepted_candidate=true`。这影响原图候选四边形的几何语义，部署视觉对准前应修复；当前模块离线，与 v19 控制实测分离。

独立物理反例：图板 X=[.30,.74]m、Y=[-.25,.25]m，候选 camera X=.390m，K=(fx=300,fy=300,cx=400,cy=100)。按原始针孔光线反解，仅渲染正深度地面上可见的图板部分，没有用模块的 BEV 来制造输入。初审版本得到 111 good /103 inliers/76 distinct query、面积 .219747660m²，却接纳了近侧 X≈.300m 的两个角；原图四角包含约 (-434.51,-477.81)/(1232.19,-477.10)，四边形非凸。高匹配数与小重投影残差不能消除这一投影域错误。

初审实际保留证据为 `horizon_template.png`、`horizon_case.json` 和 `horizon_red.xml`。草稿曾列出的 `horizon_source.png` / `horizon_bev.png` 实际未落盘；已纠正清单，修复后通过独立物理公式重新生成并核验 PNG 写入，另存为 `horizon_source_final.png` / `horizon_bev_final.png`，没有覆盖旧 JSON 或红结果。独立可执行回归：

```text
python -m pytest -q runs/review/20261112_ground_plane_matcher/horizon_regression.py
```

初审版本实际 **1 failed**，原红结果 `horizon_red.xml` 保留。已向主代理与实现代理报告，由实现代理修复；本审查未自行改生产实现。

## 修复后独立复核通过

冻结修复源码 SHA256：`1b9c9b3cd658f5192307161dbbe21954968e9680c98424250d2d0836e1dc710d`；生产测试 SHA256：`5d2caffcbe16af77844494f1894e23799821baec0125c9284be90aeb4377af9c`。最小修复在输出接纳候选前增加四角 camera depth>0、原图四边形有限/凸/面积>0 门；未降低既有 SIFT、支持覆盖、面积、重投影或重叠冲突门槛，也未增加 ROS/控制/裁判接口。

独立旧反例加 8 项检查实际 **9 passed in 2.23s**：

```text
python -m pytest -q runs/review/20261112_ground_plane_matcher/horizon_regression.py runs/review/20261112_ground_plane_matcher/review_cases_final.py --junitxml=runs/review/20261112_ground_plane_matcher/independent_final_green.xml
python runs/review/20261112_ground_plane_matcher/final_review_metrics.py
```

同一反例仍为 111 good /103 inliers/76 distinct query、面积 .219747660m²，但正确输出 `accepted_candidate=false`，原因同时保留 `nonpositive_camera_depth` 和 `source_projection_nonfinite_or_nonconvex`。四角深度约 [.349901,.349409,-.089822,-.089932]m，原图四边形非凸，强 SIFT 支持不会绕过物理投影门。

新真实五帧报告为 `runs/analysis/20261112_ground_plane_matcher_dev/development_check_final/report.json`，SHA256 `a4b78d35184756a0b25e1397f951ff77e6824846ec205409bd542835009197e6`。独立核查 12 模板×5 帧的 60 对诊断和源哈希，接纳数量保持 **[1,1,0,0,0]**；两项候选全部四角正深度、原图有限且凸；三张 v18 图仍拒识，未改变研究证据边界。完整复核机读记录为 `runs/review/20261112_ground_plane_matcher/independent_final_review.json`，旧报告/红证据不覆盖。

结论：本次独立审查发现的投影域缺口已闭环；当前离线模块未发现新的阻断问题。它仍只是候选标定下的模板几何工具，不能据此认定官方类别正确、任务板 ID、真实控制精度或裁判得分。

## 已通过的独立检查

`review_cases.py` 实跑 **8 passed in 2.86s**，结果 `targeted_review.xml`，覆盖：

- 非等焦距、非中心主点 K 下，独立针孔公式与 source↔BEV 变换一致，轴方向及 half-pixel 中心差值<1e-9px。
- 同一物理合成图板同时配置成两个不同 class_id，端到端 SIFT 几何均成立后双方都被 overlap conflict 拒识，不按最高分选择赢家。
- provenance 在构造时快照，调用方或返回候选的后续修改不污染内部/diagnostics；标签不生成 board_id/ground_id/task_id，official_acceptance 始终 false。
- A–B–C 重叠链检查全部原几何候选，输入不被修改；精确接边无正面积重叠时不误拒。
- 非有限 correspondence、不合法 mask、长度不一致均显式 ValueError。
- 真实五帧与12参考的源文件/哈希一致，共60对完整诊断；接纳项所有既定门槛均满足。以独立针孔公式重投影米制四角与中心，原图坐标差值<.001px。

## 真实开发证据的边界

只读报告：`runs/analysis/20261112_ground_plane_matcher_dev/development_check/report.json`，初审 SHA256 `cf068a3d4b7c4dfd7fbec0ba1de74e2dc1ffa7610e238d12702fd0d1c996e765`；源研究报告 SHA256 `b6b4738c7ca1248cb48d645c257f0410ae0bbfc31980c1b0762108d33bfd0c5c`。report 的 class_ground_truth/accuracy=null、held_out=false、official_acceptance=false。12份参考都保留显式 source_url、Git blob、SHA256与 `official_current_library_verified=false`，没有将候选资料当作官方真值。

| 真实帧 | 接纳候选注释 | inliers / distinct query | 候选平面面积(m²) |
| --- | --- | ---: | ---: |
| v16b | rescue_camp_02 | 49 / 38 | .210142308 |
| v17 | trapped_people_02 | 37 / 33 | .210474198 |
| v18 capture_0 | 无 | — | — |
| v18 capture_1 | 无 | — | — |
| v18 capture_2 | 无 | — | — |

v18三图每个参考都保留拒识原因，query 独立位置均不足12；没有因为 good/inlier 数较高就选类。前两项只是第三方参考注释下的几何候选，不能称作官方识别正确率、ground_N 映射或裁判认可。v16b末图本身没有有效曝光prior姿态，不能从该图倒推已验证世界坐标；body-plane 输出仍建立在固定候选相机高度、水平姿态和无畸变前提上。五帧为开发样本，后三帧强相关，不能据此报准确率或泛化能力。
