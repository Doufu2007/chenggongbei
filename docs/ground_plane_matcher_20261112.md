# 独立 GroundPlaneMatcher：候选标定与几何拒识

新增 `src/inspection_mission/inspection_mission/ground_plane_matcher.py`，把冻结的整幅地面 IPM + SIFT 封装为独立纯感知模块。它从显式本地模板路径读取图像，返回几何候选及每张模板的全部拒绝原因，没有ROS、控制、任务ID、裁判或网络接口，也未接入 `vision_node` 或主任务。

## 接口与来源

`TemplateSpec(path, class_id, label, provenance, expected_sha256=None)` 明确提供模板及其标签注释。构造 `GroundPlaneMatcher(specs)` 时计算SHA256，若提供expected哈希则必须一致。模板不会自动打入src；本次开发核验使用本机 `external_resources/wbxxmz_ros2-car-first/` 的12张公共参考，来源URL/git_blob_sha/哈希在冻结IPM报告中。

`matcher.match(image_bgr, K, height_m=.173)` 校验非空uint8三通道BGR图、有限3×3针孔K（正焦距、零skew、正确末行、主点在图内），并只接受固定候选height=.173m。K由调用方显式传入，不暗补默认内参。调用方需先处理非零畸变；本次实测CameraInfo畸变为0。

输出包含 `candidates`、`diagnostics`、BEV、双向H及候选标定信息。每个几何候选有原图四角/中心、body-plane四角/中心、原模板SHA256、标签与provenance。`candidate_calibration=true`、`official_acceptance=false` 始终保留；标签并不构成官方类别真值或评分接受。

## 冻结方法与门槛

相机候选x=.390m、相机距平面height=.173m，假定水平前视；地面body forward X[.45,3.2]m、left Y[-1.6,1.6]m，固定300px/m、960×825。整个warp不读取人工角点、板号、类别或模板输出。

使用 `cv2.SIFT_create()` 默认float描述子，保持OpenCV原描述子数值，不另做RootSIFT或L2归一化；reference→query BF L2 ratio<.75、min8 good、RANSAC3px和原固定seed。独立几何接纳门全部同时满足：

| 条件 | 固定门槛 |
|---|---:|
| 不同template内点xy（0.001px取整计数） | ≥12 |
| 不同query内点xy | ≥12 |
| template内点凸包 / template面积 | ≥15% |
| inlier / good | ≥.25 |
| BEV投影 | 有限、凸、正面积 |
| 投影地面面积 | [.15,.30]m² |
| 内点重投影中位误差 | ≤1.5px |
| query内点凸包 / 投影多边形面积 | ≥10% |
| 四角相机光轴深度 X−.390 | 全部>0 |
| 回投原图四边形 | 有限、凸、正面积 |

任意两个已通过几何门、不同类的投影只要有正面积重叠，就双方拒识；不取高分方。多个不重叠物理区域可同时返回。同类重叠模板保留为多份参考证据，不进行分数胜者选择或合并；消费者不能据候选数量推断物理板数量。每模板目前只拟合一个全局H，不保证找出同图案的重复实例。

## 验证与修复证据

按TDD先记录缺失模块的预期RED，再实现。合成测试直接渲染已知面积.22m²、中心(1.30,0)m的纹理矩形，验证真实SIFT匹配、原图四角、物理位置、SHA/provenance。另覆盖重复单点、两侧覆盖不足、内点比例/面积/误差门、跨类冲突、不重叠多候选、输入K/height/BGR及模板hash。

独立审查发现“局部可见板跨相机深度零线”反例：板X[.30,.74]m，相机x=.390m，原门仍可能把103内点/76不同xy、面积.21975m²的BEV凸形接纳，但原图四边形非凸。生产回归先实际RED，再增加四角正深度与原图有限凸形门；原诊断和负深度保留，不放宽任何旧门。

- 初版完整项目测试：1126 passed，66.73s，`runs/analysis/20261112_ground_plane_matcher_dev/full_suite.xml`。
- 修复后受影响专项：32 passed，`specialized_final.xml`。按父任务明确要求，这个最小纯模块修复后不重复此前全套。
- 独立最终复核：9 passed（跨相机平面反例及另外8项），`runs/review/20261112_ground_plane_matcher/independent_final_green.xml`；审查文档为`docs/control_review_ground_plane_matcher_20261112.md`，无新增阻断。
- 红反例：`camera_plane_red.xml`；独立审查目录：`runs/review/20261112_ground_plane_matcher/`，保留其原始反例和红证据。

## 固定真实开发图核验

对冻结v16b、v17、v18三CAPTURE原样运行全部12模板，60对good/inlier与原冻结IPM实验逐项完全一致。初版与补充正深度门后的对照分目录保留，避免覆盖。

| 开发图 | 几何候选（模板注释） | inlier / 不同query xy | 面积 | 其余模板 |
|---|---|---:|---:|---|
| v16b | rescue_camp_02.jpg | 49 / 38 | .21014m² | 11拒识 |
| v17 | trapped_people_02.jpg | 37 / 33 | .21047m² | 11拒识 |
| v18 capture_0 | 无 | — | — | 12拒识 |
| v18 capture_1 | 无 | — | — | 12拒识 |
| v18 capture_2 | 无 | — | — | 12拒识 |

v18每一模板都不足12个不同query内点；求得H的投影也全非凸或退化，原图投影残差不能作为位置绑定。两张保留候选的原图overlay已经目视核查，落在近板纹理边界。当前没有跨帧稳定分类、运行时任务点绑定、官方裁判接受或held-out准确率结论。

复现：

```powershell
python -m pytest tests/test_ground_plane_matcher.py -q
python tools/diagnose_ground_plane_matcher.py --study-report runs/analysis/20261112_ground_ipm_sift_exploratory/report.json --output runs/analysis/20261112_ground_plane_matcher_dev/development_check_final
```

`development_check_final/report.json` 保存全部候选与拒绝、原图和body-plane坐标、模板provenance；`scores.csv`保留60行；每图保存BEV和原图候选overlay。开发样本只有同一环境的5图，3张CAPTURE连续相关；外参仍为公开候选模型，公共参考非当前官方完整纹理库真值。不得把2个几何候选当作2/5识别准确率。
