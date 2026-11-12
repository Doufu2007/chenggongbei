# Ground Plane Matcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 提供不依赖人工ROI、ROS、控制或裁判的候选标定地面IPM+SIFT纯感知模块。

**Architecture:** 模块从显式模板路径计算默认OpenCV SIFT描述子；对固定整幅BEV完成BF ratio/RANSAC，再执行独立几何接纳门及跨类重叠冲突拒识。全部模板诊断保留，候选携带原图四边形、body-plane位置和来源，且保持candidate_calibration标记。

**Tech Stack:** Python、NumPy、现有OpenCV SIFT、pytest；不新增依赖。

**Spec:** 父任务2026-11-12消息及冻结 `runs/analysis/20261112_ground_ipm_sift_exploratory/report.json`；本文复制全部固定接纳要求。

## Global Constraints

- 只新增独立源码/测试/离线工具/文档，不改vision_node、主任务、profile或控制，不上传。
- 固定相机x=.390、height=.173仍是候选外参；X[.45,3.2]、leftY[-1.6,1.6]、300px/m；K必须来自显式输入。
- 默认SIFT原描述子，不追加RootSIFT/L2归一化；BF L2 ratio<.75、min8 good、RANSAC3px。
- 接纳要求：两侧不同内点xy各≥12、template凸包覆盖≥.15、inlier/good≥.25、有限凸投影、地面面积[.15,.30]m²、中位误差≤1.5px、query凸包/投影面积≥.10。
- 跨类投影有正面积重叠则双方拒识；不按最高分强选，不按板ID选类别。输出不是官方接受。
- 父任务正在跑v19全套，收到完成消息前，不将新测试命名为test_*.py或写入生产src。

## Review Focus

- 输入BGR/uint8、K非有限/奇异/非针孔、错误height必须明确拒绝，避免悄悄使用默认K或高度。
- 单点/少数重复坐标和低覆盖假阳性，不能靠高RANSAC内点数通过。
- 几何上有效但物理面积、误差或inlier比例不合要求，必须逐条给出拒绝原因。
- 不同类别重叠时双方拒识；互不重叠的物理候选可并存。
- 模板路径/hash/provenance必须保留，不把标签当官方真值，不因本机外部模板自动打入src。

### Task 1: 纯模块与合成TDD

**Files:**
- Create: `src/inspection_mission/inspection_mission/ground_plane_matcher.py`
- Test: `tests/test_ground_plane_matcher.py`
- Preparation only: `runs/analysis/20261112_ground_plane_matcher_dev/tdd_cases.py`

**Interfaces:**
- `TemplateSpec(path, class_id, label, provenance, expected_sha256=None)`；`GroundPlaneMatcher(templates).match(image_bgr, K, height_m=.173)`。
- `rectify_ground_plane(image_bgr,K,height_m=.173)` 返回BEV和双向H。
- `verify_geometry(template_points,bev_points,inlier_mask,H,template_wh)` 返回全部门诊断；`reject_overlapping_class_conflicts(rows)`拒绝跨类重叠。

- [x] 写合成纹理、已知H、输入错误、hash错误、退化/覆盖/面积/误差/比例门、重叠与多物理板测试；开发fixture不依赖真实模板。
- [x] v19完成后移入tests，先确认缺失模块的预期RED，再逐步实施至GREEN。
- [x] 保持源码无ROS/控制/裁判依赖，模板只显式路径读取。

### Task 2: 固定开发图核验与交付

**Files:**
- Create: `tools/diagnose_ground_plane_matcher.py`
- Create: `docs/ground_plane_matcher_20261112.md`
- Output: `runs/analysis/20261112_ground_plane_matcher_dev/`

- [x] 对冻结v16b/v17/v18三CAPTURE原样运行，保存每模板诊断、候选/拒绝、BEV及来源哈希，不把开发数据当真值/泛化率。
- [x] 运行专项与项目要求检查，记录实际结果；请求独立审查，修复可验证问题。
- [x] 产出源码/测试/工具/证据SHA256与文档，确认本任务未修改旧冻结报告、vision_node、控制及profile。
