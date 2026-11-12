# v13 真实 ROI 的冻结 CLIP 图像近邻诊断

日期：2026-11-12。使用本队 v13 相机真实曝光的候选地面区域，固定四旋转，与 12 张公开参考照片逐张比较。结果仍不足以确定查询照片类别；没有类别上报、阈值选择、训练或裁判交互。

## 输入与复现

原始曝光、CameraInfo、位姿和候选投影见 `docs/ground_v13_first_roi_20261112.md`。原图 SHA256 为 `d2244845f6e50da75e75a6722c1fbeecc55bbe3ff6275f6482bde8691a820700`，曝光 23.845s，前序 odom 23.840s。预测区域在原图约 333×62px；透视展开后保存为 256×352px，这个尺寸变化不增加原始采样细节。

本轮查询为 `runs/analysis/20261112_survey_v13/ground4_candidate_rectified_world_axes.png`，SHA256 为 `11a974d44ad61daaa418bb2409fd8bd5f5ad4a2d70fabc60496f05e865895043`。展开以世界 +X 向右、+Y 向下，不保证照片语义直立；相机外参、板平面和同号映射仍为 `candidate_unverified`。本轮检查逆时针 0/90/180/270°，没有检查镜像反射。

参考照片来自公开仓库 `https://github.com/wbxxmz/ros2-car-first` 的提交 `48b3dac26b6f562ff21dd321db03fd6ffc91b492`；每张图片的来源与哈希存于结果文件。仅使用照片，不读取其任务代码。当前比赛实际图库是否与这 12 张一致仍未验证；来源清单的 `license` 为 null。

编码器为官方冻结 CLIP ViT-B/32，权重 SHA256 为 `40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af`，实现固定在 `openai/CLIP` 提交 `d05afc436d78f1c48dc0dbf8e5980a9d471f35f6`。使用官方 224px Resize、CenterCrop 和归一化；图像特征转 float32 后进行 L2 归一化，以 float32 点积计算余弦相似度。没有文本提示或额外分类头。

```powershell
python tools/diagnose_clip_image_neighbors.py --projection runs/analysis/20261112_survey_v13/ground_next_path_projection.json --output runs/analysis/20261112_survey_v13/image_neighbors
```

## 观察结果

| 查询逆时针旋转 | 第一近邻文件 | cosine | 第二近邻文件 | cosine |
| --- | --- | ---: | --- | ---: |
| 0° | window_target_1.jpg | 0.607429 | window_bird_01.jpg | 0.593714 |
| 90° | window_target_1.jpg | 0.639807 | window_bird_01.jpg | 0.634663 |
| 180° | window_target_1.jpg | 0.581775 | window_bird_01.jpg | 0.577755 |
| 270° | window_bird_01.jpg | 0.628289 | window_bird_02.jpg | 0.621933 |

已查看查询与前三近邻接触图、全部参考图库。天空背景参考图处于榜首，但接触图没有提供可确认的同一照片关系。榜首变化和较接近的分数也不能独立证明错误或正确。文件名是参考照片的注释，不是查询照片的真实类别。

完整 4×12 相似度、每个查询的全部 12 名排名、来源、模型版本及产物哈希保存在 `runs/analysis/20261112_survey_v13/image_neighbors/image_neighbors.json`；同目录还有：

- `normalized_image_features.npz`：4×512 查询与 12×512 参考特征。
- `query_reference_neighbors.png`：每个查询与前三近邻接触图。
- `reference_gallery.png`：全部 12 张参考照片。
- `query_ccw_000.png`、`query_ccw_090.png`、`query_ccw_180.png`、`query_ccw_270.png`：固定查询旋转版本。

CUDA 设备为 RTX 4050 Laptop GPU，torch 2.6.0+cu124。本次权重校验和加载约 14.8s，16 张图编码约 1.24s。这是本机一次诊断耗时，不能直接当成平台实时延迟。

## 校验与解释边界

运行 `python -m pytest tests/test_clip_image_neighbors.py tests/test_reference_vision.py tests/test_zero_shot_vision.py tests/test_analyze_platform_camera_frames.py -q`，结果 **28 passed**。重新核验了 32 个源文件和产物的 SHA256，保存的 512 维特征可以重现 4×12 相似度矩阵，排名均覆盖 12 张参考照片，无缺项。

`class_decision`、`class_ground_truth`、`accuracy` 均为 null，`published_to_referee=false`，`threshold_selection=false`。最高 cosine 不是类别概率，不能直接作为接受门槛。四个旋转版本来自同一曝光，只有一个独立查询样本。本次结果不能证明分辨率是主要阻塞；透视、语义朝向、可能的反射、候选外参和参考照片域差异均未排除。

## 后续验证划分

现有 12 张参考照片中，`collapsed_ruins` 和 `damaged_vehicle` 各仅一张，其余五类各两张。既有工具划分为 7 张 reference、5 张 held-out；本诊断将全部 12 张作为近邻候选，因此不是 held-out 准确率实验。

后续按原始照片分组划分；同一照片的裁剪、旋转、压缩、透视和其他变换必须放在同一组，不能跨 reference/held-out。两个单照片类别需要新增独立公开照片才能评估跨照片泛化；其余五类可做两照片交换验证，但样本量不足以代表正式赛场准确率。

优先在同一局、同一照片上保留远近配对相机曝光及其位姿和内参，固定预处理、编码器及比较规则，比较采样改善后的近邻稳定性，再判断分辨率的影响。独立校验投影方向与可能的反射。取得查询类别的可验证依据和真实平台计分证据前，不把本诊断写成识别成功或比赛得分。
