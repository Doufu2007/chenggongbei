# v16b 单张真实近板：离线透视纠正探索

现有固定方法尚未在这张真实近板上给出可用于运行时决策的稳定输出。人工四角纠正改变了 CLIP 输出，并产生可解释的场景候选；方向敏感性、未知真值与单张数据仍阻止自动类别判定。OpenCV 原方法没有接受结果。

## 数据与固定协议

- 来源：`../20261112_approach_v16b/native_L000020_a672e7739804717c.jpg`，800×800，SHA256 `a672e7739804717cb0633a2d26dda3b94e4c058384a86d7c9cd3e4355bfe3ac1`。
- 人工近似四角：`(267,477), (524,477), (592,531), (198,531)`。板面在源图中仅占约 54px 高；这是人工标注，未实现运行时定位。
- 预先固定 10 种原图/裁剪/上下文/透视变体，每种保留 0°、90°、180°、270°，共 40 个结果，不按分数删选。
- 本地官方 CLIP ViT-B/32，沿用原 7 类英文提示与原拒识门槛：top1 cosine ≥0.20 且 top1−top2 ≥0.02。门槛未经校准，未在本图调参。
- OpenCV 沿用原 TemplateMatcher/detect_targets 与原门槛。图像近邻使用 12 张已验证来源的第三方参考，不能视为当前官方平台纹理真值。
- 透视纠正只使用真实像素双线性重采样，输出尺寸/纵横比是敏感性候选；没有超分、生图或细节补造。

## 主要结果

5 种未纠正输入（完整场景、紧外接框、其 letterbox、更宽上下文、其 letterbox）×4 个方向：CLIP **20/20 拒识**。全部 40 个输入：OpenCV **40/40 无接受结果**。

下表的类别均为模型候选，`accepted` 仅表示越过原探索门槛。cosine 不是概率。

| 输入与逆时针旋转 | CLIP top1 候选 | top1 cosine | margin | CLIP 门槛结果 | 图像最近邻 / cosine |
|---|---|---:|---:|---|---|
| 原图 0° | rescue_camp | 0.1934 | 0.0060 | 拒识 | window_target_2.jpg / 0.6275 |
| 紧外接框 180° | window_bird | 0.2279 | 0.0050 | 拒识 | window_bird_01.jpg / 0.6203 |
| 更宽上下文 180° | collapsed_ruins | 0.2056 | 0.0129 | 拒识 | window_target_1.jpg / 0.6679 |
| warp_4_3 180° | rescue_camp | 0.3115 | 0.0715 | accepted 候选 | rescue_camp_02.jpg / 0.6631 |
| warp_square 180° | rescue_camp | 0.2742 | 0.0371 | accepted 候选 | rescue_camp_02.jpg / 0.5815 |
| warp_4_3_context 180° | rescue_camp | 0.2988 | 0.0640 | accepted 候选 | rescue_camp_02.jpg / 0.6633 |
| warp_4_3_expand_y2 180° | rescue_camp | 0.2742 | 0.0370 | accepted 候选 | rescue_camp_02.jpg / 0.6688 |
| warp_4_3_contract_y2 180° | rescue_camp | 0.2793 | 0.0320 | accepted 候选 | landslide_02.jpg / 0.5977 |
| warp_4_3 270° | collapsed_ruins | 0.2925 | 0.0237 | accepted 冲突候选 | window_target_1.jpg / 0.5922 |
| warp_square 270° | collapsed_ruins | 0.2922 | 0.0232 | accepted 冲突候选 | window_target_1.jpg / 0.6249 |

`warp_4_3` 的方向对照：0° 为 damaged_vehicle（0.2393 / margin 0.0178，拒识），90° 为 rescue_camp（0.2781 / 0.0095，拒识），180° 为 rescue_camp（0.3115 / 0.0715），270° 为 collapsed_ruins（0.2925 / 0.0237）。未验证的方向不能靠事后挑最高分解决。

人工纠正后 180° 图可见一排橙色帐篷/物资与山地场景，公开营地参考在场景层面相似。这是可见内容与模型候选的描述，不能据此认定查询类别或当前官方纹理。加入外部上下文未展示明确收益：warp 180° 加上下文后文本 cosine 从 0.3115 降为 0.2988；单张数据不足以判断泛化效果。

## 可以支持与不能支持的判断

- 可以支持：严重斜视和原 CLIP 中心裁剪会妨碍输入表示；在本图中，人工几何纠正显著改变候选及分差，值得独立验证。
- 不能支持：稳定自动识别、准确率、当前官方类别真值、运行时四角提取能力，或上下文普遍改善。7/40 是同一 JPEG 的变体越门槛次数，**不是识别率**，40 也不是 40 个独立样本。
- 边界上下 ±2px 仍会改变分数，收缩版本的图像近邻变为 landslide；输出纵横比与方向均未由数据验证。
- 重采样后 ORB 关键点数量增加不等于获得新细节；54px 源信息的限制仍在。图像近邻 cosine 不具备类别概率含义。
- 源文件是已留存诊断 JPEG，不是新采原始相机字节；原关联位姿仍标记为未验证。此次没有获取新图、联网、上传、部署或改动控制/正式感知配置。

后续最有价值的证据是官方平台上更近、更正视且有可靠关联的多帧原图。若继续研究运行时透视纠正，需要另行验证自动四边形提取与方向判定，并在预先固定方法下评估多个独立板面/视角；人工 warp 不能替代这些能力。

## 产物

- `report.json`：完整 40 变体、各类文本分数、全部图像近邻、门槛、来源和限制。
- `scores.csv`：40 行摘要，包含所有 7 类文本 cosine 与全部 12 个参考 cosine。
- `manual_quad_overlay.png`：人工标注；`candidate_gallery.png`：10 候选 0°；`query_reference_neighbors.png`：固定 warp_4_3 的四方向与各 top3 近邻。
- `reference_gallery.png`、各候选/旋转 PNG、`image_features.npz`：保留可复查像素与特征。
- `experiment.py`：原离线复现脚本；`manifest.json`：本目录文件 SHA256 与原始来源校验。

本次整理只读取已保存报告和特征，没有重复模型推理。详细浮点数以 `report.json` 为准。
