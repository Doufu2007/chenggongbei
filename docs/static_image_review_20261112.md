# 公开静态图片复核（2026-11-12）

## 范围与证据

仅检查 `external_resources/ranmengjia_ICAN/STATIC_IMAGE_ADDITIONS_20261112.json` 指定的 8 张公开静态图片。公开仓库固定提交为 `82b041043849a5d86e085390bf0df787366d98cd`。未读取随机贴图分配、裁判实现或他队任务代码；以下图像内容不代表当前平台测评的随机类别与点位绑定。

图片均位于 `external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/materials/textures/`。8 张均已完整到达并按实际图像内容目视检查；大图使用完整解码后等比例缩略图查看。没有依据文件名直接给官方类别贴标签。

## 逐图结论

| 文件 | 解码尺寸 | 实际内容 | 可作为七类目标参考图吗 |
|---|---:|---|---|
| `aerial_target.jpg` | 1722×2436 | 白底绿色军用物资箱 | 不能据此贴为官方五个地面类中的任一类 |
| `inspection_marker_1.png` | 6664×4892 | 绿色“指挥中心”字牌 | 否，场地标记 |
| `inspection_marker_2.png` | 6664×4892 | 灰底黑圈数字 1 | 否，场地标记 |
| `inspection_marker_3.png` | 6664×4893 | 灰底黑圈数字 2 | 否，场地标记 |
| `window_target_1.jpg` | 1024×626 | 蓝天背景的白色四旋翼无人机完整照片 | 是，可作为公开无人机照片候选；不能证明平台当前类别映射 |
| `window_target_2.jpg` | 1024×626 | 浅蓝灰天背景的白色四旋翼无人机完整照片 | 是，同为无人机候选，不能标作飞鸟 |
| `ican_sticker_large.png` | 14174×11339 | 蓝底整场地图，含指挥中心、数字圈、灰色任务格和 H 降落标记 | 否，整场静态地图 |
| `inspection_sticker_small.png` | 9845×7876 | 蓝底整场地图，布局与大贴图相同，含指挥中心、数字圈、灰色任务格和 H 降落标记 | 否，整场静态地图 |

两张地图预览保存在 `runs/analysis/static_image_review_20261112/ican_sticker_large_preview.jpg` 与 `inspection_sticker_small_preview.jpg`（均为 1800×1440）。这些文件仅用于审查可视化，不是新的视觉参考。

## 完整性记录

| 文件 | 字节数 | SHA256 |
|---|---:|---|
| `aerial_target.jpg` | 190926 | `14f20bbdab3ea25aa7fc3f64550ca56930a8a2532618d723d6af084a93487a01` |
| `inspection_marker_1.png` | 346793 | `2f0f8af5b4f2fa619f66795fcb849e32732d7c73e428b75a946412eef6c4fdab` |
| `inspection_marker_2.png` | 226063 | `0dddb5c36b6c4b262330760fcc18c1390c8b72881db93cfa54073d2578a45c62` |
| `inspection_marker_3.png` | 254305 | `607fa18f198239717942bca798b77607159b0a15145c3af013c8a097f9a1a521` |
| `window_target_1.jpg` | 57883 | `8c4ae0fbce7eff2812068acde1e7414bb2578594d0693db1ce3f290bb19fe42e` |
| `window_target_2.jpg` | 42160 | `aa9454848f219097403fa26d5c5165c0108dfb9d143ad18ed37687bdd863c5e6` |
| `ican_sticker_large.png` | 6050915 | `369c8654abc349256002611b6ba423ff9936a8e376011e2a28ee2fef4885352f` |
| `inspection_sticker_small.png` | 4510093 | `726510e8880b0441eac3d3f2c77fdf806040f77beeb625ef06cb0bde22f57070` |

`inspection_sticker_small.png` 最初出现截断，现已通过公开 Git blob `96e3b5d143583b21fc42cb119c28172ec0e2565f` 完整重取，并通过 PNG 全部校验。下载入口为 `https://api.github.com/repos/ranmengjia/ICAN/git/blobs/96e3b5d143583b21fc42cb119c28172ec0e2565f`。

## 对后续测试的影响

当前 8 张完整新增图不足以补齐官方七类目标图库：两张窗上图是同一无人机类别；尚未取得飞鸟与五类地面对象的完整可核验参考照片。不得把箱子、数字牌或地图当成这些类别，不得把窗图 2 硬标为飞鸟。下一步应以本队真实相机图像的可辨认内容验证类别或继续取得公开参考资料；当前随机点位和类别关系仍须由运行中实际视觉观测确认。

本次只做素材审查，没有更改识别配置、官方话术编码、profile verified 标志、计分证据或运行包。
