# v12 真实图像的地面1/2候选ROI

日期：2026-11-12。本次只读 v12 本队图像与公开候选 world，未改控制或发布任何消息；未运行分类器。保留 v11 全部产物，不复用 v11 的类别猜测。v11 ground_4 的 CLIP 四旋转均低 margin 拒识，不能当类别依据。

## 结论

v12 已向南转到约 `-65.23°`，首点仍为 `ground_2`。候选 ground_2 完整进入原图右上方，但外接框仅 **159.79×11.78 px**，短边最低约 **7.89 px**，仍不足以稳妥支持细类别声明。ground_1 的照片在右侧被裁断，候选多边形只有约 **57.23%** 面积在画面内，不能将其当完整目标送出识别上报。

投影标注与可见两张照片的位置吻合。透视展开后的 ground_2 纹理因原始采样不足而明显模糊，放大至352×256没有增加真实细节。因此下一层应改善观察位置，沿上一份路线分析逐步验证直行，不应继续仅在起点转向后降低分类阈值。

## 产物与复现

目录：`runs/analysis/20261112_survey_v12/`。

| 文件 | 含义 |
| --- | --- |
| `ground12_candidate_projection.py` | 可复现离线投影及裁剪脚本 |
| `ground12_candidate_projection.json` | 原始图/summary/world哈希、完整原曝光位姿、CameraInfo、候选外参、角点、面积和裁剪边界、产物哈希 |
| `ground12_candidate_projection_overlay.png` | 原图副本标注 ground_1 橙框、ground_2 绿框 |
| `ground1_visible_fragment_bbox.png` | 只保存原图可见片段，明确不是完整任务图 |
| `ground2_candidate_bbox.png` | 原图 `[590,435,752,449]` 外接框裁剪 |
| `ground2_candidate_rectified_world_axes.png` | ground_2 的候选平面展开图；世界+X向右、+Y向下，不保证语义直立 |

```powershell
python runs/analysis/20261112_survey_v12/ground12_candidate_projection.py
```

已运行脚本、核对原图哈希、查看标注图与展开图；代码没有网络、ROS或裁判依赖。ground_1 不做单应展开，避免把画面外缺失部分补成完整目标。

## 保留的来源与假设

原图：`native_L000009_cb6cec12ad73d25a.jpg`，800×800，SHA256见同目录JSON。曝光 `t=30.218 s`，真实前序 `/uav/odom` 为 `30.200 s`，差 `.018 s`。机体位置约 `(-.00328489,-.00282134,.05500022)`，yaw=`-1.13843872 rad`；计算使用完整原始四元数。

CameraInfo的两次K/D一致，K=`fx=fy=476.7030836,cx=cy=400.5`，D=0；首header为t=2.885s，末header为t=30.218s，与本张图的原曝光stamp相同，原header均保存在JSON。这次有同步内参消息，但不代表外参已获官方确认。本离线计算用K，不用P中的baseline替代外参。

固定公开候选 XML：`external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world`。相机 body 平移 `(.390,0,.130)`，零旋转；灰色任务板顶面z=.012。相机前向+X，图像右=-Y、下=-Z。**相机外参、平面与 ground_N 同号映射仍为 candidate_unverified**，四角为预测值，未独立检测角点，视觉吻合不能升级为官方合同确认。

角点顺序固定为世界 `(x-,y-) → (x+,y-) → (x+,y+) → (x-,y+)`。

| 候选点 | C0 uv | C1 uv | C2 uv | C3 uv |
| --- | --- | --- | --- | --- |
| ground_1 | `(874.05,492.04)` | `(567.18,473.39)` | `(541.98,507.85)` | `(1045.30,553.86)` |
| ground_2 | `(727.91,439.28)` | `(594.90,435.49)` | `(590.24,441.86)` | `(750.03,447.26)` |

ground_2候选原多边形面积约1010.86像素²，ground_1完整候选面积约17140.96像素²、画内约9809.14像素²。单精度多边形求交造成 ground_2 的 visible_area_fraction 比1大约1.4×10⁻⁸，属于计算舍入，JSON保留原数值；不构成额外可见区域。

本轮没有类别、识别接受、到点、任务完成或平台成绩证据。此结果支持“先近距离稳定采图”的下一步，而非将诊断过程当正式计分成功。
