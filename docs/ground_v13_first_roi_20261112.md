# v13 首点ground_4真实ROI

日期：2026-11-12。本轮只读本队v13原图与公开候选XML，未运行分类器、未改控制/测试、未发布任务声明。

本局公开顺序为`ground_4,ground_3,ground_0`，控制回传结束原因为`target_yaw_reached`。这证明原地观察转向的一个停止条件已满足，不是完整任务计分成功。选取的真实末图曝光t=23.845s，前序`/uav/odom`为23.840s，差.005s；机体约`(-.00241611,-.00543359,.05500007)`、yaw=-.57935562rad。末CameraInfo同stamp23.845s，K与先前收到的内参一致。

已在`runs/analysis/20261112_survey_v13/`生成独立文件，未覆盖v11/v12：

- `ground4_candidate_projection.py`：离线复现脚本。
- `ground_next_path_projection.json`：原图/summary/world哈希、原曝光位姿、原CameraInfo、候选外参、四角与单应矩阵、产物哈希。
- `ground4_candidate_projection_overlay.png`：原图副本标注候选地面板。
- `ground4_candidate_bbox.png`、`ground4_candidate_rectified_world_axes.png`：首点候选区域的真实裁剪与平面展开。

复现：

```powershell
python runs/analysis/20261112_survey_v13/ground4_candidate_projection.py
```

原图`native_L000008_d2244845f6e50da7.jpg`，SHA256：`d2244845f6e50da75e75a6722c1fbeecc55bbe3ff6275f6482bde8691a820700`。

四角仍依次为世界`(x-,y-)、(x+,y-)、(x+,y+)、(x-,y+)`，本图预测uv：`(582.811,486.589)、(454.845,464.297)、(250.081,483.671)、(332.363,526.054)`。完整多边形位于图内，外接框约332.73×61.76px。已查看标注图与展开图：候选投影贴近真实最近照片边缘，照片仍有明显斜视采样损失，放大不会补充原始细节。

相机外参`(.390,0,.130)`、平面z=.012m和ground_N同号映射仍为`candidate_unverified`；四角来自几何预测，未作独立像素角点测量。单应输出世界+X向右/+Y向下，不保证照片语义直立，供识别器检查时可做旋转对照。

本图是本局首点的候选区域，具备进一步离线分类检查价值；它本身尚不是类别或上报接受证据。v11另一局同区域CLIP四旋转低margin拒识不能转化成本局类别。本分析没有给出目标类别，也没有取得任何分数。
