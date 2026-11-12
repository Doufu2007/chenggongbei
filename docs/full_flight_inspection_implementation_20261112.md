# 完整飞行续程 v41：实现与验证

## 交接与阶段

生产入口：`create_flight_inspection_node(reporter=None, *, session_id, ground_return_evidence, wall_clock=time.monotonic)`。

总桥必须先完成三地面点、真实返航静稳和实际地面零发布，再把同一 session 与原始 `return_completion` 交入。飞行 factory 拒绝缺失的双钟静稳、超界姿态/速度和未实际返回的零发布记录。原冻结模块保持原样。

飞行含 26 个任务阶段：起飞至 1.282 m；窗1接近、声明、当局视觉识别、下降至 .75 m、穿窗；两个空中点分别前视识别定位、移动到下照悬停、依据真实灯反馈累计有效照射；窗2识别穿越；沿公开候选返航路径回家落地。地面始终发布零。

成功标记要求实际 home XY/Z/yaw、线/角速度、源钟与墙钟各至少 1 s 静稳，独立三通道零发布返回、灯反馈关闭及完成消息发布返回。最终零发布与完成发布之后均重新核验当前源、终姿态与灯反馈；任何迟返失鲜都不能设置 `mission_completed`。

## 几何与传感器合同

- 公开固定 world：`ranmengjia/ICAN@82b041043849a5d86e085390bf0df787366d98cd`；SHA256 `595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1`。
- 静态扫掠使用完整旋翼磁盘与上下包络，路径采样间隔不超过 5 mm 并额外膨胀 5 mm；仅 home 支撑柱允许 floor 接触。名义 19.44 m 路线的全部连接段通过静态扫掠。
- 前云的实际 FOV 结果独立留证。前云覆盖外的空间由明确的公开静态模型补充；实际点云/环扫返回仍可以否决路径。当前点的三维扫掠联合检查 Z 接触时间区间和 XY 圆柱距离，覆盖纯上升、纯下降与组合位移。
- 公开节律：环扫 8 Hz、前云 12 Hz、下视 20 Hz、相机 30 Hz。新完整场景合同采用环扫源/墙龄 .35 s、前云 .5 s、下视 .30 s、odom 与 clock .20 s；重复 stamp 不刷新墙龄，同 stamp 改写内容拒绝。该合同依据传感器节律和静态空间补充建立；未测得的停距响应余量 .02 m 仍明确标为工程假设。控制限速 .12 m/s、静态 lookahead 1 s、失联 watchdog .25 s。
- 相机候选为 body→camera `(.390,0,.130)`，来自公开 world 与 launch static-TF 配置。v34 的 58 对同步 body 原点最大 Z 差 .0005 m；该运行未实收 camera TF。详情见 `runs/review/full_flight_independent_20261112/camera_and_scan_sources.json`。板平面世界 Z=.012 单独求交，不从外参再次减去。
- Z 使用 v37 在 .85 m 实测的 `.02695` trim 闭环；1.282 m 仍属待平台验证的候选高度。落地 cutoff 后实际零 Z。

## 当局视觉、缓存定位与照射

窗识别使用保留原门限的 `TemplateMatcher`。地板小板使用独立 `floor_plane_detector.py`：真实 K、原曝光前序 pose、完整姿态 IPM、显式多尺度/镜像参考派生、原 ORB 12 内点/.55 比例/可见四角门，并核公开 .35×.30 m 板物理尺寸。模板只提供类别参考，不提供当局点位类别答案。

`_vision_job` 保留原 stamp/frame、原 K 与 prior pose；反投影原相机中心像素，调用生产 `localize_floor_pixel` 得到 `TargetFix`。板中心候选仅用于物理点关联距离门。固定板中心不替换视觉 fix，缓存不重打时间戳。空检出、匹配异常和迟返错阶段结果也保留，最多每点三帧、四点共十二帧；诊断图是全分辨率 JPEG85 有损副本，另保留原 RGB SHA/尺寸，不能将其称为已留原始 RGB 字节。

照射将两个不确定性分开处理：原 ray 的完整 XY 界与缓存漂移保持不变；公开水平平面的 Z 界采用独立 `.005 m` 工程假设。严格 `target_geometry.assess_illumination` 继续校验版本、会话、源龄、真实 pose、下视 range 与真实 Bool 反馈，然后将完整横向界加入最终 spot 界，仍执行原 `.12 m` 总误差和 `.145 m` 命中半径。实际 a1 射线得到约 `.05639 m` 横向界；直接把它同时当平面高度误差会重复放大。本分解不改 fix、地图对齐、时间戳或命中半径。

## 验证边界

专项包括原始返航 proof、阶段顺序、窗净空、原光学定位、trim/落地 cutoff、缓存过期、灯反馈、真实 factory DDS 迟返、构造失败清理、整阶段发布顺序、重复源、yaw、三维点云扫掠、最终零迟返、空/迟视觉留证、报告多片双 SHA/长度及 64 KiB、真实参考图和 IPM worker 接线。独立审查已复跑迟返完成与垂直撞点反例，均改为正确拒绝。

离线真实 factory 闭环脚本：`runs/review/full_flight_closed_loop_synthetic_20261112.py`。按实际发布 body XY/yaw 积分，Z 使用明确合成模型；类别与投影像素为合成输入，fix 由生产 `localize_floor_pixel` 计算。结果 `317.0 s`、路径 `19.341873 m`、完整阶段落地后 `FINISHED / mission_completed=true`；详情在同名 JSON。这验证控制与几何联调，不能证明平台动力学、真实图像识别、计分接收或官方得分。

诊断报告采用 `gzip+base64`、40000 字符分片、压缩/解压长度与双 SHA256。官方计分保持 `unverified`。本实现报告不包含上传或新平台运行结果。

????????? SHA256 `619d3dab44c0c47251d8a603946dbc84aa8f60367d1ea0b6e38b4898222209cd`?floor ?? `a0ae1f6eb8e9950e69eb0e4a7bceffef32abeb1809c51901b6266b2bdbcdafd9`?150 ?? fastpath / ???? 300 ?? fallback???????+???? 27 ????floor ?? 14 ?????? monotonic ?? factory ????? 14.433 s ??????????????????????????????? `runs/review/full_flight_async_timing_20261112.{py,json}`???? pytest ????????????????
