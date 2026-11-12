# v38 三张真实 JPEG 的视觉阻塞根因

结论：现有三张曝光的直接阻塞来自公开参考库缺少 `damaged_vehicle_02.jpg`。补齐两张下载失败的公开参考后，原 `GroundPlaneMatcher`、原内点/覆盖/面积/重投影门均不改，三帧均只接受该显式模板并成功绑定 `ground_3`。

证据目录：`runs/analysis/20261112_v38_visual_root_cause/`；可复现入口 `replay_ten_templates.py`，完整对应、H 与诊断见 `ten_template_replay.json`，摘要见 `root_cause_summary.json`。JPEG 来自 `runs/analysis/20261112_three_point_recovery_v38/`，来源是本次真实回调而非合成图。

|曝光|good|inliers|不同 query 位置|平面面积 m²|中位重投影 px|
|---|---:|---:|---:|---:|---:|
|0|180|106|96|0.208073|0.236754|
|1|188|102|92|0.207981|0.245186|
|2|190|103|94|0.208273|0.225641|

原八模板的 24 对诊断在补齐前后逐项一致：不同 query 位置均仅 1–5 个，24 对全部投影非凸或退化，全部拒绝。原门在正确拒绝不支持的匹配，继续放宽阈值没有证据依据。

恢复依据为 `external_resources/wbxxmz_ros2-car-first/SOURCE_MANIFEST.json` 中两次 `IncompleteRead`；两张公开原图经过固定 commit、source URL、SHA256、Git blob SHA1 与 size 联合校验，见 `runs/analysis/20261112_missing_templates/recovered_manifest.json`。新独立 `ground_template_catalog.py` 保留原八张，严格追加 `damaged_vehicle_02.jpg` 和 `collapsed_ruins_01.png`；不引入运行点到类别的映射。

三帧匹配中心约 `(1.1229,-1.9172)`，距公开几何候选 `(1.125,-1.925)` 约 8.1 mm。中心距离只用于诊断一致性；类别来自真实曝光与显式模板对应。

限制：回放使用首末 CameraInfo 一致所支持的稳定 K 假设，缺少逐曝光 K 与原 worker 的完整输出，不能称逐字节运行时重现。三帧是同一静止场景的相关曝光，不能据此估计泛化准确率，也不代表官方识别接口已经接受。类别不从目视、地图或截图方向推断。
