# 地面任务显式使用 UAV 相机

2026-11-12。本轮只补本队控制器的观测接线，没有启动平台动作，没有修改默认 `verified`、地图坐标、相机外参或有界探针/survey模块。

## 启用方式

仅在已经核对观测归属的某个 `ground_points` 项内显式添加：

```json
{
  "camera_source": "uav",
  "camera_frame_id": "uav/camera_link"
}
```

它选择该点 `GROUND_RECOGNIZE` 阶段接收哪一路真实相机。省略两个字段时，行为与之前一致：接收 `source=stage.vehicle`；地面阶段拒绝 UAV 图。显式配置必须同时给出 `camera_source`（`ground` 或 `uav`）和非空、无空白的 `camera_frame_id`。目前不支持在空中点/窗户上配置这组覆盖，避免静默影响其他阶段。

原 `ground_image_topic`、`uav_image_topic` 仍分别订阅各自真实输入；不要将 UAV topic 填进 ground 槽重标来源。`RuntimeInputs` 保留并校验真实 `source/frame_id/stamp`，控制器严格匹配显式绑定；缺帧、错帧、过旧/未来图、处理错误或内外层元数据冲突均无法满足识别。无效更新使该帧和连续识别证据失效，旧 ACK 不能推进新报告。

## 控制和报告语义

`stage.vehicle`、`active_vehicle`、实际位姿、速度、净空门禁和话术 `kind` 全部仍是 `ground`。相机绑定不申请 UAV 位姿，不产生飞行或灯光指令，地面视觉伺服继续仅输出地面 yaw。

识别报告保留：

- `source`、`camera_source`：真实观测来源，例如 `uav`；默认同车相机行为未变。
- `vehicle`、`kind`：任务/控制阶段，例如 `ground`；官方话术按 `kind` 编码，仍发送地面话术。
- `image_stamp`：用于生成该报告的真实图像曝光时间；`stamp` 仍是报告产生时间。
- `frame_id`：仅在真实观测有此字段时复制，不创造缺失frame。

新增相机绑定只附着于该点识别阶段，后续地面点未配置时继续使用地面相机，空中点和窗户继续使用 UAV 相机。阶段切换不继承前一点覆盖或识别dwell。

## 验证与边界

新 `tests/test_ground_camera_binding.py` 共21项，实施前15项失败、6项已有拒绝行为通过；实施后全部通过。覆盖实际运行时解析到控制器的来源/帧/原时间保留、显式使用UAV识别、无配置拒绝、错误/缺失frame及旧/未来图拒绝、dwell/旧ACK重置、ground视觉伺服、后续点不继承、坏配置拒绝、加载器接通及地面官方话术。

连同 `test_mission.py`、`test_runtime.py`、`test_geometry_runtime.py`、`test_official_adapter.py`、`test_official_protocol.py`：**203 passed**。全套回归由主代理统一执行。

本功能仅解决“地面控制使用另一真实相机观测”的软件接线。配置这个frame不自动证明相机到机体外参、双位姿刚性关系、图像所属物理点、当前类别、路径净空或裁判已计分；这些继续由实际观测和后续平台单点练习验证。
