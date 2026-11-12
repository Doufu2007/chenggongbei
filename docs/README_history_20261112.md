# 晟功杯全域巡检任务工程

2026-11-12 的 v25 已在官方练习平台取得首个真实地面点 **10 分**：测评 `#86630201`、`run_20261112_060713_501f412f`，官方状态 `success`、总用时 `275.179s`，赛后诊断明确“ground_0 到达并计分 +10”。本局完成三帧在线识别、一次话术发送、实际入区停车和一次到点声明；修复前一版因实际约3Hz相机与0.2秒图像活性门冲突而误停的问题。来源见 [官方终态](runs/evidence/20261112/single_point_v25_list_final.json) 和 [计分截图](runs/evidence/20261112/single_point_v25_official_success.png)。**这只验证首个地面点；三个地面点、两窗、两空中点及降落的完整任务尚未跑通。**

正式配置选择 `protocol: inspection_v1`。流程由 `mission.py` 驱动：五个候选地面点按本局公开抽签选三点，依次在点外识别、进入点内并声明到点 → 地面返航 → 起飞 → 窗口1声明、识别、下降和穿越 → 恢复巡航高度 → 两个空中点先识别确认，再稳定照射 → 窗口2声明、识别、下降和穿越 → 水平返航 → 降落前声明 → 实际降落。早期人工演示模式只用于离线回归。

每次识别绑定相机来源、观测时间、图像中心、实际视觉身份、物理任务点和本局报告 ID。默认 `event_ack` 等待明确确认；显式 `transport_only` 仅根据本队话术发布返回、等待和持续观测推进，不代表裁判接受。旧图、错误来源、多候选歧义、漂移、灯光反馈中断会清空连续证据。地图可用 `target_id: "*"`，现场身份由图像产生，禁止预填本轮随机答案。

视觉支持 QR、ArUco，以及用户提供的参考图 ORB/RANSAC 匹配；处理当前帧光照，拒绝相似模板的身份歧义。已知真实尺寸和 CameraInfo 时可估计相机坐标；不会猜测尺寸或相机外参。可选视觉对准只在已验证相机方向时做有界偏航，不做未知外参下的升降/平移。竖直对准仍需测量观察位姿或另接真实云台合同。

新增 `target_geometry.py` 提供完整姿态下的光学坐标到世界坐标定位、下视灯悬停求解与照射误差评估，要求显式帧、版本、外参和误差界；51项专项及独立复核通过。它尚未接入运动控制器，需要真实曝光位姿、标定与灯/测距合同后再接线，不能据此宣称空中照射已在平台通过。2026-11-12 02:25 的既有代码基线为917项本地测试通过，证据为 `dist/geometry_v14_full_suite.xml`；该结果不包含随后新增且尚未冻结的接近观察模块。

导航支持明确经由点和速度闭环。激光按拟运动方向检查机器人扫掠范围；可选点云检查3D通道。当前支持接近平面的姿态合同，倾角超限停止；无点云时竖直净空依赖测量地图，不能称完整动态3D避障。它不包含未知场景自动建图和全局重规划。

## 本地验证

```powershell
python -m pytest tests -q
python tools/mission_scenario.py
python tools/preflight.py .
python tools/build_submission.py
```

`mission_scenario.py` 是明确标注的人工地图与运动学模型，用合成观测检查完整控制顺序和故障门禁，**没有官方仿真、裁判或真实成绩**。旧 `tools/dry_run.py` 仅保留早期航点控制回归，不是比赛验收工具。

## 接通正式环境所需资料

已从官方课程 API 取得《应急救援机器人挑战赛裁判系统判分细则 V1.0》，原文保存在 `docs/referee_rules_v1.txt`。已据此绑定抽签 CSV、识别话术、`current_target`、`cruise_event` 和灯光命令。内部视觉观测使用 `/inspection_mission/observations`，裁判话术使用 `/inspection/detections`。

免费环境配置课程展示了 `demo_inspection_ws.zip`，尚未取得实际官方下载链接。现已用公开候选世界与本队真实激光、里程计和图像推进等价资料验证：v6 静止及 v14 约2cm移动后的首末激光均与固定候选几何相容，v11—v13 地面板投影与真实相机照片位置相容，足以继续有限路线实验。它们不等于官方版本确认、完整外参标定或隐藏贴图答案。当前地面相机近乎全绿，UAV前视可用于明确绑定来源的地面观察；公开12张参考照片与固定CLIP方案在v13实景斜视ROI上仍未形成可靠类别证据。控制响应已取得；识别接受载荷、空中运动、照射与结算仍待验证。每局必须重读公开抽签，内部报告UUID不能假定由裁判回传。

确认这些资料后填写 `src/inspection_mission/config/` 的 `mission_plan.json`、`interface_profile.json`、`inspection.yaml`。默认模板保留空白，启动前会明确阻塞，避免运行虚构航点。详细字段和仍缺内容见 [docs/platform_submission.md](docs/platform_submission.md)。

生成文件为 `dist/inspection_src.zip`、`dist/run_inspection.sh`、`dist/SHA256SUMS.txt` 和 `dist/readiness.json`。`platform_run_verified: false` 表示尚未完成正式平台验证。
