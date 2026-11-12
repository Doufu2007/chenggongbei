# Complete Inspection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans; 用户已要求直接补全执行。

**Goal:** 将现有基础骨架补成包含全部比赛动作的可测试控制器及 ROS2 适配，明确实际平台尚未验证的接口。

**Architecture:** 纯任务闭环与视觉处理独立实现，ROS 节点只做传感器转换、接口健康检查、控制/识别/灯光发布。正式配置不携带虚构航点或识别答案；示例场景仅用于本地运动学集成检查。

**Tech Stack:** Python 3、ROS2 Humble/rclpy、OpenCV、numpy、pytest。

**Spec:** ../specs/2026-11-05-complete-inspection-design.md

## Global Constraints

- 仅使用允许的传感器与公开任务接口；不启动仿真或裁判；不读取隐藏结果。
- 未确认的平台话题类型/坐标系/数据格式必须显式配置，不能静默默认成功。
- 不付费、不创建队伍、不消耗两次正式上传机会。
- 本目录无 Git 仓库，直接在授权工作区实施，不引入无关仓库操作。

## Review Focus

- 连续定稳被错目标、旧图、漂移、相机源混用绕过。
- 穿窗前未到位/未识别/未下降，或低高度一直保持到空中巡检。
- 里程计有发布者却没有更新，坐标系错误，传感器 QoS 不兼容。
- 光照/参考图不匹配，错误的视觉身份被上报。
- 包可静态编译但 ROS 节点初始化/启动脚本不能实际执行。

### Task 1: 完整任务控制器（子代理）

- [x] 先写失败的任务顺序、识别门禁、窗口轨迹、照射定稳与安全测试。
- [x] 新增 mission.py，严格校验任务地图；返回 MissionCommand。
- [x] 实现 ground_order、连续证据、明确报告、超时和停止。
- [x] 跑负责的测试并交付接口。

### Task 2: 视觉与避障几何（子代理）

- [x] 先写失败测试：真实 QR/ArUco、多光照、模板拒识、射线/位姿几何、障碍。
- [x] 更新 vision_core.py/vision_node.py 并新增 perception.py。
- [x] 每路相机使用独立节流、SensorData QoS、CameraInfo；禁止写 Node.subscriptions 属性。
- [x] 跑测试并交付视觉数据契约。

### Task 3: ROS 适配与部署（根代理）

- [x] 为严格消息解析、配置门禁与控制转换写失败测试。
- [x] 新增 protocol.py，替换 mission_node.py；接入真实传感器、灯光反馈、识别确认。
- [x] 配置显式坐标系和控制话题，订阅 QoS 兼容；输出缺失接口诊断。
- [x] 修复启动脚本在任意位置寻找安装包配置、子进程退出与依赖。

### Task 4: 集成审查与交付

- [x] 独立完整任务运动学集成检查及故障测试。
- [x] 尝试真实 ROS2 节点构造/传感器消息集成，环境不足据实记录。
- [x] 独立审查后修复，运行相关测试、合规检查、打包。
- [x] 更新 README、平台接口缺失清单、测试报告与 SHA256。
