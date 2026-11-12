# 全域巡检 ROS2 提交包实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 生成一个适配晟功智能 ROS 仿真测评平台的轻量全域巡检 ROS2 Humble 提交包，覆盖地面与空中巡检编排、视觉识别、超时保护、静态合规检查和可上传 ZIP。

**Architecture:** 采用标准 ROS2 消息和参数化话题名，地面/空中控制分别封装为可替换执行器；任务管理器使用有限状态机编排初始化、地面巡检、空中巡检、返航和结束；视觉节点只使用相机图像、CameraInfo、里程计/TF 与平台允许的脱敏任务信号，不依赖 Gazebo 真值或随机化文件。

**Tech Stack:** ROS2 Humble, Python 3, rclpy, geometry_msgs, nav_msgs, sensor_msgs, std_msgs, cv_bridge, OpenCV（可选 QR/ArUco）。

**Spec:** 用户请求、平台评测指南与 PDF《ROS仿真测评系测评注意事项》；平台提交目录必须为 `src` 源码压缩包 + `.sh` 启动脚本，源码 ZIP ≤20MB。

## Global Constraints

- 启动脚本只启动自己的节点，不启动仿真、裁判或 RViz。
- 不订阅 `/gazebo/*`，不读取随机化结果文件，不使用 `/referee/delivery_order`。
- 不在脚本中出现 `gzserver`、`gzclient`、`gazebo`、`ign gazebo`、`ros2 launch ican_*_sim`、`ros2 run ican_*_referee*`、apt/dpkg/.deb 安装命令。
- 代码必须可在没有视觉帧、没有位姿或任务消息时安全降级并持续输出心跳。
- 源码包不携带大模型权重；所有话题名、航点和阈值通过参数或环境变量配置。

## Review Focus

- 话题无发布者：节点不能把 `topic list` 的名字当作数据可用，必须依赖实际回调/超时状态。
- 光照随机：视觉识别不能只使用固定亮度阈值。
- 任务消息缺失或格式变化：状态机必须超时并进入安全结束状态。
- 控制接口无响应：必须停止发布危险速度并记录故障。
- 提交包结构错误：ZIP 必须包含 `src/<package>/package.xml`，脚本必须可执行且不含禁用命令。

### Task 1: 可测试的纯 Python 任务状态机

**Files:**
- Create: `src/inspection_mission/inspection_mission/fsm.py`
- Test: `tests/test_fsm.py`

### Task 2: ROS2 任务节点、视觉节点和配置

**Files:**
- Create: `src/inspection_mission/package.xml`
- Create: `src/inspection_mission/setup.py`
- Create: `src/inspection_mission/setup.cfg`
- Create: `src/inspection_mission/resource/inspection_mission`
- Create: `src/inspection_mission/inspection_mission/mission_node.py`
- Create: `src/inspection_mission/inspection_mission/vision_node.py`
- Create: `src/inspection_mission/config/inspection.yaml`
- Test: `tests/test_config.py`

### Task 3: 评测启动脚本和本地合规检查

**Files:**
- Create: `scripts/run_inspection.sh`
- Create: `tools/preflight.py`
- Test: `tests/test_preflight.py`

### Task 4: 构建提交 ZIP、文档和许可证说明

**Files:**
- Create: `README.md`
- Create: `docs/platform_submission.md`
- Create: `THIRD_PARTY_NOTICES.md`
- Create: `tools/build_submission.ps1`

