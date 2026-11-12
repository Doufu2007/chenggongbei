# 官方环境绑定与提交状态

当前状态（2026-11-12）：已在官方练习平台验证本队源码启动、传感器接收、有限原地转向、传感器过期暂停恢复和约2cm闭环短直行；静态候选地图已用本队激光与图像核对。**完整任务仍未取得有效计分，默认正式配置尚未完成绑定。** 最新已完成的控制实验为 v13 转向与 v14 短直行，详见对应平台报告；接近观察模块仍在开发。

## 已核实的官方资料

《ROS仿真测评系测评注意事项》允许使用 `/odom`、`/uav/odom`、TF、相机与激光；提及 `/referee/ground_tasks`、`/referee/current_target`、`/referee/recognition_status` 等脱敏接口，但没有定义消息类型、字段或控制命令。原始事件和真值/随机结果不得使用。代码不访问这些通道。

课程地址：https://www.splenach.com/courses/43296edb-1673-4643-8922-f13a7523f892/video 。官方视频开头显示 `demo_inspection_ws.zip`；页面作业为空，未发现该 ZIP 的可见下载链接。视频已下载至用户 Downloads 供核对。本机未找到官方工作空间。

后续已取得官方《应急救援机器人挑战赛裁判系统判分细则 V1.0》：https://www.splenach.com/api/courses/129d6e75-f19a-4c97-9ec6-0853695a4c48 。原文保存在 `referee_rules_v1.txt`。表1确认 `ground_tasks` 为 CSV String、识别话术为 String、`current_target/cruise_event` 为 String、`laser_fire` 为 Bool；表5确认12条中文话术。以上内容已经接入配置及编码器，不再属于未知字段。

## 尚需绑定的合同

| 内容 | 必须核实的值 |
| --- | --- |
| 地面/空中运动 | 真实话题、消息类型；UAV Twist 是世界系还是机体系；速度模式能否起降 |
| 起降 | 如需触发：真实 takeoff/land/flight_abort 话题与字段，紧急中止语义、自动起降最大速度 |
| 里程计 | 地面/UAV frame_id，地面与UAV各自地图的坐标约定；不自动猜 TF |
| 相机 | 两路 Image、CameraInfo 的真实话题，光学帧与可选真实目标尺寸 |
| 目标识别 | 七类官方图像的实际图库；recognition_status 类型/字段/时序及如何关联当前点与本次报告 |
| 本轮任务 | 实际公开抽签已观察到 ground_0..ground_4；同号静态区域映射仍以可检验候选假设记录，继续用真实观测验证到点 |
| 灯光 | 反馈话题、类型、开关反馈值与发布频率；灯光轴线和相机光轴关系 |
| 几何与避障 | 测量地图、观察位姿/返航/经由点、窗口净空、真实机体半径；扫描帧和偏航外参 |

这些剩余字段尚未出现在已取得官方资料中。未确认时不能把示例值标成官方接口。

## 文件与字段

`mission_plan.json` 仅含队伍测量的公开场地几何：

- `home_ground: [x,y,yaw]`，`home_uav: [x,y,landing_z,yaw]`。
- `protocol: inspection_v1` 为正式适配模式，绑定层要求五个唯一候选点ID，每轮根据公开抽签选三点；实际公开抽签已包含 `ground_0..ground_4`，模板中的 `ground_1..ground_5` 是尚未回填的旧占位符。`aerial_points` 恰好2项。地面点含点外观察 `approach`、点内 `pose` 和可选 `via`，须结合真实地图观测绑定。地面姿态3维，空中4维。显式 legacy 人工演示仅供旧测试。
- 两个 `windows` 含 `id,target_id,approach,exit,crossing_z,bottom,top`，可选 `via`。出口高度须等于低空穿越高度；识别后必须观测到真实下降，再以满足机体尺寸、净空和到位误差的高度穿越窗口。官方 V1.0 未规定固定下降距离。
- `home_ground_via` 和 `home_uav_via` 支持避障路线；地面顺序随机时，经由点必须对每个实际起点都安全。UAV返航经由点必须保持指定巡航高度，最后再降落。
- `target_id: "*"` 表示识别当前位置实际出现的唯一目标，报告使用相机观测身份。具体标签仅用于任务公开指定或已知图库类别，不得填写本轮隐藏答案。
- `settings` 可调整有界速度、超时、稳定时间；空中照射在1–3秒，高度不得低于0.8m。可选 `visual_servo_enabled:true` 必须显式给 `camera_yaw_sign:1/-1`；只做水平偏航，假设两路相机相同方向合同。

`interface_profile.json` 是本工程适配器配置，含部分已确认官方 V1 字段，其余仍为空：

- 真实核实后才能设置 `verified:true`；必须填 `ground_frame,uav_frame,uav_velocity_frame,flight_mode`。
- `channels` 每项含 `topic,type`，传感器 QoS 默认 best_effort，其他默认 reliable；可通过 `qos` 显式指定 reliability/durability。
- 地面/UAV速度目前适配 `geometry_msgs/msg/Twist`；位姿 `nav_msgs/msg/Odometry`；扫描 `sensor_msgs/msg/LaserScan`。不匹配这些类型的官方环境需要另写适配，不能改字符串冒充。
- scan 必须含 `frame_id,yaw_offset_rad`。可选 `uav_cloud` 为 `sensor_msgs/msg/PointCloud2`，点需在已确认的机体坐标帧。默认倾角上限0.05rad，可显式调整但不得超过0.15rad；超限拒绝平面几何。
- 正式识别采用 `encoding: inspection_phrase_v1`、`fields: {data: $phrase}`，由实测中文类别生成表5准确话术。`current_target/cruise_event` 使用 `{data: $value}`，灯光使用 `{data: $enabled}`。内部视觉观测为 `/inspection_mission/observations`，内部心跳为 `/inspection_mission/status`，均不冒充官方裁判通道。
- `ground_tasks` 采用 `encoding: csv,path: data,id_field: '',selection_count: 3`，严格拒绝重复、未知、空点或错误数量。通用 JSON 解析仍用于离线兼容。
- 当前 `recognition_ack` 的 `path,id_field,accepted_field` 解析器按内部报告 ID 匹配，**该关联方式尚未被官方确认**；官方上报话术不包含本队 UUID，不能假定裁判会回传该 UUID。拿到真实公开状态 schema 后仍需实现和验证确认关联适配，不能仅填类型便声称完成，也不得把任意状态消息当成功。
- `light_feedback.path` 提取显式布尔，支持 Bool 或 String JSON。需要持续新鲜的硬件/平台反馈；仅发命令不算照射成功。
- `flight_mode:trigger` 还必须确认可中断语义，提供 `trigger_abort_stops_motion:true`、`trigger_max_speed_mps` 与 `takeoff/land/flight_abort` 消息模板；缺少时拒绝运行。

`inspection.yaml` 控制节点参数和双相机；两节点 `use_sim_time:true`，检测话题一致。非空 plan/profile 路径及参考图路径必须是**运行容器内的绝对路径**，不能填写 Windows 路径上传。空 plan/profile 使用安装包 share/config 文件。可先编辑源码配置再重新打包。

## 启动和提交

启动脚本先 source ROS/已编译工作空间，通过 `ros2 pkg prefix --share inspection_mission` 定位安装配置，验证配置后只启动本队两个节点。外部 `INSPECTION_CONFIG` 参数文件若留空 plan/profile 路径，预检也显式使用同一安装包 `share/config` 根目录。不会安装系统软件、启动仿真/裁判/RViz。节点退出时清理子进程；Linux有setsid时按进程组清理。

打包 ZIP 必须含 `src/inspection_mission/package.xml`，源码上限按当前页面20MB。`.sh` 为独立第二文件。生成器排除缓存并输出校验值和明确就绪状态。静态合规检查只是有限lint，不是完整安全审计或平台编译证明。

用户已开通场景权限并登录。平台不展示用户构建/运行日志，当前通过本队临时回传通道保存真实诊断证据，已验证代码运行与有限运动。v13 `run_20261112_021236_269a9fac` 转到当局首点观察方向；v14 `run_20261112_021755_cef799e3` 实测前进约2.02cm停车。这些包不执行完整巡检、不发识别或到点声明，平台仍显示“未提取到成绩”；不能据此宣称完整任务成功。源包、脚本、哈希与平台终态均见 `runs/records/` 和对应报告。仅使用练习入口，未购买或修改队伍。

本机无可用ROS2/rclpy/Gazebo运行环境；本地pytest和人工运动学不是官方仿真。上述真实ROS传感器与运动证据来自官方平台容器；仍无完整任务成绩或正式效果录像。
