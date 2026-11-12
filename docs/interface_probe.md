# 只读接口诊断包

- 上传源码：`dist/inspection_probe_src.zip`
- 启动脚本：`dist/run_interface_probe.sh`
- 入口：`ros2 run inspection_mission interface_probe --duration 30`

节点无需 `interface_profile.json` 或场地图。它从 ROS 图获取真实消息类型，再订阅公开里程计、TF、传感器及官方公开裁判接口，日志均以 `INTERFACE_PROBE` 开头。日志包含可见话题、消息 schema、发布端 QoS 和前两条消息摘要。

它不向控制话题或裁判发布消息，也不产生或伪造任务完成、得分或裁判结束标志。它只用于取得接口证据，因此它本身不会完成巡检比赛。

若平台日志仍只有“裁判”终端，没有脚本最开头的 `INTERFACE_PROBE launch_start`，无法据此断言节点没有执行；应由平台提供用户脚本 stdout/stderr、编译日志与退出码。