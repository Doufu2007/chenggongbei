# 2026-11-06 平台实测与当前阻塞

账号已登录，城市应急全域巡检场景已开通。实际在“仿真测评模拟系统”提交；本轮没有在竞赛中心正式提交作品材料或代码。

| 测评号 | 内容 | 时长 | 结果 |
| --- | --- | --- | --- |
| run_20261105_231853_9250eb18 | 巡检开发包 | 15.3 s | failed，未提取到成绩 |
| run_20261106_002014_40ef0cc3 | shell 公开话题探测 | 11.5 s | failed，未提取到成绩 |
| run_20261106_002558_9b17bc00 | 最小 echo / pwd / sleep 脚本 | 21.1 s | failed，未提取到成绩 |
| run_20261106_004839_665ea9d0 | gnome-terminal window 格式话题探测 | 29.4 s | failed，未提取到成绩 |
| run_20261106_005323_885f0a05 | gnome-terminal tab 格式独立 ROS 只读探针 | 43.1 s | failed，未提取到成绩 |

## 已证实

- 最新测评号 #80236820，详情：https://www.splenach.com/ros-simulation/evaluation/log/fc5b9c04-754c-44f3-8f51-74fa6f1b2ca5
- 服务端原始字段 `status=failed`、`score=null`、`scoreDetail=null`。网页把失败时空分数显示为“0 分”；这不构成有效算法成绩。
- 所有可见运行记录仅返回裁判启动日志。最新 `tabLogs` 只有 `referee`，`terminalLogs=[]`，未见用户脚本标记、节点日志、编译输出或退出码。
- 普通脚本、官方示例中的 gnome-terminal 启动风格均已尝试。仍无法从现有日志区分“脚本未启动”“编译/加载失败”和“用户输出未回传”；不能直接认定平台故障。
- 所有探针只读公开接口，不发布裁判消息、不伪造结束标志、不读取随机答案和 Gazebo 真值。

## 代码状态

- 原有巡检任务控制器的官方接口配置仍 `verified=false`，地图仍有 null；因此尚不能执行真实完整比赛任务。
- 新增 `interface_probe` ROS console entry：动态发现实际类型和 QoS，订阅白名单公开消息，打印字段与有限消息摘要，不发控制动作。
- 专用诊断包：`dist/inspection_probe_src.zip`，脚本 `dist/run_interface_probe_tab.sh`。
- 最新本地全项目 164 项测试通过（子代理执行）；主代理复核探针 4 项通过。本地测试不代表平台任务验收。

## 继续执行所需的最小资料

1. 官方 `demo_inspection_ws.zip`（含公开接口说明、示例任务源码、地图与参考目标图），或者等价的接口与场地资料。当前提供的“测评注意事项 PDF”没有这些定义。
2. 平台针对最新 runId 的用户启动日志、编译日志和退出码；并确认上传启动脚本的工作目录、source 已编译 workspace 的正确方式，以及是否必须使用 gnome-terminal tab。

可发给张老师的说明（尚未发送）：

> 城市应急全域巡检练习测评 run_20261106_005323_885f0a05，记录 #80236820，43.1 秒后 failed：流程未检测到完整结束标志（未提取到成绩）。详情仅有裁判启动的三行日志；tabLogs 仅 referee，terminalLogs 为空。普通 shell 最小 echo 脚本及 gnome-terminal competition tab 只读 ROS 探针都没有用户输出。请提供编译日志、用户脚本 stdout/stderr 与退出码，并确认启动脚本工作目录/已编译 install/setup.bash 路径。另请提供官方 demo_inspection_ws.zip 的下载地址（包含示例接口、场地图和目标图）。

证据文件：`platform_probe_result_20261106.json`、`platform_probe_result_20261106.png`。