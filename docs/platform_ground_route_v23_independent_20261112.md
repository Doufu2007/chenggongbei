# v23 平台实测：有限观察路线独立复核

2026-11-12。本轮**地面首点外观察路线完成**；平台最终状态仍为 **failed，未提取到成绩**。这两项分别有本队运行证据与平台列表证据，不能合并为正式测评成功。

## 证据身份及完整性

- 平台记录：`9fdfe2de-a2f6-4ea5-8f62-74ff099ed85d`；runId `run_20261112_051547_2766d9ab`；练习版 `ground-route-v23`。
- 权威状态缓存：`runs/evidence/20261112/route_v23_list_final.json`；`status=failed`、`score=null`、平台耗时 `171.756s`，失败原文“流程未检测到完整结束标志（未提取到成绩）”。
- 原始回传：`runs/evidence/20261112/callback_route_v23.jsonl`，674,008 B，SHA256 `a9382aa92a55ef039802a6a35fcf5725f89c408e87101b0545a7b500083c1407`。
- 新独立解析：`runs/analysis/20261112_route_v23_independent/summary.json`，SHA256 `71759fccbb80be84adc69088580b695d5ec86ce34d1947ac80dd77a367e00be5`。
- 可复算脚本：`runs/review/20261112_route_v23/review_v23.py`；结果 `independent_metrics.json`。

共 60 个事件，984 行 10Hz 轨迹、2,208 行实际发布记录、7 张原 JPEG。两段 series 均完整，无缺行、冲突、解析、组装、诊断或图片错误；末尾有 `launcher_exit=0`。本队核心 `FINISHED / route_observation_complete`，耗时 `99.806s`，`errors=[]`。命令 series 含最终关闭归零，较核心计数多一条，未将两者混同。解析器 `partial` 表示没有 full-snapshot 事件，不是该局平台成功状态。

## 本轮实际完成了什么

真实抽签顺序为 `ground_1,ground_4,ground_3`。仅执行首点候选外观察路线 `(0.6,0) → (0.6,-0.5)`，最后原地朝向该候选板；未进入目标任务点。此局没有执行 ground_3 的长路线或 ground_4 的新观察分支，也不与首点不同的 v22 构成配对性能比较。

| 复核项 | 原始/复算结果 | 对应门限与范围 |
| --- | --- | --- |
| 第一腿静稳航点误差 | `0.001077196m` | 小于原 `0.02m` 航点门 |
| 第二腿静稳航点误差 | `0.006628703m` | 小于原 `0.02m` 航点门 |
| 第一腿停车请求后累计 path 增量 | `0.000619064m` | 小于新增普通停车 `0.01m` 门 |
| 第二腿停车请求后累计 path 增量 | `0.000059337m` | 同上 |
| 请求到首条实际 DRIVE_SETTLE 零发布记录 | `1.567ms / 1.683ms` | 本队记录间隔；不是执行器收到命令的延迟 |
| 最后转向后 scan 的真实前序位置 | `(0.589627044,-0.496022911)m` | 对终航点误差 `0.011109251m` |
| 最后转向平移 | `(-0.004027217,+0.002061025)m` | 净位移 `0.004523969m`，保留原转向保护 |
| 末转航向残差 | `+0.024162431rad` | 对实际重算目标 `-2.179402343rad`；小于原 `0.025rad` 窗 |
| 核心末 odom 航点误差 | `0.011116390m` | 另列末时刻，不代替 final_heading 时刻 |

停车 path 为原 guard 从阶段请求锚开始、按接收到的 odom 累加的路径，包含首条实际零发布前运动；它不是精确的发布后制动距离、执行器 ACK 或连续物理最坏界。每腿 10Hz 采样与零发布记录已另存于机读结果，不用低频净位移冒充累计路径。

本轮出现一次 PAUSE，记录区间约 `69.410–72.021s`，随后恢复；原恢复上限和时间预算保留。最终 STOP 约 `97.776s` 后持续归零约 2 秒再 FINISHED。终端快照的 guard scan 已是终止前记录；其末时刻 stale 不冒充全程传感器连续性结论。

## 三张真实 CAPTURE

| 阶段 | 图像 stamp | 各自前序 UAV odom stamp | 实测时间差 | 对末航点误差 |
| --- | --- | --- | --- | --- |
| capture_0 | `98.724s` | `98.720s` | `0.004s` | `0.011109723m` |
| capture_1 | `99.058s` | `99.040s` | `0.018s` | `0.011110312m` |
| capture_2 | `99.391s` | `99.380s` | `0.011s` | `0.011110875m` |

原图保存在独立解析目录，文件分别为 `native_L000017_2b89c8c146b94f26.jpg`、`native_L000019_c58978ea3198d118.jpg`、`native_L000021_27517137f6dfb1b0.jpg`。三图各有不同 stamp、不同 SHA，所有文件 SHA 均和原始 JPEG transport 元数据一致，保留真实 `/uav/camera/image`、`uav/camera_link`、各自前序 `/uav/odom` 与完整 pose。没有更换来源、跨帧借 pose 或插值。这里只确认真实图像与位姿绑定，不据此宣布类别正确或裁判接受。

## 候选几何、QoS与计分边界

对 start、first_leg_end、observation_arrival、final_heading 四个原 scan，使用各自原前序 odom 与固定第三方候选 world，保持 identity 假设，不拟合、不插值：每段 360 束，共 `1440/1440` 在既定 `0.001m` 残差门内；四段最大残差依次 `2.608 / 1.431 / 1.182 / 1.339µm`。该结果支持本局静态几何相容，不独立证明官方点号、类别、全程安全或得分。

两路 odom 实际订阅仍为 depth 1、BEST_EFFORT、VOLATILE；发布端观察为 RELIABLE、VOLATILE。六个输出端点均 matched=1，只有 `/cmd_vel` 非静默。`/inspection/detections`、`/referee/current_target`、`/referee/cruise_event`、`/uav/cmd_vel`、`/uav/laser_fire` 全部仍静默；`/referee/recognition_status` 接收为 0。没有上报识别、进入声明、飞行、照射或完整结束，因此官方成绩仍缺失。

下一闭环需要本局运行时真实视觉证据、准确地面话术、合法进入和到点声明，以及裁判接受/计分证据。此次有限路线通过只为这些步骤提供输入；不构成全任务成功、95% 稳定率或获奖保证。
