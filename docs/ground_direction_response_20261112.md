# 地面 XY 方向响应实验实现记录

本次交付独立、有界的方向响应实验，复用现有传感器与 ROS 传输通道，不修改已得分路线、单点或三点控制模块。当前只完成本地实现与验证；平台输入输出、物理停距、路线可行性及官方得分均待真实练习证据确认。

## 目的与已知证据

已有 `docs/ground_observation_route_v22_axis_observation_20261112.md` 记录斜线失败与轴向成功，说明本次不需要再用小范围脉冲证明“曾经可以轴向运动”。新增实验用于测量横移与非轴向姿态下的输入输出，支持后续路线选择。

v26 原始轨迹来自 `runs/analysis/20261112_three_point_v26/bundle.json`。使用冻结的 `GroundSynchronizedApproachGuard` 回放原 `drive_origin`、`drive_yaw` 与 17.16s 位姿，仍得到 `approach_cross_track_limit`。14.60–15.24s 的位移 bearing 约为 -0.060563673rad，与段开始 yaw 相差约 0.195rad。因此保留既有横偏保护，先测真实响应；不能把偏差直接解释为某一种滑移或桥接坐标语义。

## 实现与边界

新增文件：

- `src/inspection_mission/inspection_mission/ground_direction_response.py`
- `scripts/run_ground_direction_response.sh`
- `tests/test_ground_direction_response.py`

状态流程为 `WAIT → TURN → TURN_SETTLE → PULSE → PULSE_SETTLE`，完成三次脉冲后锁定 `STOP`。初始仍要求距原点不超过 0.05m、yaw 不超过 0.05rad；先转到 -0.60rad，实测静稳后依次请求 X+0.03、Y+0.03、X+0.10m/s。每次脉冲之间独立归零，并要求源时间和墙钟都静稳至少 1s，漂移及累计静稳路径不超过 0.002m。

全局预算为墙钟 120s、累计路径 0.18m、距永久起点 0.12m、累计绝对旋转 1rad。单次转向不超过 45s，请求 yaw 角速度不超过 0.5rad/s；每脉冲不超过 2s。PAUSE 不重置全局或该次脉冲预算，STOP 不恢复。

依据主控与独审裁定，正常请求零的实测位移阈值收紧为 **0.010m**，保留原 **0.03m** 工程停止预留；**0.04m** 是实测越界锁停阈值，不能解释为物理绝不会越界。归零后另限制累计路径增量不超过 0.01m。样本不足时记录不确定，不自动放大脉冲或增加方向。

完整有限环扫要求最小距离大于 0.641m；源时间及到达墙钟新鲜度均保留 0.20s。scan 必须与真实前序 odom 配对，`odom_stamp <= scan_stamp`，skew 不超过 0.05s；不得借用未来 odom。相同 stamp、完全相同 payload 不刷新活性，相同 stamp 改变内容则锁停；危险或非有限 scan 先校验，不因重复时间戳被忽略。

输出仅为 `/cmd_vel`。真实节点的 `tick` 已接入 `linear_y`，非零发布前再检查实际 DDS 匹配；发布异常记录本地失败、锁停，并独立尝试本通道零命令，watchdog 保持归零尝试。其他已声明的 publisher 始终不发布，不发 UAV、识别、到点、灯光或完成消息。`sent=True` 仅说明本地 publish 返回，不能视为机器人已执行。

四元数原始 payload 原样保留，派生 yaw、倾角及累计旋转使用归一化四元数。专项回归复现了 norm=0.9991 可以掩盖真实 0.03005rad 漂移的候选缺陷，修复后正确锁停。

## 记录与接收

每次脉冲保留原始 pose/header/twist、源 stamp、arrival、起始 yaw、请求 XY、归零请求、静稳状态和源时间位姿差分。`response_from_odom` 输出世界系位移与起始 body 系投影，并固定 `interface_semantics_confirmed=False`；结果只说明观测到的输入输出。

原始 odom、scan、命令、状态分别有 10000、2000、12000、128 条容量及 drop 计数。核心证据和完整 gzip/base64 分片复用既有遥测接收合同，每片 48000 字符，包含原文与压缩文长度、SHA256；编码后的接收信封小于 64KiB。启动脚本为 LF，只启动本队 Python 模块，不启动仿真或裁判。

## 本地验证

先得到模块缺失的预期失败，再实现。新增四元数和发布前 DDS 匹配消失的回归均先观察到预期失败，然后修复实际回路。

- 方向响应专项现有 **44 项**，包含真实 callbacks + `node.tick` 的 Y 输出、危险重复 scan、PAUSE 预算、源配对、静稳与累积预算。
- 方向专项、严格同步、有界地面、接收器相关检查合计 **203 passed**，无失败、跳过或错误；证据为 `runs/review/ground_direction_20261112/final_related.xml`。
- 遥测专项 **4 passed**，证据为同目录 `telemetry.xml`。
- `py_compile` 通过。系统默认 bash 启动了缺失 VHDX 的 WSL，未能检查；改用已安装的 `C:/Program Files/Git/bin/bash.exe -n scripts/run_ground_direction_response.sh`，退出码 **0**。没有为此安装工具。
- 早期一次调用了不存在的 `tests/test_telemetry.py`，当次未收集测试；随后使用真实 `tests/test_probe_telemetry.py` 并通过，以上 4 项来自后一结果。

RED 证据与最终 SHA 在 `runs/review/ground_direction_20261112/`。`freeze_manifest.json` 记录源码、脚本、测试及冻结旧源码哈希。旧 route/single/three/planner 与任务开始的基准 SHA 一致；本子任务未编辑 vertical 模块。

源码 SHA256：`aeecaa1a50b585cfdd13218a94d56283f0ea93f03a71151475512e852bb21946`。

脚本 SHA256：`2b464c13550fcf24d0e891ab19a49c48d09a28f90ed4b581c47ce4946c83bc10`。

## 交接与待验证

独立审查已通过上述冻结 SHA，报告为 `docs/control_review_ground_direction_response_20261112.md`，其 SHA256 为 `b57302e21ae2cca653557677c55273c514bb9294cfdaf183473831ad53b55cc8`。7 个独立探针证据在 `runs/review/ground_direction_response_20261112/independent_probe.py` 与 `independent_metrics.json`，覆盖真实 callbacks + tick 的 402 次发布（含 34 条 Y）、独立 watchdog 归零、相同 stamp 危险数据改变、归零后往返累计路径 12mm（净位移 4mm）锁停、缩放四元数掩盖 0.03001rad 漂移拒绝及响应计算不变性。独审未发现需修改源码项，源码与脚本复核未变。

接下来由主控打包并在平台练习，保留 ID/runId、完整遥测与裁判证据；本子任务不打包、不上传、不操作浏览器。合成测试验证边界和记录逻辑，不是官方平台成功、停止距离或任务得分证明。此次没有新增北向倒车等脉冲，也不把方向实验的结束解释为全任务完成。
