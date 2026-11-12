# 首条真实地面观察路线：v2 部署修复

## 依据与约束

v19 真实回传位于 `runs/analysis/20261112_speed_v19/summary.json`。主代理确认高档请求 0.10 m/s 的短脉冲在约 0.761 s 前进 0.040947 m，峰值实测速度 0.0595459 m/s；高档含停车累计路径约 0.0417303 m，触发停车后累计路径约 0.000773 m，两次静稳均通过。总投影约 0.200286 m，但 CAPTURE=0；后段 `/uav/odom` 原 RELIABLE 订阅出现约 0.18 s 数据间隙，图像没有满足原 0.05 s prior 关联窗口。

短脉冲响应与停车通过仅支持继续尝试独立、有界的观察路线，不证明持续高档响应、路线可达、识别或比赛得分。v19 文件与既有 dist 不覆盖。

## 最小变更

- 共享适配器白名单仅从 `/odom` BEST_EFFORT 扩至 `/odom` 与 `/uav/odom` BEST_EFFORT；默认值、其他输入、全部输出 QoS 与原 v18/v19 factory 参数不变。
- 独立 route factory 显式请求两路 odom BEST_EFFORT，并接入原 v17 `CallbackTimingRecorder`，revision=`ground_observation_route_v2`。不改传感器 freshness、clock lag、图像 prior、PAUSE、恢复次数或几何/运动硬停。
- 纯计划时间默认与执行值统一：全局 300 s、累计 DRIVE 190 s、单次 TURN 45 s、累计 TURN 100 s、CAPTURE 10 s、等待 5 s。执行时若调用方提供更紧的合法时间预算，执行计划元数据同步保存真实值。
- 启动脚本仍传 `--duration 300`，仅新增同一 v2 revision；不启动仿真或裁判，不发送任务、识别、飞行、灯光或得分消息。

## 时间可达性与首次运行目标

原点出发，ground_0/1/2/3/4 的候选路长分别为 1.05/1.10/2.40/1.625/0.353553 m。前四个点是两腿，最长 ground_2 为 0.60+1.80 m；ground_4 为单腿。它们基于公开候选世界，仍需实际 scan 与图像验证。

按 v18 低档观测中位速度约 0.0049558 m/s，纯慢档名义行驶分别约 212/222/484/328/71 s，前四个不能满足既定累计 DRIVE 190 s。v19 高档短脉冲的平均响应约 0.05381 m/s；若后续持续响应保持，最远路线除末尾 0.10 m 慢段外约需 43 s，末尾慢段约 20 s，再加停车仍低于 190 s。这是条件估算，不能当作持续速度已验证。

最大计划累计转角约 2.18546 rad；各两腿路线包含约 90° 中间转向，长腿还使用较窄的对准容差。单次 45 s 与累计 100 s 是否足够仍待首条真实路线确认。首次运行应保留所有硬停，按真实公开抽签首 ID 执行，目标为到达观察位、完成最终纯转向、接收三张不同原始 stamp 且 prior 合格的 UAV 源图。该路线不产生官方计分声明。

## 验证

新增双 odom override、真实 route 订阅/回调计时、默认及更紧时间预算一致性 5 项回归先全部失败，最小实现后通过。另补无效 UAV override 边界，保持只接受指定 BEST_EFFORT。

```text
python -m pytest -q tests/test_ground_observation_route.py tests/test_ground_observation_route_plan.py tests/test_ground_odom_qos.py tests/test_ground_speed_calibration.py tests/test_ground_callback_timing.py tests/test_ground_bounded_approach.py tests/test_ground_bounded_approach_sync.py tests/test_bounded_ground_probe.py
219 passed in 14.61s
```

其中包含旧 v18/v19、原同步 guard、启动脚本真实 Bash 上传工作区与退出码回归。外部 ROS 边界为本地 mock；控制逻辑、真实适配器、计时记录与 Reporter 编码照常运行。这是本地回归证据，后续由主代理独立复核、统一全套、打包并上传平台实测。
