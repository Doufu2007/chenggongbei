# v19 独立短距双档与停车校准计划

已获主代理授权直接实施；只新增模块、脚本与测试，不改 v18、长路线或 PAUSE 机制。

目标：沿用 v18 已实测的 `/odom` BEST_EFFORT、v17 计时、原同步 guard、90 s 全局、55 s DRIVE、0.20 m 总投影、0.30 m 路径/永久原点、1.8 rad 旋转、局部 0.03 m 移动加 0.03 m 停车储备、0.20 s freshness、0.12 s clock lag、0.05 s prior skew、最多三次恢复及原全部观测速/倾角/航向/横偏硬停。

1. 新增 `ground_speed_calibration.py`：继承同步 approach。正常转向后第一段全程请求 0.03 m/s，连续 wall/sim 时间至少各 1 s；到首 0.05 m 请求停车。以触发时累计路长作锚，之后增加不得超过 0.01 m（包含首次实际成功发布零速之前的路长），并要求原静稳 1 wall/sim 秒。
2. 独立高档阶段只请求 0.10 m/s：从首次高档非零请求冻结 wall/原 odom stamp，任何一个时间达到 5 s 而未完成截止都硬停，PAUSE 不重置。主代理接受更窄距离裁定：实际投影增加 0.04 m 时请求零，整个高档阶段含停车累计路长不超过 0.05 m，post-trigger 路长仍单独不超过 0.01 m。实际观测速 hard 0.10 m/s 不放宽；异常/时限仅硬停报告。
3. 第二次停车与静稳通过后，请求 0.03 m/s 至总 0.20 m，再沿用最终静稳与三张原始图像。两档的证书分开；短脉冲通过也不宣称已验证长段持续 0.10 的响应。
4. 每档保存 bounded 的样本 count、原 stamp/位置/速度首末、最大实测速度及投影/路长；每次停车单独保存 phase 切换与首次成功零速 publish 事件、路径增量、静稳 wall/sim。后者只证明本队 publish 返回，不能代表执行器 ACK。
5. 先写失败测试：完整顺序、初档不可提早加速、两次停车超界/瞬时超速锁停、高档 wall/sim 与 PAUSE 不重置、全局原点与时限、成功零发布边界、v18 QoS/计时与 payload 有界。看红后最小实现，再跑必要旧回归。主代理统一全套/打包/官方实测。

文件：`src/inspection_mission/inspection_mission/ground_speed_calibration.py`、`scripts/run_ground_speed_calibration.sh`、`tests/test_ground_speed_calibration.py`。本地反馈是控制逻辑测试，不是平台性能或比赛成绩。

## 实施与验证记录

- 新增 11 项控制回归，先全部因独立 v19 模块不存在而失败，再实现到通过。
- 新增两项显式更紧约束测试，先失败，再拒绝与本实验固定 0.03 m/s 慢档、0.20 m 总距冲突的 `ApproachLimits`，避免静默覆盖调用方更紧约束。原其余更紧预算继续继承。
- 补测真实适配器的 publish 异常：发布失败时不得生成首次成功零速证据；完整 metadata 经实际 `DiagnosticReporter` 编码仍小于 64 KiB，保留两次停车与每档统计。
- 连续低档证书的后续暂停回归先失败：原统计从 1.0 s 增长至 2.25 s，会把后来暂停计入“连续”时长。现已在取得证书时冻结 wall/sim 时长，不修改 PAUSE 或恢复机制。
- v19 共 16 项新回归，含两次 post-trigger 累计路长、首次零发布前路长、高档 0.04/0.05 m 包络、双时钟 5 s 跨 PAUSE、实测速度 0.1001 m/s 硬停、原永久原点/55 s DRIVE/90 s 全局/三次恢复及三张源图完成流程。

2026-11-12 必要定点回归：

```text
python -m pytest -q tests/test_ground_speed_calibration.py tests/test_ground_odom_qos.py tests/test_ground_callback_timing.py tests/test_ground_bounded_approach.py tests/test_ground_bounded_approach_sync.py tests/test_bounded_ground_probe.py
166 passed in 6.97s
```

另用真实 Bash 执行启动脚本的隔离上传工作区夹具（目录含空格）：`bash -n` 通过；正常子程序退出 0、子程序退出 23、ROS setup 退出 31 三种情况均保留真实退出码，成功 source 后传入独立模块的参数准确为 `--duration 90`。未调用仿真或裁判启动命令。

本机无 ROS；测试只在外部 ROS 边界使用 mock，控制器、同步 guard、解析器及 Reporter 编码执行真实代码。这证明本地控制逻辑与证据封装，不证明平台响应、持续高档资格、自动计分或比赛成绩。平台实测、全套回归与打包由主代理统一执行。
