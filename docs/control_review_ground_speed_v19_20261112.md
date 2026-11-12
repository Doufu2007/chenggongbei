# v19 双档短距校准：独立只读审查

日期：2026-11-12。结论：**本次未发现阻断既定有限平台实验的问题。** 审查只读冻结源码、启动器及其继承链，另执行 9 项独立离线反例；没有改生产代码、启动脚本、旧包或原始证据，没有操作网站或提交新测评。该结论仅放行已授权的短距校准，不是持续 .10m/s 路线资格、实测性能或官方成绩。

冻结三项 SHA256 已逐文件核对，反例运行后仍相同：

| 文件 | SHA256 |
| --- | --- |
| `src/inspection_mission/inspection_mission/ground_speed_calibration.py` | `98437fe45a1aaeee73a1ff6bf9976955acffeff99d135740109ffa27d220fc43` |
| `scripts/run_ground_speed_calibration.sh` | `f1de21365073897c384c4428584cc537786da0e396d1a3145ad1de90b574f204` |
| `tests/test_ground_speed_calibration.py` | `749a5557eb6947bb08a4aaccd3e5caa3a4f138a929677ba2b9a6a8b38ce3dae3` |

## 审查覆盖

- 两次停车分别在真实反馈达到触发条件时冻结 `path_origin_m`；首次成功零速发布是在 adapter 的 `publisher.publish()` 正常返回后才由 `record_command` 记录。phase 切换不冒充发布成功，触发到首次发布之间的全部累计路长仍计入 .01m 停车预算。失败发布会抛异常，不形成成功发布记录；此记录仍不是执行器 ACK。
- 初档仅请求 .03m/s，至少各 1 wall/sim 秒后证书冻结；未证时出现 freshness 暂停会清空该计时。后续暂停不会将证书持续时间扩长。首 .05m 触发停车后，须有真实零速发布及原静稳驻留才能进入高档。
- 高档只在 `HIGH_PULSE` 请求 .10m/s，原始首请求 wall/odom stamp 冻结，PAUSE 保留 phase 与锚，任一时钟达 5s 即硬停。前向增量 .04m 触发停车，含停车累计高档路长 .05m、触发后累计路长 .01m 继续受控。进入 HIGH_STOP 后不再请求高档；停车仍受 .05/.01 路长及原 55s DRIVE/90s 全局限制，不将 5s 称为整个静稳过程的期限。
- 原永久原点、累计路长 .30m、最大起点位移 .30m、旋转1.8rad、横偏 .02m、heading .05rad、局部 scan anchor .03m/停车储备 .03m、实测速度 .10m/s、倾角及角速上限均继承。新增阶段加入 ACTIVE/ZERO_PHASES，原同步 freshness/scan 全环 gate 与最多三次恢复仍执行；任何硬异常不会由后续安全帧恢复。
- 完成第二次停车后回到 .03m/s，沿用总 .20m 停止、静稳和三张原始采图；未添加识别、到点、飞行或裁判完成输出。工厂保留 `/odom` BEST_EFFORT、其余 v18 QoS 和计时边界。启动器启动独立模块，不启动仿真或裁判；源码定位及退出码继承已有启动器模式。

## 独立反例结果

`runs/review/20261112_speed_v19/targeted_review.json` 保存 9 项结果，均为真实 Python 控制器/adapter 加外部 ROS mock 的**离线输入**，不是平台传感器或平台测评。

1. LOW_STOP、HIGH_PULSE、HIGH_STOP 各注入一束 .2m unsafe scan：全部 `scan_invalid:scan_too_close` 锁停，后续安全刷新保持零速与原原因。
2. 两次停车分别延迟实际零速 publish，再使累计路径越过 .01m：分别 `calibration_low_stop_path_limit` / `calibration_high_stop_path_limit`，首次成功零速仍为 null、verified=false。证明前置停车运动不被漏算。
3. 高档进入 PAUSE 后持续不提供新 scan：所有命令零速，5s 墙钟到期 `calibration_high_wall_time_limit`，高档首请求锚不变，resume_count=0。
4. 初档一秒证书尚未取得时触发暂停：计时归零，恢复时不借用暂停时长。
5. 用真实模块离线走到 CAPTURE 所生成的 `speed_calibration` 字典，经既有分析器深拷贝后逐值不变；score=null、task_completion=unverified，无 parse/assembly error。无需更改生产分析器。
6. 真实 adapter 在高档 tick 和随后零速 publish，只向 `/cmd_vel` 发送；其他已声明输出保持静默，`/odom` BEST_EFFORT 与 `/scan` RELIABLE 配置不变。

本代理没有重复全套。实现代理报告必要定点 166 passed；主代理另报告全套 XML **1095 tests / errors0 / failures0 / skipped0，62.924s**。这些本地检查都不替代平台响应、停车实测、完整巡检与裁判计分。平台下一局应先验证实际首零发布、两段停车路径、档位响应和原图，再据真实反馈推进。
