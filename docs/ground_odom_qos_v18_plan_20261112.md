# v18：只改变地面 odom 订阅可靠性的对照

## v17 证据与判断

真实原始文件为 `runs/evidence/20261112/callback_approach_v17.jsonl`，SHA256 为 `9cc1380ef74ce12020684dcf1993c09329303709d8000d4182f1bf8e50619e99`。独立核对原始 core 的 `callback_timing` 与分析器 assembled 完全一致。

| v17 测量 | 实测值 | 含义 |
| --- | --- | --- |
| 全部 receive wall / 当前线程 CPU 总耗时 | 2.889 / 2.882 s | 本队回调处理没有持续占满主线程 |
| `/odom` 回调 | 均值 0.383 ms，最大 12.613 ms；2901/2905 次不超过 1 ms | 与后段约 180–190 ms 的输入节律不是同一数量级 |
| `/scan` 回调 | 均值 1.024 ms，最大 2.046 ms | scan 转换与 guard 处理较短 |
| tick 最大值 | 1.172 ms | 本队控制决策没有百毫秒级阻塞证据 |
| spin_once | 最大 32.350 ms，CPU 总计 11.453 s / 约 89 s | 不支持单次执行器处理阻塞 180 ms 的解释；仍不能分离等待、取样、反序列化与调度 |
| graph 查询 | 总计 0.05891 s，最大 0.240 ms | 不是此次主要耗时 |
| scan 同步 | 683 次 incoming，144 次 late prior，169 次 pending 过期 | 对应 odom 晚到时，scan 已消耗大半有效期 |

末尾同步样本等待 odom 的真实 wall 时间为约 96–176 ms；scan 在本队 receive 入口的 clock-header 差全部不大于零，而 `/odom` 后段出现约 0.18/0.02/0.02 s 的 header 节律及约 0.19/0.01/0.01 s 的 arrival 节律。原始时间保留后，同步器能配对，但有效期仍不足。v17 实测前进 0.07684 m，因 DRIVE 55 s 时限停止，未达到 0.20 m，也没有任务得分证明。

这些证据将重点转向回调入口之前的消息供应/调度；它们不证明 RELIABLE 是根因。替代解释仍包括仿真源的发布节律、公开桥的抽样/转发、DDS 重传或缓存、按主题的执行器等待，以及平台其他进程的 CPU 限额。`clock_minus_header` 使用最近已接收的 10 Hz clock，并非独立同步的传输时延；大部分负值不能解释为消息逆时传输。receive 从 ROS 反序列化后起算，spin_once 包含 receive，不能把两者总耗时相加。

## 唯一实验变量

v17 的适配器在创建输入订阅时，先选 BEST_EFFORT；若查到的公开 publisher 非空且全为 RELIABLE，则升级请求为 RELIABLE。此项只在创建时执行，不随以后 graph 变化重新协商。v17 实际 `/odom` publisher 与请求均为 RELIABLE，depth=1，durability=VOLATILE；`/clock` 已为 BEST_EFFORT。

v18 在原策略之后，仅覆盖 `/odom` 的 requested reliability 为 BEST_EFFORT。可靠发布者允许与 BEST_EFFORT 请求者匹配，因此无需改变公开发布者或任何输出。所有其他主题、depth、durability、时间戳和 payload 处理保留原行为。`/scan`、`/uav/odom`、`/tf`、图像以及裁判输入均不在本轮变更范围。

适配器新增可选 `input_reliability_overrides=None`，默认行为保持原样。覆盖项在创建 ROS 节点前复制为只读映射，仅允许键 `/odom`、值 `BEST_EFFORT`；其他主题、输出键、未知键、类型或值均在 ROS 副作用发生前报 ValueError。调用者修改原映射不会改变冻结配置。

独立入口为 `inspection_mission.ground_odom_qos`，启动脚本为 `scripts/run_ground_odom_qos.sh`。`mode` 仍为 `ground_bounded_approach`，launcher / startup / callback timing 的 revision 为 `v18_odom_best_effort`。计时器复用冻结 v17 类，仅修改 revision。

## 执行与验收

1. 主流程完成全套测试和打包后，在已授权的官方平台运行一次 v18。脚本不启动仿真或裁判，也不声明得分完成。
2. 运动与安全完全保留：90 s 全局、45 s 转向、55 s DRIVE、目标实测 0.20 m、线速 0.03 m/s、路径/原点预算 0.30 m、scan anchor 运动 0.03 m 加停车 0.03 m、传感器 freshness 0.20 s、clock lag 0.12 s、prior skew 0.05 s、末尾五次零速以及原 watchdog。
3. 先核查日志实际请求 `/odom` BEST_EFFORT、公开 publisher 仍可见且收到 `/odom`；所有其他输入 QoS 与 v17 一致。若无匹配/无样本，保持原安全停车，不放宽门槛。
4. 主要比较 odom header/arrival 长间隔、clock-header 分布、`expired_pending / incoming_scans`、late-prior 等待消耗的 scan wall 年龄、PAUSE 占比与实测前进距离。完整 20 cm 后三帧静稳属于动作验收，不等于裁判计分。
5. 抽签/转向角度和平台负载可能不同，不能仅以本局更快或更远认定因果；记录本局抽签、公开桥名称、原始 SHA 和所有指标。若出现明显改善，应在相同变量设置下复跑或恢复 baseline 作对照。

## 本地验证记录

遵循 test-driven-development：新增文件在生产修改前运行，得到 **14 failed / 3 passed**。预期失败点为缺少 `input_reliability_overrides` 参数和独立 v18 模块；三项原始默认策略已通过。实现后执行：

```text
python -m pytest -q tests/test_ground_odom_qos.py tests/test_ground_callback_timing.py tests/test_bounded_ground_probe.py tests/test_ground_bounded_approach_sync.py
116 passed in 2.32s
```

测试在本机缺少 ROS 时，仅替代外部 ROS 边界，实际运行生产适配器构造过程并捕获其 create_subscription / create_publisher 参数；这证明本队请求与默认策略，不能替代官方 DDS 匹配或机器人实测。覆盖非法配置、调用者映射突变、其他主题与输出不变、原 guard/budget、实际 QoS 记录和满 bounded timing 的真实编码小于 64 KiB。全套检查与平台运行由主流程统一执行。
