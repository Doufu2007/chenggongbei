# 地面方向响应标定独立审查

结论：**Approved（仅限独立有限标定练习）**。冻结源码与脚本未发现阻断项。此结论不是平台控制语义确认、物理停距确认、任务完成或官方计分证据。

## 冻结对象

- `src/inspection_mission/inspection_mission/ground_direction_response.py`：`aeecaa1a50b585cfdd13218a94d56283f0ea93f03a71151475512e852bb21946`
- `scripts/run_ground_direction_response.sh`：`2b464c13550fcf24d0e891ab19a49c48d09a28f90ed4b581c47ce4946c83bc10`
- 作者清单：`runs/review/ground_direction_20261112/freeze_manifest.json`
- 实现报告：`docs/ground_direction_response_20261112.md`

旧计分源码保持冻结：single point `902424ca5ba3ec208a0028645d377f4ce92ff77e5ee56205c87fac95a9b349cb`；route `5ab108c4128db8b32c18cee084fce636e8ce80700f18dd37e468006a2a58b945`；three point `6f7a7bb24f9e5269c72bafc05929085d111599a66a135f2dd3782bef5728abc2`。

## 规格审查与修正

初稿正常请求零点为 .035m，至 .04m 观测硬限仅 .005m；允许反馈速度 .15m/s、年龄 .20s 时，这个差不能视作已证实的物理制动余量。主控裁定将正常请求零点收紧为 **.010m**，保留原 .03m 停止预留、.04m 观测硬限与其余时间/累计预算。最终实现与规格一致，并明确不声称已证明物理停距。

预读时指出了非单位四元数会影响 yaw 硬界；最终版本使用局部归一化派生几何，保留原始 payload，并补作者先红后绿回归。旧共享源码没有改动。

## 独立证据

探针：`runs/review/ground_direction_response_20261112/independent_probe.py`

结果：`runs/review/ground_direction_response_20261112/independent_metrics.json`

独立 7 项全通过，没有重复运行作者 203 项关联检查或 4 项遥测检查；已只读核验作者 XML 及 SHA 清单。

| 独立反例或接缝 | 观测结果 |
| --- | --- |
| 原始 callback → 实际 node.tick → XY publisher 接缝 | 在模拟 ROS transport 下实际执行节点代码，402 条发布含 34 条非零 Y，发布与 evidence 逐条一致；三脉冲流程有界结束 |
| 不再调用 tick 后独立 watchdog | 时钟数据过期，独立线程将 .03 X 请求归零并进入 PAUSE |
| 同 stamp、范围仍安全但内容变化的 scan | `direction_scan_duplicate_changed` 锁停 |
| 同 stamp odom 的 Y 速度改为 .151m/s | `direction_odom_duplicate_changed` 锁停 |
| 归零后往返，净位移仅 .004m、累计路径 .012m | `direction_zero_stop_path_limit` 锁停，没有用净位移掩盖累计漂移 |
| q norm=1.0009，真实 yaw 漂移 .03001rad | `direction_pulse_yaw_drift_limit` 锁停 |
| 位姿共同平移、两个 q 分别缩放 .9991/1.0009 | `response_from_odom` 位移、yaw、bearing 等结果在 1e-12 绝对容差内不变 |

适配器接缝探针使用确定性合成反馈和模拟 DDS/ROS 类型；其 fake clock 循环停用并行 watchdog 以避免调度混淆，另用独立线程探针验证 watchdog。这里的发布指实际执行被审代码的 publish 接缝，**不是向官方平台机器人发送命令**。

## 适用限制

允许的下一步是按上述 SHA 打包单独练习，以公开传感器测量输入输出。实验只发布 `/cmd_vel`，不发布飞行、识别、灯、到点、裁判或完成事件。平台上的实际方向响应、制动距离和是否具备足够有效样本，须由新练习原始反馈裁定；不足时不能扩大运动预算或把合成反馈结果写成官方得分。

脚本只启动本队 Python 源码，不启动仿真或裁判。core/gzip 分片延续 64KiB 接收合同。STOP 吸收、PAUSE 不清脉冲/总预算、请求零与实际停稳分别记录。
