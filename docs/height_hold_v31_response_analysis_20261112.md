# v31 平台有限高度闭环实证分析

本局没有取得高度保持、下降闭环或完整局部协议的成功证据。程序已实际启动并接收传感器；上升阶段遇到一次水平 scan 到达间隔超过固定 0.20s 门，依合同永久归零，尚未到达 +0.10m，更没有进入 HOLD。

平台 ID `f21b350d-dcba-4121-ae93-9781c554471c`。根代理随后取得官网终态：**#85465159 / run_20261112_081239_56ea65d7 / 未通过 / 33.8s / 未提取成绩**，快照保存于 `runs/evidence/20261112/height_v31_official_page_final.json`；回传中的 `FINISHED` 和 launcher exit0 只表示用户程序结束，不等于官方测评成功。此有限辨识程序没有发送巡检完成或计分声明。

## 原始数据与完整性

`runs/evidence/20261112/callback_height_v31.jsonl` 为 136436 字节、8 个事件，SHA256 `2e0a266b4ab9e5e985bcc05f8cb0d6d7a738aa773d44ce2a9e48d9b4c4026baf`。严格按 `bounded_height_hold_v1 / bounded_height_hold_*` 解码，3 片 gzip 完整，压缩/解压长度、双 SHA、会话和 core 共有字段均一致。

- 压缩体 97890 字节，SHA256 `e1ff00d14305a732b872903a51028dbb4f3cf0400f902b784b75ebaa2e539ecb`
- 原始 JSON 608389 字节，SHA256 `c94604be273945f94e97f44b09b835a31daecb12bf1b2634b033e019724a71dd`
- 221 帧 odom、95 条命令、35 帧水平 scan、87 帧下视 scan、441 条 TF；各保留队列 dropped 均为 0。
- launcher exit0；两个发布器无异常；地面全零95次、UAV正Z24次、UAV零71次。

这些哈希证明回传内部字节一致，来源仍为 `unverified_callback`，`source_authenticated_independently=false`；没有据此独立认证平台来源或裁判分数。

## 中止发生的位置

所有下表时间均为用户进程经过的墙钟秒，source stamp 另列，不能混作同一种时钟。

| 事件 | wall elapsed | source stamp | 说明 |
| --- | ---: | ---: | --- |
| ASCEND 开始 | 1.164480726 | odom 3.94 | 实际发布 +0.05 |
| 中止前最后水平 scan | 2.231525722 | 5.004 | min range 0.649972439m |
| 发现 scan 超龄并转 ZERO_RETURN | 2.436796573 | odom 5.14 | 上一 scan 墙钟龄 0.205270851s |
| 首条 ZERO_RETURN 零请求返回 | 2.437171297 | odom 5.14 | measured z 0.109898716m，vz +0.0461m/s |
| 下一水平 scan 到达 | 2.439159980 | 5.154 | 比中止晚 0.002363407s；min range 仍0.649972439m |

这两帧 scan 的 source 间隔为 0.150s，arrival 间隔为 **0.207634258s**。因此这次直接触发的是一次到达超龄，不能写成 scan 永久消失、碰撞或净空不足。其后 scan 持续恢复。单凭这些回传数据不能进一步区分传输、DDS、消息转换或执行器调度分别贡献了多少延迟。

永久中止后的下一控制检查仍处在相同超龄窗口，因此 OBSERVE 转 STOP，理由 `return_sensor_lost:horizontal_scan_stale`，约两秒后进入本地 FINISHED。这说明用户程序的失败条件清楚发生在代码内部，和“未见用户日志就推断用户代码没运行”不同。

## 实测结果与逐帧核验

原始 odom 重算实升峰值 **0.055820008m**，累计路径 **0.111872266m**，最大横漂 **0.000186334m**。最末 odom 已回到 z≈0.055000216m，但这不等于完整返回验收通过。

- 没有进入 HOLD 或 DESCEND，原始轨迹中不存在可供验证的 HOLD 三秒窗口。`hold_verified=false`、`descent_verified=false`、`protocol_complete=false`、`support_return_verified=false` 与独立分析一致。
- 全部非零均为 ASCEND 的 +0.05，没有实际 trim / 高度反馈命令；因此本局不能证实 `.02695` 已悬停，也不能评价反馈增益优劣。
- 根据命令的原始 odom stamp逐条核对，没有测量字段不一致或固定公式错误，没有低高度阶段非法非零，也没有超过永久25s或HOLD8s的非零。
- 归零后、首个 FINISHED 之前的末段48帧满足支撑高度±5mm、速度≤0.005m/s、角速≤0.01rad/s、位置半径≤0.002m；其 source 跨度只有 **0.94s**、arrival 跨度 **0.956344727s**。不能将其写成独立测得连续1s静稳，更不能把支撑附近静止写成触地传感器证据。

本局不放宽 .20s 门，不恢复已中止非零，也不根据终态旗标宣称协议成功。后续如处理调度抖动，应以独立实现与新证据另行验收。

## 可复核产物

`runs/analysis/20261112_height_v31/` 保存逐字节 `evidence.json.gz / evidence.json`、`validation.json`、原始轨迹和命令 CSV、独立 `metrics.json` 与 `timing_landmarks.json`。执行：

`python runs/analysis/20261112_height_v31/analyze.py`

分析器自身用实际 factory 的合成正例通过双SHA恢复和完整局部协议判定；保持成功 flags 而把真实HOLD速度改为越界的反例被独立拒绝。旧 section与缺失分片也被拒绝。校验位于 `runs/review/height_hold_independent_20261112/validate_analyzer.py` 和 `analyzer_validation.json`。这些校验只验证分析逻辑，不新增平台成绩。
