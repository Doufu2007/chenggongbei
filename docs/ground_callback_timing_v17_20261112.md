# v17 回调与执行器计时诊断

本变体只定位 v16b 消息过期的来源。动作、同步、输入 QoS、90 秒全局时限、freshness 与安全门槛均沿用 v16；动作范围仍为转向本局首个地面点、实测前进 20 cm、停止并验证静稳。不发送识别、计分或全任务完成消息。`mode` 保持 `ground_bounded_approach`，`revision` 为 `v17_timing`。

## 测量边界

- `receive:<topic>` 从 Python 订阅回调入口开始，记录原始消息 header 与最近已接收 `/clock` 的差值、相邻回调入口间隔、相邻 header 差值，以及本队适配器处理的 wall / 当前线程 CPU 耗时。入口处消息已由 ROS 反序列化；此项不含此前的取样或反序列化成本。
- `spin_once` 包围实际 `rclpy.spin_once`，包含执行器等待、取样、反序列化及所执行的回调成本。因此它与 receive 存在包含关系，二者总耗时不可直接相加。
- `tick` 包围一次原有控制决策。原有命令节奏仍为最多 20 Hz，慢迭代不追赶补发。
- 原生图像只读取 header，不复制或处理像素。所有 guard 使用的 stamp、wall clock 和消息内容均不替换。
- `input_qos` 记录实际创建订阅时请求的 depth / reliability / durability，以及创建时已查询到的至多四个公开 publisher QoS；不添加 graph 查询，不改变 QoS。它描述配置，不能单独证明延迟来源。
- guard 锁等待没有单独测量。receive wall 时间含其中发生的锁等待；未给锁加包装或额外获取锁。
- 入场 clock-header 年龄由消息源、桥、DDS、执行器调度和最近 clock 样本共同影响，不能由本项单独分离。

指标使用固定桶及 count / total / min / max；仅保留最多 12 个最大慢事件，未知 topic 合并到一个固定桶。专项测试用全部输入、最大 QoS 样本和满慢事件列表，经真实 `DiagnosticReporter._encode` 验证 payload 小于 64 KiB。

## 下一局的判断方式

1. 若 receive wall / CPU 明显变大，优先查对应 topic 在本队适配器中的转换与 guard 处理；wall 大而 CPU 小还可能含锁等待或线程被抢占，不能直接解释成网络延迟。
2. 若 receive 很小，而 spin_once wall / CPU 明显变大，继续查取样、反序列化、执行器和平台 CPU 配额；spin_once 的 wall 大也可能只是正常等待。
3. 若 receive 入场时 header 已旧，而本队处理很小，问题发生于回调入口以前。下一独立变体才能改变一个输入传输变量做对照，不能因此放宽 freshness、修改 header 或预设桥故障。
4. 结合原有 PAUSE 原因、clock 进度、消息计数和时间桶解读；计时成功不等于任务完成或裁判得分。

## 已有本地基准的限制

`tools/profile_guard_callback_costs.py` 的结果保存在 `runs/analysis/20261112_approach_v16b/local_guard_costs.json`。这是本机合成静止 DRIVE、history 满 2048 条的纯 guard 成本测量，不是官方平台运行证据。该次 odom 中位约 0.106 ms、p99 约 0.374 ms；scan 中位约 0.439 ms。它不包含 ROS 取样/反序列化、`message_to_ordereddict`、DDS、平台 CPU 配额或真实跨线程锁等待，不能外推为平台回调性能。Windows 当前线程 CPU 计时出现 0 / 15.625 ms 量化，原数保留，不将 CPU=0 解释为无开销。
