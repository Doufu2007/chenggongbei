# v31 单次 scan 超龄的只读根因调查

现有证据可以确认“单次到达超龄导致合同中止”，还不能定位到 `message_to_ordereddict`、某个 guard、DDS 查询或宿主调度中的单一环节。没有修改控制器、0.20s 门或上传复测，也没有重跑大测试。

本局官网终态已由根代理保存：`runs/evidence/20261112/height_v31_official_page_final.json`，测评号 **#85465159**，runId **`run_20261112_081239_56ea65d7`**，未通过，显示33.8s，未提取成绩。它与本地有限实验中止分别记录，不把页面展示0分解释成已提取正式任务得分。

## 从本局 raw 能确认的事实

水平 scan 的 source stamp 从5.004到5.154，差0.150s；Python `receive` 入口墙钟从2.231525722到2.439159980，差0.207634258s。中间TF回调入口2.436796573触发超龄，此时上一scan年龄0.205270851s。

异常附近其他已保留序列也出现本局最大的间隔：

| 记录 | 最大相邻入口/记录间隔 |
| --- | ---: |
| UAV odom | 66.311ms |
| TF | 83.724ms |
| 下视3束scan | 97.996ms |
| 命令发布返回记录 | 112.478ms |
| 水平360束scan | 207.634ms |

把这些序列按记录时间合并后，最大共同空档是 **64.642876ms**：前一条为wall2.372153697的UAV odom入口（source5.14），后一条为wall2.436796573的TF入口（source5.14）。其后odom、TF、两类scan又密集进入。这个现象更适合优先排查一次非scan专属的服务迟滞，而不是立即断言水平scan自身计算慢。

限制：本局没有保存 `/clock`、地面 `/odom` 的每次入口，也没有 callback 退出、CPU或span信息。共同空档只表示“这些保留序列没有记录”，不等于期间主线程完全没运行。source clock也不能代替源端发布的真实墙钟时间；模拟推进减慢、源端晚发布和下游晚处理仍不可分。

## 实际上传源码的检查

已从 `dist/inspection_height_v31_src.zip` 只读核对四个执行相关模块，与当前源码字节一致；具体SHA保存在 `runs/analysis/20261112_height_v31/timing_investigation.json`。因此下列检查针对本次实际上传代码。

- `bounded_vertical_response.receive` 在转换前捕获wall，然后同步执行 `message_to_ordereddict` 和guard。raw中的wall为入口时刻，不是转换或处理完成时间，无法据此测出这两者成本。
- `spin_control_loop` 顺序执行一次 `rclpy.spin_once`，到命令时隙再 `tick`。timeout最多20ms，并非每个50ms命令周期只处理一个输入，也没有固定50ms睡眠。但任何慢receive、spin内部或tick都会延迟其他待处理输入。
- `tick` 的图查询按0.5s节流；匹配数查询和两路publish仍同步调用。它们可能阻塞，但本局没有查询起止时刻，不能将64.6ms归给其中任一调用。独立watchdog也不能保证阻塞期间的硬实时归零。
- 水平scan为360 ranges+360 intensities；其转换后还有两次 `deepcopy` 和 `_append` 内的 `_safe` 递归复制。TF每条只有一个transform，下视scan只有3束。它们均有成本，但当前处理是复制本条消息，不是每回调复制整段历史；队列本局没有溢出，所以 `pop(1)` 不是本次成本证据。
- 主循环没有图像或点云处理。大JSON序列化、gzip、分片同步回传在spin结束后进行，不能解释wall2.436s时的中止。只有启动小事件进入异步上报；没有证据证明它导致争用。

## 旧性能证据的适用范围

`tools/profile_guard_callback_costs.py` 测的是本机v16纯guard和合成消息，不包含ROS取样/反序列化、`message_to_ordereddict`、DDS、平台CPU配额或真实跨线程锁等待；也不是v31高度guard。其scan中位0.439ms、最大1.884ms，不能拿来排除v31的60ms停顿。

旧v17有真正的平台计时：水平scan receive最大2.046ms，tick最大1.172ms，spin_once最大32.350ms。它说明该次运行中观测到的scan适配器处理并没有60ms级成本，但v17的适配层、输入负载和运行条件不同，不能代替本局测量。v17 receive时间还包含guard处理和其间等待；不能从它拆出转换成本。spin与receive是包含关系，不能把两项总耗时相加。

## 最小可证伪假设及所缺证据

优先假设 **H1：本次约60ms额外迟滞主要发生于Python回调处理入口之前的spin/执行调度阶段，而非某次scan转换或guard的持续CPU计算**。它能解释多种保留序列同窗跳大、正常150ms source周期被拉成207.6ms入口间隔；但当前只属待检验假设。

最小下一证据应复用已有v17计时方法，在相同协议、不改控制时钟/门/QoS的条件下保存一小段有界时间线：所有输入的receive入口/退出wall与当前线程CPU、spin_once入口/退出及请求timeout、tick入口/退出，并保留原始header和最近clock。必须包含本局未保留的clock与地面odom入口。不要只保留最终平均数，应能将一次超龄附近的span对齐。

可证伪判据：若超龄窗口由一个receive或tick的长span覆盖，H1被否定，应再把那个span最小拆成转换/guard或查询/publish；若receive和tick都短、而spin wall长且CPU很小，才支持前入口等待/调度假设。即使如此，也还需源端发布墙钟、DDS就绪/取样或宿主调度证据，才能区分源仿真减慢、DDS等待、执行器选择和CPU抢占。当前不据此声称已定位平台故障或应改多线程。

复核脚本为 `runs/analysis/20261112_height_v31/investigate_timing.py`，只读原始恢复数据、旧v17计时和已上传ZIP，生成 `timing_investigation.json`，未驱动机器人。
