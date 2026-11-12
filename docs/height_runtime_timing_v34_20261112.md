# v34：有限高度实验的时序观测适配层

本轮只为已冻结的v31有限高度实验增加诊断，定位上一次真实运行约64.64ms的共同记录空档。没有平台上传或新真实运行；不能据此解释成已找到平台故障、已修复真实超龄，或已完成巡检任务。

## 实现边界

新增 `height_runtime_timing.py` 和 `run_height_runtime_timing.sh`。适配层直接调用原 `create_height_node`，原height、vertical、ground、telemetry和height启动器均保持字节一致。保留原25s运动预算、45s总预算、0.20s传感器年龄门、控制公式、QoS、原command_lock/guard锁与watchdog。启动器只运行本队模块，不启动仿真或裁判。

实际订阅注册的lambda动态调用包装后的receive，7类输入均被计数：`/clock`、`/uav/odom`、`/odom`、`/scan`、`/uav/laser/scan`、`/tf`、`/tf_static`。receive先捕获monotonic入口，再只读clock/header标量，记录源stamp、state/phase、退出和当前线程CPU差；不读取图像、点云或复制scan数组。元数据观测异常计数后仍调用原receive。

原 `spin_control_loop` 的observer钩子记录主循环 `spin_once` 与外层 `tick`；原node.tick另包装为 `control_tick`。UAV publisher代理原样转发publish和其他属性，仅计时 `get_subscription_count`，用线程局部上下文区分 `dds:control_tick` 与 `dds:publish_boundary`，不改变返回值或原异常。最后五次清理spin没有额外spin标签，但其中的receive仍覆盖。

每个固定label保留完成次数、wall/CPU总量和最大值、首末入口；最近完成span上限2048、最大的慢span上限32（wall≥20ms），均有丢弃计数。`runtime_timing`只加入原完整gzip证据，不改变 `bounded_height_hold_v1` schema、mode、任务未完成或官方分数未知的语义。无回调内同步网络写入。

## 检查结果

先写真实factory callbacks+原spin测试，观察缺模块RED；本轮保存的 `red.txt` 记录入口捕获晚于metadata及启动器不存在的4个失败。随后新增元数据故障不阻断真实receive测试，也观察到原故障。修正后10个专项全部通过：

- 实际注册的clock/地面odom/UAV odom/scan回调和原spin/tick均命中记录器。
- metadata耗时计入receive，入口不晚于metadata；metadata异常不阻断原clock更新。
- 最后DDS查询延迟0.21s或抛异常时，原发布边界仍只发零；查询wall/线程CPU被记录。
- 原factory与适配factory在相同合成输入下的全部输出、command和phase transitions精确相等。
- 7类输入计数、trace/slow容量、溢出计数和真实DiagnosticReporter编码、gzip恢复一致。
- 启动器保留0、子进程17、ROS setup 29的真实退出码。

必要关联检查只运行原height 42项、原spin节拍1项、原watchdog1项，44项通过。累计本轮54项通过；未展开整仓回归。RED、两份GREEN输出及SHA清单见 `runs/review/height_runtime_timing_20261112/`。

## 解释限制与后续

插桩有开销，source stamp是仿真源时钟而非源端wall发布时间；嵌套span重叠，不能将receive、spin与tick相加。当前未独立测量DDS就绪、take/反序列化、主机CPU调度或配额、guard锁等待。原同步查询阻塞期间仍不能保证硬实时归零，只能在其返回后由原发布边界重检。

trace是有界近期记录，旧span可能被覆盖；slow记录只保留最大的32项，不能保证保留每一个异常。完整计数和丢弃数用于说明证据覆盖范围。本地合成transport不验证ROS/Gazebo动力学，也不验证官方得分或完整任务。下一步由根代理安排独立审查，审查通过后才能考虑同合同的有限平台观测；本子任务不上传。
