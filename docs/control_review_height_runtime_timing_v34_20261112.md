# v34 高度时序包装独立审查

**结论：当前冻结诊断包装通过软件审查，可供原 v31 合同下的有限时序观测；未发现新增控制行为或阻断。** 插桩有开销，本结论不证明平台超龄原因、真实高度控制成功或完整任务成绩。

- 模块 SHA256：`cb2c0b4dadb30b8b2ff1700e19e669ff7759bbf69dbaf58a71058cd5f8dee6be`
- 启动器 SHA256：`29201e1323e2cb9e1d6b7b9763701617c11a5c6bb0f4b8368bb6ce6e15e010d1`
- 独审时间：`2026-11-12T00:41:43.745939+00:00`。`runs/review/height_runtime_timing_20261112/manifest.json` 中 15 个文件实核全部匹配；5 个 v31/父层/telemetry 冻结文件保持字节一致，模块在独审前后相同。

独立命令 `python runs/review/height_runtime_timing_independent_20261112/independent_probe.py` 返回 0，**8/8 组通过**；同目录 `independent_metrics.json` 保存详细结果和哈希。启动器 Git Bash `bash -n` 返回 0。未改生产代码、未重复作者 54 项、未上传。

| 独立检查 | 结果 |
| --- | --- |
| 实际 factory 注册 callback 与原 spin | 7 类输入入口/退出各1次，spin_once 7次，外层 tick 与 control_tick 各1次；入口/退出 wall、CPU 非负，topic/source 元数据存在 |
| 同输入无插桩基线对比 | 正常全流程 538 次实际发布、commands、phase transitions、终态精确相同；最终 DDS 查询延迟 `.21s` 和抛异常两场景也逐项相同，且只发布零 |
| DDS 时序 | 最终查询被标记 `dds:publish_boundary`；注入 `.21s` wall 与 `.003s` CPU 差均准确记录，原超龄/异常处理保持有效 |
| 异常原样传播 | 包装前 receive、tick、_publish、spin_once 抛出的同一个异常对象仍向上传播，finally 完成时序记录；原 publisher 查询与 publish 的异常对象也未被包装吞掉 |
| 元数据隔离 | `.03s` 元数据开销纳入入口 span；元数据自身异常只计诊断错误并继续原 receive；header reader 不访问或复制附带的大 payload |
| 容量、汇总与证据 | 2100 个完成 span 保留最近2048、丢弃52；最大慢 span 保留32、丢弃2068；count/total/max/首末入口准确，wall/CPU 合计约44.1/2.1s。原 gzip 双长度/双 SHA 恢复通过，runtime_timing 与 snapshot 一致，mode/schema/任务未完成语义保留 |

Publisher 兼容性已单独核查：严格夹具要求 publish/get_subscription_count 的 `self` 必须是原对象，且 Node 实体登记与销毁只接受原对象；当前包装均通过。源码只替换应用自身 `publisher_by_topic` 查询表，原 Node 登记未改；委托时调用原对象的 bound method。

**该代理本身不是 `rclpy.Publisher` 实例。** 若未来把代理直接传给要求具体 Publisher 身份的 API，例如 `destroy_publisher`，不能保证兼容。当前冻结路径没有该调用，也没有把代理交回 rclpy；本机未安装 rclpy，严格夹具不等于真实 ROS 类型集成实测。

只读检查确认：包装没有新增 sleep/wait、同步回调网络写入或锁；原锁及调用次序由原方法维持，源龄 `.20s`、运动25s/总45s、公式、QoS、watchdog和发布前安全检查未变。只读 header/clock 标量；copy/deepcopy 仅用于有界诊断记录的快照，没有复制 ranges、图像或点云。生产默认计时为 `time.monotonic` / `time.thread_time`，线程局部 publish 上下文避免与 watchdog 串线；watchdog零路径没有 DDS 查询。

探针初次误用不存在的 `clock_time` 字段，已改为真实 `clock[0]`，只修改独审夹具。上述基线等价证据依赖相同合成输入和虚拟控制时钟；真实插桩开销仍可能改变到达时序。嵌套 span 重叠不能相加，source stamp 不能当作源端墙钟发布时间，也不能由这些计时反推出主机调度、DDS take/反序列化或 guard 锁等待原因。SHA 变化后本结论不自动沿用。
