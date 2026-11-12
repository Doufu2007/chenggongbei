# v34 原有限高度实验的有界时序观测

根代理已明确授权本轮自主实现此诊断适配层并安排后续独审；不上传。目的仅定位v31中64ms共同记录空档，不增加控制能力。

新增 `height_runtime_timing.py`、`run_height_runtime_timing.sh` 与专项测试；原 `bounded_height_hold.py`、`bounded_vertical_response.py`、`bounded_ground_probe.py` 冻结。直接调用原 `create_height_node` 后包装实际receive/tick/publish与UAV publisher查询，保留原guard、控制时钟、.20s源龄、公式、25s/45s预算、watchdog与全部发布前检查及锁。

记录器仅读原始消息header/clock元数据，不复制ranges、图像或点云。记录monotonic入口/退出、当前线程CPU差、topic、source stamp、入口phase/state及span标签；覆盖全部7个原输入，包括clock和地面odom。原spin的observer钩子记录spin_once/tick，适配tick另标control_tick；重叠span不相加。

UAV publisher透明代理仅包装get_subscription_count并原样调用；每次查询区分 `publish_boundary` 与 `control_tick`，以识别最后DDS检查耗时。publish原方法及command_lock/guard锁不移动，不插入sleep/wait，不改变返回值或异常传播。所有生产时间读数默认 `time.monotonic`，CPU用 `time.thread_time`；测试可注入单调函数。

每个固定label有count/total/max/首末入口汇总，完整trace最多2048条近期完成span，溢出明确计数；另保留最大32个慢span，便于停止后仍保留异常。证据进入原完整gzip的可选 `runtime_timing` 对象，保留原schema、模式和任务未完成语义，加revision `v34_timing`。无同步网络写入回调；最终仍由原report_final输出。

先写测试证明实际注册callback和真实spin路径必须被覆盖，观察缺模块的RED，再实现。补原样控制输出/慢DDS后归零、异常路径计时、容量及gzip恢复、启动退出码专项。插桩自身有开销，不能宣称完全零扰动，不能用时序证明得分或把source clock当墙钟发布时刻。
