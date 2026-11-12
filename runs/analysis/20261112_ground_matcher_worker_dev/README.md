# 图像worker独立原型计划

范围：只在本目录新增纯感知原型与实测证据，不改src/tests/冻结算法，不接ROS锁、控制或裁判。使用spawn子进程运行原GroundPlaneMatcher，避免SIFT计算占用主进程GIL。

接口：`LatestFrameWorker(template_manifest).set_context(session_id, first_ID, stationary_epoch)`；`submit(job)`接收绑定原JPEG字节/SHA、原图stamp、原pose、实际K与递增job_seq的工作；`poll()`非阻塞收取完整诊断与过期标志。最多1个执行中Future、1个最新待处理输入、1个最新结果；覆盖计数显式记录，绝不为每帧追加Future。旧context结果保留诊断但标记不可用于当前context。

TDD：先测试待处理帧替换/过期context/输入原始元数据保持与拒绝不良stamp/序号，再用真实v20单帧和冻结12模板完成进程实测。主循环按20Hz调度，记录每tick时间/落后/工作耗时；冷加载期间提交新帧只保留最新。完整匹配耗时、模板hash、实际执行/覆盖job、心跳间隔均保存。相同原图的重复任务仅用于worker机制/耗时，不当作多曝光识别证据。

结果一律`official_acceptance=false`；worker不能决定进入区域、发表检测、宣称到点或收到官方接受。调用方必须另验当局/首ID/静止epoch、原曝光pose/K、当前任务区域关联和多独立帧稳定性。

## 最终实测与审查

`benchmark_report_final.json`绑定当前worker/benchmark源码SHA，旧`benchmark_report.json`仅保留为早期版本证据。最终用v20同一原JPEG：冷加载7.898s，两次真实匹配2.955s/2.688s；主进程14.407s内289tick，19.99995Hz，最大间隔51.56ms、P99 50.93ms、0跳过调度。owner每tick工作最大13.86ms、P99 1.29ms。主PID23932与worker PID29024不同，真实计算在spawn子进程中。

4次submit仅执行job1/4，job2/3在单槽pending内被覆盖。job1因epoch变化标记context不适用，job4 context适用，但两者`fresh_exposure_validated=false`与`eligible_for_recognition=false`。当前context相同不等于图像新鲜，既有缓存也会在epoch改变时立即失效。调用方须在任一运动/位移/转角/传感器epoch重置时调用set_context，不能靠耗时放宽原.20s传感器freshness。

最终13项本地原型测试通过，独立8项反例通过。审查期间修复旧缓存epoch标记未撤销、future pose、K/CameraInfo冲突与非零畸变；JPEG header在子进程实际解码前必须为800×800，压缩payload≤2MB、metadata≤64KB。该限制保证输入尺寸/槽位有限，**并未测量或证明进程RAM上界**。固定matcher仍显式读取全部12模板。

`first_ID`是context冻结字段；正式接线可将其重命名`frozen_first_ID`，不能每帧从可变task表补写。当前返回完整原job（topic/frame/stamp/hash/prior pose/K/CameraInfo含原header），以及worker PID、模板path/hash/label/provenance、全部候选与拒绝、匹配耗时。正式接口仍需补worker UTC起止/取消超时/崩溃重启与ROS采集合同，本原型没有这些生产保证。

`close(wait=True)`只在owner循环停止后的shutdown执行，实测闲置后61ms；禁止在控制tick里等待。这里没有接ROS/控制锁或真实传感器回调，Windows调度与单次负载记录不构成硬实时上界。全部改动限定本目录，生产src/tests及冻结识别算法未改。
