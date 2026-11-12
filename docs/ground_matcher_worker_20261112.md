# 独立生产 GroundMatcherWorker

新增同包 `ground_matcher_worker.py` 和 `tests/test_ground_matcher_worker.py`。它把冻结GroundPlaneMatcher放进spawn子进程，提供单执行中任务+单槽最新待处理帧+单槽结果，主循环不等待SIFT。没有ROS、控制、网络、裁判发布或类别上报接口；未修改matcher、vision_node、共享控制或profile。

## 接口

```python
worker = GroundMatcherWorker([TemplateSpec(path, class_id, label, provenance, expected_sha256), ...])
worker.set_context(session_id, frozen_first_ID, stationary_epoch)
worker.submit(job)
results = worker.poll()  # 非阻塞，零或一条结果
worker.cancel_context()  # 立即失效并清待处理；不等待正在计算的旧任务
status = worker.close(timeout_s=.5)  # 有界宽限退出，必要时terminate
```

所有调用由单一主循环owner执行；结果反序列化由独立daemon reader承载，不占主循环的SIFT计算路径或外部控制锁。子进程使用同包import，模板仅显式TemplateSpec，不假定仓库ROOT，不自动下载/打包外部参考。

job必需字段：`job_seq`全生命周期严格递增整数，`session_id`、`frozen_first_ID`、`stationary_epoch`、`source_jpeg`原bytes、`source_sha256`、`image_stamp_s`、`topic`、`frame_id`、`pose`原前序stamp/position/quaternion，显式3×3`K`，原`camera_info`（width/height/k/d/header.stamp/header.frame_id及其它原字段）。元数据有限JSON≤64KB，JPEG≤2MB且SOF为800×800；标准库解析JPEG header，不需Pillow。K须与CameraInfo一致且为有效针孔模型，畸变全0；原pose不能来自曝光之后。

结果回带去JPEG后的原job及hash/内部context generation、全部matcher候选/拒绝与模板path/hash/label/provenance、worker PID、worker起止UTC/monotonic、load/decode/match耗时。错误返回绑定job的`error`；初始化失败/进程退出可诊断并禁止继续提交，不自动重启或放宽门。首次poll分发前失败仍保留pending原图的SHA/stamp/首ID；`job_was_dispatched`明确是否已分发。初始化失败或尚未分发时`job_was_executed=false`；已分发后进程异常退出时为`null`，父进程无法判断任务是否曾部分执行。

`matches_current_context`只说明当局/首ID/静止epoch与内部generation相符，**不是freshness**。`fresh_exposure_validated`、`eligible_for_recognition`、`official_acceptance`始终false。context更换、取消、关闭会立即撤销缓存适用标志；epoch即使回退到旧元组也不能让旧generation结果复活。控制方仍必须根据当前clock/odom/scan、曝光以来无运动命令和位姿漂移、原图时间、区域关联和多帧一致性独立决定。

`cancel_context`不强杀SIFT，只令结果失效，后续最多保留一张最新待处理。`close(timeout_s)`先失效，半预算宽限然后terminate，最多等待给定预算附近的OS调度开销；返回process_alive，不能把未退出进程当退出成功。不得在控制锁内关闭。这里不证明OS调度硬实时或进程RAM峰值上界。

## TDD与真实实测

生产测试先观察模块缺失RED再实现，另先复现意外进程退出不回报错误的RED再修复。独立审查再发现首次poll前失败丢失pending绑定，两个反例先RED再修复；已分发后异常退出不能证明未执行，也先以断言观察RED再收紧诊断。18 worker专项覆盖：有界最新替换/输入快照、context往返与cancel、原pose/K/hash/畸变/JPEG门、合成模板spawn真实匹配、初始化时有界关闭与进程异常诊断。与32 matcher专项合计50 passed（2.79s）；`py_compile`通过。没有在测试中重复外部12模板秒级推理。

最终源码的独立6项反例全部通过（0.62s）：pending两种错误路径保留源绑定、active异常退出执行状态未知、epoch回切不能复活、坏job不耗序列、NaN原K拒绝。独立XML为`runs/review/20261112_matcher_worker_production/independent_final_v2.xml`；实测心跳CSV与12模板结果也经独立逐项复算，见同目录`benchmark_independent_arithmetic.json`。

真实性能仅在独立离线脚本执行1次v22原图，不作三次重复。该实测对应失败诊断修复前的源码SHA `2362025e5a3f013cc6bb124c190e9b8cbaa303675c48ac8347aa5cbdf6697ed1`，最终源码未重复实测；修复只涉及失败路径绑定与执行状态：

| 本机单次实测 | 值 |
|---|---:|
| 20Hz owner周期 / 时长 | 226 / 11.25053s |
| 有效频率 / 跳过周期 | 19.99906Hz / 0 |
| 最大 / P99周期间隔 | 53.42 / 51.10ms |
| owner最大工作时间 | 17.79ms |
| 子进程模板冷加载 / match | 8143.61 / 2623.49ms |
| 空闲关闭 / 子进程存活 | 53.89ms / false |

全部12模板good/inlier/接纳与同步v22冻结报告逐项相同。原图只有一帧，现存候选不是新曝光或在线稳定识别。该本机结果没有ROS回调、仿真器CPU竞争、平台部署或硬实时上界保证。

证据：`runs/analysis/20261112_ground_matcher_worker_dev/production_specialized_final.xml`、`production_execution_unknown_red.xml`、`production_benchmark_report.json`、`production_heartbeat_ticks.csv`、`benchmark_production.py`。早期原型证据继续分文件保留。最终生产源码SHA `d50ae59a24f11d80ca566193cf63e254eb5c55846eff278def24905b96735ec8`，测试SHA `72315136f31d5c09617ec0f43f2c15746a045095113a146861fba21b6725f627`；冻结matcher仍为`1b9c9b3cd658f5192307161dbbe21954968e9680c98424250d2d0836e1dc710d`。
