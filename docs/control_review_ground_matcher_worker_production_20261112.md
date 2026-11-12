# 生产图像 worker：独立复核

2026-11-12。审查对象为同包 `ground_matcher_worker.py`，未接 ROS/运动/裁判；未改生产源码、生产测试或实现代理原 benchmark。本代理反例及复算保存于 `runs/review/20261112_matcher_worker_production/`。

## 实际发现与复核

初次独立反例发现：已 `submit()` 的图还在 pending，而子进程在首次 `poll()` 前已退出或初始化失败时，错误分支仅用 `_active or {}`，随后清 pending，导致错误中的 job 变成空字典、丢失图 SHA/曝光 stamp/首 ID。确定性控制流反例实际 **2 failed / 3 passed**，保留 `independent_initial_behavior.xml`。更早 `independent_initial.xml` 是审查 fixture 初次缺本包 sys.path 引起的收集错误，已单独修正，未算作生产缺陷。

实现代理仅修两个失败路径，优先取 active、否则取 pending，补 `job_was_dispatched` 和执行状态。对子进程意外退出，active 的执行状态必须为 `null`：父进程不知道任务是否已被取走或计算到何处；pending 未 dispatch 才是 `false`。初始化错误发生在进入任务循环前，可明确 `false`。

最终 worker SHA256 `d50ae59a24f11d80ca566193cf63e254eb5c55846eff278def24905b96735ec8`。独立最终 **6 passed / 0.62s**，保留 `independent_final_v2.xml`，覆盖两种 pending 失败绑定、active 意外退出的执行未知、旧 epoch 回切不复活、非法 job 不消费序列或覆盖 pending、原 CameraInfo K 为 NaN 时拒绝。实现代理自身 18 worker + 32 matcher 专项通过另有 `production_specialized_final.xml`；独立 6 项未冒充整套生产或平台验证。

## 结构和有界的含义

采用显式 spawn `Process`，输入队列、输出队列和结果 reader 的线程队列各 maxsize=1。owner 保持一个 active、一个可覆盖 pending、一个 latest result；一次只 dispatch 一个 job，读取结果后才允许下一个。SIFT 在子进程执行，结果反序列化由 daemon reader 承担；owner 仍有 SHA/JSON/深拷贝/调度等工作，不承诺不阻塞或硬实时。

context generation 可在 session/首 ID/epoch 值回切时继续识别旧任务为过期；关闭或取消也失效已交付 latest result。所有结果的 `fresh_exposure_validated`、`eligible_for_recognition`、`official_acceptance` 仍为 false。context 相同只证明绑定相同，不能把计算完成时间当新曝光，不能据此允许上报。

输入限制 JPEG 2MiB、metadata 64KiB，stdlib JPEG SOF 检查仅接受原 800×800，再在子进程 decode。原 JPEG SHA、topic/frame/stamp、先前 pose、K 和原 CameraInfo 保留并校验；这些检查不认证外部调用方数据真伪，不证明原图新鲜或类别/点号正确。显式模板清单和槽位上界不是任意模板集的固定 RAM 上界。

`close(timeout_s)` 先失效 context，最多使用半预算等待优雅退出，仍存活则 terminate，并使用剩余预算 join；不等待 queue feeder/reader。返回 `process_alive` 必须保留，timeout=0 等情形不可只依据 `closed=true` 宣称进程已退出。该函数是退出阶段操作，不应塞进运动 tick。没有重启、ROS 或裁判 publisher。

## 保留的真实单图 benchmark 复算

`production_benchmark_report.json` 和 `production_heartbeat_ticks.csv` **对应修前 worker SHA `2362025e5a3f013cc6bb124c190e9b8cbaa303675c48ac8347aa5cbdf6697ed1`**。错误诊断修补没有改正常匹配路径；为保持精确版本身份，没有重标旧测时或再跑一次真实 SIFT。

独立 CSV 复算：226 tick，11.250529800s，按报告定义 **19.999058178Hz**，最大间隔 **53.420500ms**，P99 **51.103400ms**，0 skip；owner 单 tick 工作 max **17.794700ms**。另列首个实际 tick 的微小启动差异后，首末实测频率为 19.999062266Hz，没有偷换报告公式。关闭原记录 0.0538866s，child 已退出、未 terminate。

只进行了一次真实 v22 图像匹配，原曝光 `101.782s`、先前 pose `101.780s`；12 个模板的 SHA/good-matches/inliers/accepted-candidate 与原同步报告逐项完全一致。result worker PID 29968；原报告没有保留主 PID，独审不补造数值。得到候选不等于官方图库或类别已确认。context=true，fresh/recognition/official 三标记仍 false。

原 report SHA `ee7da6b0a61940364e829c5b2b2a090a6b0209407503bb6f8b2762539b109401`，CSV SHA `4826085d3e130481f213de74e6aa20f95bd2cf54f28cc06ed9ed13d098dcc1a6`。独立复算脚本 `recompute_production_benchmark.py` 和结果 `benchmark_independent_arithmetic.json` 保留来源 identity。

当前独立发现已修复，6 项反例与保留 benchmark 算术通过；未发现阻断本地离线使用的新问题。新单点节点中的 ROS 执行器、线程资源、传感器时效、唯一板关联和实际裁判计分仍须另行验证。
