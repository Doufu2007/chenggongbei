# 独立 v36 相机短空窗恢复

v35 的停止原因是当前无人机相机墙钟龄 .530038013s 超过原 .50s 门，发生于 RECOGNIZE_WAIT，尚未识别上报。控制流仍新鲜；worker 两帧已识别但第三帧未形成完整结果。本修复只让这类合法正向超龄在未 attempt 时进入零速 PAUSE，恢复实时图像后重新满足原 1 秒双时钟静稳，再由父逻辑消耗原恢复额度并开启新 epoch 重采。不修改冻结 v35 文件，不放宽图像 .50s、控制 .20s、先验 .05s，保留每点一次重采、全局三次恢复及跨点累计路径/旋转/用时账本。

新增模块 `src/inspection_mission/inspection_mission/ground_three_point_camera_recovery.py`、启动器 `scripts/run_ground_three_point_camera_recovery.sh`、专项测试 `tests/test_ground_three_point_camera_recovery.py`。新 factory 从构造开始生成新 guard；内部可选 `guard_factory` 仅供后续固定子类复用，校验类型及同一 frozen_limits 对象、同一起始墙钟，无 CLI 类加载或运行中 guard 替换。除构造段外，factory 全部传输/最后 DDS/派发/独立零速/计数/异常逻辑与冻结 axis 工厂 AST 完全相同。

结构非法、header 倒退及视觉使用期间 clock-image < -.12s 继续永久停；REPORT_WAIT 或已有 recognition attempt 后不重采、不重报。移动期相机 stamp 暂时领先当前 clock 不新增负差锁停，负差判断局限于 CAPTURE/RECOGNIZE_WAIT/REPORT_WAIT 的视觉使用。重复 stamp 不刷新 arrival。恢复事件记录真实 wall/source/原因/epoch/点位与已用额度，最多 16 条；不输出裁判结束或成绩标记。

验证记录：原冻结 guard 的相机空窗专项先红 16 失败、3 通过；首次实现 19 通过。随后独立验证无回调 PAUSE tick 的墙钟单调门，先红 1 项，修复后纳入；移动期采样差与 factory 注入先红 2 项，修复后最终专项 22/22 通过。必要关联 axis、single_point、matcher_worker 连同最初 20 项专项合计 108/108 通过；后续仅新模块的移动期负差边界和内部 factory 注入接线变动，最终专项再次全部通过。py_compile、Git Bash bash -n 通过。所有 9 个冻结父文件 SHA 与原 manifest 一致；详见 `runs/review/ground_camera_recovery_v36/freeze_manifest.json`。

v35 在识别前已耗尽 resume_count=3，所以相同完整传感器时序在默认 v36 下仍必须因额度耗尽停；本修复不能被描述为官网通过、完整任务完成或必然得分。实现尚未上传，须独审后由主代理选择候选版本并在官网实测。
