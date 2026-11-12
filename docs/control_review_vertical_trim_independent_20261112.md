# 低高度 Z trim 独立复核

2026-11-12。**最新结论：发布边界修复已独立复核通过，可按固定协议开展 v29 有限平台辨识。** 本 reviewer 未上传。以下旧 SHA 的阻断和反例保留为历史；不得用旧版本打包。

## 发布边界修复的定点复核

新源码 SHA：`54744be026c23d43bfe8dba5686ffd7d25d9028a8d4eaf06e667501b5984d89c`；脚本保持 `bb304a609f3c72da8baa462c61366444737aeab80c1ea93cfeca11f014502a95`。已实核文件哈希。作者新增7项先红后绿，35项trim专项检查；本 reviewer 只定点复核，不重复149项历史检查。

新独立脚本 `runs/review/vertical_trim_independent_20261112/publish_boundary_recheck.py` 及同目录 `publish_boundary_recheck_metrics.json` 保存五项复核：

1. 原慢成功 ground publish 条件保持：负阶段期限 `3.45s`，ground 调用返回 `3.51s`。修后 UAV 只实际收到零，记录同为墙钟 `3.51s` 的零请求，永久进入 `ZERO_RETURN`。
2. ground 调用及时返回但后续 DDS matched-count 查询自身耗时到 `3.51s`，同样只发送零，永久中断。
3. 新 DDS 查询抛异常，吸收锁停并仍成功发送 UAV 零。
4. 新 DDS 查询发现匹配消失，进入永久零返回，不能恢复有效反馈后再发非零。
5. 正常 factory→callback→tick→signed publish 合成通路仍完整：166条实际UAV请求与证据逐条一致，包含6条负请求；取得合成支撑高度驻留后才结束，运动时间7.1秒。

原慢成功 publisher 漏检已经关闭。适配层在 ground 发布和随后的 DDS 查询均返回后重新取得墙钟、核永久预算／阶段时限／源龄／匹配／命令，避免已知过期请求继续发出。仍不保证阻塞调用期间硬实时归零，也不保证底层 UAV publish 调用内部时延或执行器即时响应；这些是后续实测需记录的边界。放行仅限规格内低高度辨识，不扩大为悬停、受控降落或完整任务成功。

## 新确认的发布边界缺口

反例：`runs/review/vertical_trim_independent_20261112/delayed_ground_probe.py`；原始结果：同目录 `delayed_ground_metrics.json`。

实际 factory/callback/tick 到达负 trim，阶段开始墙钟 `3.1500000000000004s`，固定 `.30s` 到期界 `3.45s`。下一次 tick 在 `3.16s` 进行合法性检查；随后 `/cmd_vel` 的零请求发布成功，但模拟其调用耗时使墙钟推进到 `3.51s`。此时新的 signed publisher 仍向 `/uav/cmd_vel` 实际发送 `-.010`，并将该请求以墙钟 `3.51s`、状态 `ASCEND` 记录为成功。既跨过负 trim 期限，原 odom/scan 也已陈旧 `.36s`。

这不是“阻塞中无法瞬时制动”的不可避免保证问题：ground 调用已经返回，UAV 非零发送前有机会再次检查但当前没有执行。应在成功 ground publish 后、实际 UAV 非零发送前重新核对最新墙钟、永久预算、phase 期限、传感器源龄与命令合法性；过期只能发送零。独立 watchdog 在当前 command/guard 锁内无法介入阻塞调用，该硬实时局限仍需保留，不能把补检查宣称为阻塞期间的保证。

此前九例通过仍有效，但不覆盖这个慢成功 publisher 场景，因此不足以放行旧 SHA。

## 冻结对象

- `src/inspection_mission/inspection_mission/bounded_vertical_trim.py`：`445d23768dda55ea70195ca0a1cfbff2b4ca2a7d6df1269f1a0ed0d18f210abd`
- `scripts/run_bounded_vertical_trim.sh`：`bb304a609f3c72da8baa462c61366444737aeab80c1ea93cfeca11f014502a95`

已核原 v27 guard、已计分单点源码和观察路线 SHA 均等于冻结基准，没有改动生产源码。只读核验作者 JUnit 为149检查、0失败、0错误；没有重复整仓检查。

## 独立反例与接线验证

独立脚本：`runs/review/vertical_trim_independent_20261112/independent_probe.py`；结果：同目录 `independent_metrics.json`。执行命令：

```text
python runs/review/vertical_trim_independent_20261112/independent_probe.py
```

六组、九例均通过：

1. 实际 `create_trim_node` 建立实例后，经原订阅 callback、`tick` 和新的 signed `_publish` 路径执行一个独立合成执行器。实际输出的166条 UAV 请求与证据中的166条命令逐条相等，包含6条负值；地面请求全零，其余 UAV 轴全零。固定正/负阶段全部执行后，只有在支撑高度附近静稳才 `FINISHED`；合成模型总运动墙钟7.1秒。
2. 两例在实际 callback 刚转 `PHASE_GATE` 时分别注入 NaN odom 和 NaN scan，再送有效替代帧。两例都吸收锁停，不能覆盖非法输入继续非零。
3. 实际负向阶段的低高度 callback 立即请求 UAV 零并永久进入 `ZERO_RETURN`；后续恢复高位反馈不补跑非零阶段。
4. 三种状态下验证首次运动后10秒预算：`PHASE_GATE`、`ZERO_RETURN` 以及显式状态故障注入的 ascent `PAUSE`。三例均因 `motion_wall_time_limit` 锁停，原点与起始时间不重置。
5. 在整个第一 trim 阶段保留新鲜前进的20条 odom、但仅实际发送零请求，阶段结束时因缺少本阶段成功非零请求永久退出；不能把反馈推进当请求成功。
6. 在实际负向阶段停止 tick、只让独立 watchdog 工作，成功请求零并永久中断协议。

以上 transport 由本机模拟 ROS transport 提供，运行的是实际工厂、回调和发布适配接缝；不是 ROS DDS、Gazebo 动力学或官方成绩的证据。合成执行器的各请求响应值只用于通路验证，不是对平台插件的拟合。

## 静态核查与适用边界

新 guard 覆盖永久 `_budget`，包括 `OBSERVE` 和 `PHASE_GATE`；复用回调动态访问实例 `guard`；新 publisher 实际替换实例 `_publish`，负号不会被旧仅正值校验截断。正负零计数按实际成功发送累积。地面通道异常独立尝试 UAV 零；原独立 watchdog、传感器源龄、DDS、水平环扫和 TF 门保留。`PHASE_GATE` 的非法 odom/scan 已有子类即时锁停处理。

零请求已在 v27 被观察到持续下降。这里的故障归零仍不代表悬停、触地或受控降落；`support_return_verified` 只是 odom 回到起始支撑高度附近并静稳。近零正请求与负请求的真实动力学、调度延迟、停止响应、是否接近 `.15m` 上界，必须由此次平台数据决定。固定上界不足时应保留协议未完成结果，不扩大动作或补跑阶段。

reviewer 未打包、上传或启动平台实验，也未改作者 manifest 的 pending 状态；由主控核对本报告 SHA 与冻结源码后决定练习调用。
