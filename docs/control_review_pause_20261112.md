# PAUSE 有界观察补丁：独立控制审查

日期：2026-11-12。审查只读，未修改控制源或已上传包。审查对象是拟用于下一轮练习的工作区源码，**不是 v11 archive**。

- `bounded_ground_probe.py` SHA256：`296ceee6a535da20176ef1e97501a453af5936929d5777197fd8412cc5cd8d32`。
- `ground_observation_survey.py` SHA256：`2c826c5b7bc6e4f1fd30ae5b5d3e5d9c326d44583fe88c005a87a1e867d0ecbf`。
- 独立运行 `tests/test_bounded_ground_probe.py`、`tests/test_ground_observation_survey.py`、`tests/test_ground_probe_launch.py`：**143 passed in 4.57 s**。

结论：上述冻结源码可进入下一轮**有界地面原地观察练习**。该结论不代表已完成巡检、已具备全部官方接口或能够获得比赛成绩。

## 已核查的安全与恢复条件

1. 非法 clock/odom/scan、时序倒退、近障、超速、位移/tilt 越界均触发永久 STOP；活动期每条有效 odom 的物理越界会立即锁存，不能被下一正常样本覆盖。
2. TURN 或 PAUSE 中继续累计观测旋转，累计角预算、到角/越过目标可在回调锁停；暂停不会清空既有旋转或最大位移。
3. 临时 freshness 故障进入 PAUSE，命令线速度/角速度均为零；数据重新 fresh 且静止，墙钟与 odom 仿真时间均满 1 秒后，才恢复原目标。
4. 恢复最多 3 次；`pulse_started`、`pulse_origin`、目标、初始方向及累计预算不重置。PAUSE 消耗原 25 秒转向墙钟预算，1.8 rad 旋转、0.06 m 位移、0.2 秒新鲜度上限保留。
5. 图查询位于发布锁外；watchdog 独立发送零速。锁顺序未出现同时持有 guard→command 的反序路径；tick/handler 的 command→guard 顺序不会因此死锁。
6. 全部 fresh 的 PAUSE 中，watchdog 仅持续发布零速，不调用临时故障处理反复清驻留。

## 复核期间发现并修复的竞态

初稿 `temporal_fault` 无 guard 锁。独立双线程复现：watchdog 通过 TURN 条件后暂停在赋 PAUSE 行；另一线程先写入 `STOP/unexpected_motion`；放行后旧赋值把硬停覆盖为 PAUSE。该行为可能恢复已因硬故障停止的动作。

控制代理已在 `temporal_fault` 与 `handle_watchdog` 整段使用同一 `RLock`，并先观察失败、再修复通过并发回归。独立重跑确认：暂停转换期间硬停线程正确串行，最终状态保留 `STOP/unexpected_motion`，之后补入 fresh 数据也只返回零速。

## 已记录的次要边界

watchdog 先计算的 freshness reason，可能在等待 command lock 时已过时；处理这个旧 reason 可能额外清一次已经建立的 fresh 驻留。它使恢复更保守，不会复活硬停、绕过预算或产生越界非零命令；本轮不因此扩大补丁范围。

0.5 rad/s 是本轮候选实验请求上限；候选半径0.447 m下的尖端线速度约0.2235 m/s，只是计算值，不验证官方机体尺寸、真实控制增益或碰撞安全。实际平台回传仍需逐局检查；上传及后续改动须核对上述源码哈希和包 manifest。
