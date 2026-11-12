# v40 同账本三地面点与真实返航实现

2026-11-12。按root已批准的15m/40rad/1800s、turn1000s/drive650s独立合同实现。**这是待平台验证能力；没有上传或启动本地/平台测评。v38实际因`single_point_visual_binding_rejected`终止，不能写作三地面点已经成功。**

新增 `ground_return_plan.py`、`ground_three_point_return.py`、`scripts/run_ground_three_point_return.sh` 和必要专项测试；未改setup.py、配置或v38/v36及冻结父源码。10个冻结依赖前后SHA完全相同，详见 `runs/review/ground_return_v40_author_20261112/source_manifest.json`。

## 可运行接口与状态

```python
from inspection_mission.ground_three_point_return import (
    ReturnThreePointLimits, GroundThreePointReturnGuard, create_return_three_point_node,
)
node = create_return_three_point_node(reporter, wall_clock=clock, worker=worker)
```

工厂复用冻结v36受检`guard_factory`，构造时核对同一limits对象和创建墙钟；没有先造旧guard再替换。默认/上限为path15m、rotation40rad、global1800s、turn累计1000s、drive累计650s，其余原门及恢复32次/单5s/累计60s保留。启动器执行 `python3 -u -m inspection_mission.ground_three_point_return --duration 1800`，真实退出码保留。

第三点的声明发送返回并满足原2s等待后，只归档一次，从当前真实末pose规划返航并在同guard加载`role='return'`。session、origin、创建时间、累计path/angle、turn/drive时钟、恢复和累计暂停均不清零。旧视觉epoch作废，返航不接收新识别jobs/results、不发识别/到点声明；原三点发送与原始帧已归档。所有UAV/灯/飞行声明继续静默。

返航operation耗尽被独立分支接住，不进入父类`DECLARE_WAIT`。只在真实xy半径≤.02m、yaw误差≤.025rad且原速度/锚点门满足、源stamp及墙钟同时驻留≥1s后置`return_ready`。原adapter真实零publish返回后，再复核实时freshness/净空/预算/终点/驻留，才置`return_completed`，并STOP锁存`three_ground_return_complete_official_score_unverified`。发布抛异常、迟返过期、坏帧或预算不足不形成完成；迟返过期会清ready并走原PAUSE账本。

完整运行桥应以`guard.return_completed`交接飞行；`return_ready`及任意STOP均不能视为通过。这里不负责飞行阶段接线，也不发`mission_completed`。同一guard保留给桥读取完整累计账本。

## 验证与真实边界

先红后绿证据在 `runs/review/ground_return_v40_author_20261112/`：纯planner最初3项缺功能失败，新增guard/launcher15项缺功能失败，修正独立单腿验证须保持原3m cap；零发布迟返的freshness状态断言也先失败再修复。

最终命令：

```text
python -m pytest tests/test_ground_three_point_return.py tests/test_ground_three_point_recovery_budget.py tests/test_ground_three_point_camera_recovery.py tests/test_ground_three_point_axis.py tests/test_ground_three_point_axis_plan.py -q
```

**85 passed in 50.43s**，其中v40专项22项、必要冻结父关联63项，未重复项目全套。覆盖60种名义路线/原净空拒绝、独立预算、第三点handoff与永久账本、返航视觉/声明禁止、真实终点/双钟驻留、迟返/失败零发布、真实factory成功与失败、receiver事件格式、启动器退出码，以及正向整三点`g4→g3→g0`后驱动返航的合成反馈。

名义60种最大13.99m/37.752432539827rad仍只是公开候选几何。合成反馈`g0→g3→g2`实际累计27.766924321rad，末pose约`(.000786516,-2.688987220,3.117287140)`，返航计划12.542065101rad，加原.10储备为40.408989423rad，**正确拒绝**40rad合同。该拒绝已单独保留测试；不重锚、不改预算，不用名义角替代实际角。ground_2向南偏20mm的原`.640<.641m`拒绝也保持。

## 冻结哈希与剩余项

| 新文件 | SHA256 |
| --- | --- |
| `ground_return_plan.py` | `a4f4709b7b2dbcbd752fe9cf15307c9c1f56e74cd119dece7bc6afee79cb9a1d` |
| `ground_three_point_return.py` | `b70cd5dde22054cae748cbb6bfcff7e26c5b40c5ba80f1760c8b2aac355d79dc` |
| `run_ground_three_point_return.sh` | `70e0d537c39a28eba379cb2a08bfbd361391f9fc282d413d2b66d016b3f24520` |
| `test_ground_three_point_return.py` | `e75f115d9a948efb41ed4bcb2b4e143abc01d66ebb30f921926489d08d851816` |

剩余：root安排非作者定点审查；完整运行桥集成后的必要联合检查；同局三地面点/返航真实测评及原始证据独立分析。当前没有官方三点/返航完成或分数证据。本文未上传局部测评、未继续高度试验、未扩大飞行范围。
