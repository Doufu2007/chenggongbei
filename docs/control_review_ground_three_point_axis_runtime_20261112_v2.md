# 轴向三地面点运行器独立复审 v2

结论：**本次冻结候选的软件适配器审查通过，旧版两项阻断已关闭。** 结论只绑定下列源码与启动器；实际负 X 响应、制动和反向停稳仍未验证，不能据此执行完整路线或宣称官方成绩。

审查源码 SHA256：`e033c119790ad8a32a066cd21cd9fd841e5b967092b8d0ac0355fd5d5f602f76`。启动器 SHA256：`3b5251e1f4b5dcc8d48d23c59c6ce102b422af1e2c031fa5ae05f0e0032bdfd4`。本轮独审时间为 `2026-11-12T00:22:22.530637+00:00`。独审者未参与适配器修复，未修改生产代码、冻结清单或原探针，未打包、上传或启动平台运动。

旧报告 `docs/control_review_ground_three_point_axis_runtime_20261112.md` 保留为对旧 SHA `5ece68ce…7f24454` 的历史拒绝结论。本报告更新的是当前冻结候选结论，不覆盖旧反例证据。

## 证据与范围

执行命令：

```text
python runs/review/ground_three_point_axis_independent_final_20261112/final_independent_probe.py
exit 0，11/11 通过
"C:\Program Files\Git\bin\bash.exe" -n scripts/run_ground_three_point_axis.sh
exit 0
```

新增独审驱动与结果分别为：

- `runs/review/ground_three_point_axis_independent_final_20261112/final_independent_probe.py`
- `runs/review/ground_three_point_axis_independent_final_20261112/final_independent_metrics.json`

新驱动直接导入并调用原 `independent_probe.py` 中的五个函数，没有复制或修改其断言。原探针 SHA256 仍为 `2cedc240d441bc2d0d99e0df625389341ee612fcf02a55260b7d13034b08773e`，原结果文件也未覆盖。另加六项针对实际适配器的独立边界检查。全部使用模拟 ROS transport 与本地合成反馈，没有真实 DDS 网络、真实 matcher 识别、执行器回执或官方计分证据。

读取并核对了旧独审、作者报告、当前与初版冻结清单，以及新适配器和继承安全门的必要接缝。清单中的九个文件在测试前、测试后均与冻结 SHA256 一致。相对保留的旧源码，当前修复仅重建适配器及其 import，guard、planner 调用、反向运动语义和限值代码保持不变。没有重复运行作者的 179 项或整个仓库。

## 原两项阻断关闭

| 检查 | 当前独立观察 | 判定 |
| --- | --- | --- |
| 其他输出 topic 图查询期间 `/cmd_vel` 实际匹配从 1 变 0 | 最终实际匹配读取为 0，guard 缓存也变为 0；只有 `(0,0)` 发布，状态 STOP，原因 `no_external_matching_cmd_subscriber` | 通过 |
| 真实 `/cmd_vel` publisher 对负 X 抛异常 | 异常未逃逸；尝试顺序为 `(-0.03,0)`、`(0,0)`；状态 STOP，记录 `axis_command_publish_failed:RuntimeError:independent negative publish failure` | 通过 |

源码 `ground_three_point_axis.py:309` 开始先做轻量匹配和必要图刷新，`:316` 在全部描述性图查询后重新读取实际匹配。`:322` 获取命令锁和 guard 锁，`:328` 用查询返回后的当前墙钟重新 `step`；`:330` 再检查预算、观察运动异常和当前 `_condition`。`:334` 保留运动状态、幅度、有限值和禁止同时平移转向检查。发布异常在 `:275` 吸收锁停，`:340` 失败后在同一 tick 独立尝试零。

## 新增发布边界检查

以下四项均从原独审的真实 factory→callbacks→tick 夹具出发，在第二次实际 DDS 匹配查询中注入变化；每项查询均返回正匹配 1，故结果不能归因于 DDS 为零：

| 注入 | 实际门状态 | 发布结果 |
| --- | --- | --- |
| 墙钟从 `.05s` 到 `.30s` | clock/odom/scan 到达龄 `.25s`，超过原 `.2s`；PAUSE，`clock_stale` | 仅零 |
| 墙钟到 `1200s` | STOP，`three_ground_global_wall_time_limit` | 仅零 |
| 查询中调用 STOP | STOP，保留 `independent_phase_stop_during_final_query` | 仅零 |
| 相同墙钟下 `/clock` 从 `10.05` 到 `10.4` | odom/scan 到达龄仍为 0；源龄 `.35s` 超过原 `.2s`；PAUSE，`odom_stale_or_invalid` | 仅零 |

最后一项通过真实 `/clock` 回调更新状态，专门区分源时间龄与本地到达龄。底层 `_fresh` 仍同时要求墙钟到达龄不超过 `.2s`、源龄不超过 `.2s`，并保留 `.12s` 的既有 clock 采样相位下界。没有以当前墙钟重标旧传感器时间，也没有放宽这些门。

第五项使用真实继承 `arm_watchdog`，先将 guard 锁停，再令第一次零 publish 抛错：watchdog 线程下一轮仍发送零并成功返回。记录分别为 `returned=false` 和 `returned=true`，成功零计数至少为 1。该检查同时证明新 `publish_zero` 的异常吸收没有终止继承 watchdog。

第六项保留 factory 创建的 node.guard 和 coordinator.guard 引用，将同一 guard 送入已有合成反馈 harness 到达 RECOGNIZE_WAIT，然后把本 epoch 三帧匹配结果交给真实 coordinator 的 `worker.poll` 路径。实际 tick 完成接收结果→dispatch→String publisher，进入 REPORT_WAIT；context 为 `test-current-session/ground_0/2`。刷新图谱再 tick 后仍只发送一次，attempted/returned 均为 1；飞行、灯和完成事件输出仍标记静默。该探针只替换图像准备函数和 worker 输入，不直接调用 accept/dispatch 代替 tick，也没有把 node 或 coordinator 改绑到另一个 guard。

## 其余原独审结果

原真实回调负 X probe 仍发布 `-0.03`，command evidence 记录相同值；机体朝北向南移动 1mm 时，进度为正 1mm，机体 drive_yaw 保持 `+pi/2`。TURN_SETTLE 漂移后仍重新瞄准 `1.5618078048292405rad`，下一 forward 清除反向标志。

原额外三点顺序 `ground_1→ground_2→ground_4` 仍通过：旧 epoch 匹配被拒绝，epoch 为 2、5、8，逐点识别与声明对应目标，永久起点/创建墙钟不重置，路径/旋转/分组时间单调累计；终点账本路径约 `9.943m`、旋转约 `25.623762rad`。这仍是合成反馈证据。

新合同仍固定为旋转 28rad、路径 12.5m、原点位移 3m、全局 1200s、转向累计 700s/单次 45s、直行累计 500s；原父源合同与 planner 均未变化。实际发布检查继续使用冻结 guard 的反馈、安全和视觉准入条件。

## 放行边界

本轮未发现阻止当前适配器软件审查通过的问题。该结论不构成完整路线执行许可：负 X 的真实方向、制动距离、反向停稳、候选地图和窄几何余量仍须独立平台证据。局部软件完成、零异常退出或回传成功均不能代替真实三地面点完成与官方成绩。任何生产源码或启动器 SHA 变化后，本结论不自动沿用。
