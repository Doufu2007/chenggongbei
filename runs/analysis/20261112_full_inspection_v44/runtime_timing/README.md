# v44 提前结束的只读调查

本目录仅分析已保存的本队测评、公开前端和本队代码；没有平台请求、私有裁判访问或生产源码修改。原始详情中的账户字段、签名资源地址与诊断接收地址均未复制。

## 已证实的事实

官方详情终态为 `success / 10分 / 492622ms / abnormalEnd=false`，而非全巡检完成证明。原始详情仅有裁判启动与一次陆地到达提示，无完整分项诊断。`result` 仍是排队文案，因此该字段不适合作为完整结束证据。

| 节点（北京时间） | 时间 |
|---|---|
| 官方开始 | 21:55:07 |
| 裁判启动日志 | 21:56:27.049989 |
| launcher_started | 21:56:32.508126 |
| 首个 GROUND heartbeat | 21:56:40.338788 |
| 最后 GROUND heartbeat | 22:03:11.788548 |
| launcher_exit，上报退出码0 | 22:03:12.572226 |
| ground_motion_core | 22:03:12.897238 |
| 官方结束 | 22:03:20 |

最后 heartbeat 的地面累计时间为391.450584秒，仍处于第二目标 `ground_4 / point_index=1 / completed_count=1 / DRIVE`。core 累计392.267秒，`reason=full_pipeline_ground_stage_exit`，明确保存了 `axis_command_publish_failed:RCLError:Failed to publish: publisher's context is invalid`。这证明清理时 ROS context 已失效；尚不能区分收到信号、进程组清理或其他 context shutdown 来源。

本队启动脚本没有 `timeout` 命令，入口的本地地面时限是1800秒。本地 `spin_control_loop` 的退出入口是 `ros.ok()` 为假、duration达到上限、FINISHED，或异常。末次状态不是FINISHED，实际持续远未1800秒；当前全程入口除 main 的 finally 以外没有主动调用 `rclpy.shutdown()`。因此不能用本地duration正常到期解释这一退出。

## 尚未证实的原因

检查了两版已缓存详情页JS、测评指南、官方注意事项与判分细则。详情前端有“评测未正常结束（疑似超时）”“时限内未完整跑完全部任务”提示，但没有具体运行时限数值。该提示依赖 `success && abnormalEnd`，本局为false。约400秒的launcher时长仅是一次观察，不能据此宣布存在400秒平台deadline。

launcher的EXIT事件比同一台平台主机上的Python core事件早0.325012秒。单独的EXIT上报码0也不可靠：附带的Bash合成实验在本机复现了 `controller_term → launcher_exit=0 → child_term → child_cleanup`。Windows Git Bash外层返回码为3840，与EXIT陷阱记录的0不同。此实验只说明“陷阱先于子进程清理且捕获旧0值”在信号退出下确实可发生，不能证明本局收到了任何特定信号，也不代表Ubuntu/rclpy行为完全相同。

## 下一版最小判别诊断建议

1. shell显式记录INT/TERM来源，并保留130/143等非零信号退出码；必要时向受管子进程转发同一信号。不得吞信号后继续执行、重新启动任务或延长平台时限。不要把EXIT陷阱的旧 `$?` 当作Python的真实退出码。
2. 在controller开始 `_dispose`、覆盖guard.reason之前，排入一个轻量 `stage_exit` 事件，保存原始state/reason/phase、point_index/completed_count、elapsed、`ros.ok()`、return_completed与导致spin离开的异常类型。记录“正常返回/异常返回/ROS不可用”等观察，不猜测外部原因。
3. 首次STOP原因也应只写一次，在后续清理前保留。使用现有非阻塞本地队列；网络请求不得进入控制tick或信号处理路径，不因诊断发送失败改变控制、成功门或官方分数。
4. 验证应包含合成INT/TERM退出、ROS变为不可用、已有STOP原因在cleanup后仍可查，以及诊断队列失败时控制仍能归零。平台下一次只改变这一组诊断时，可凭真实信号号/原始退出原因判断外部中断假说。

重放本分析：`python runs/analysis/20261112_full_inspection_v44/runtime_timing/analyze_timing.py`。`summary.json`保留筛选后的数值与原始证据哈希；`signal_order_*`仅为合成Bash实验。
