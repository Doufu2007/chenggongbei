# v38 官网运行独立证据审查（2026-11-12）

本轮在首个目标 `ground_3` 的视觉绑定阶段停止，原因记录为 `single_point_visual_binding_rejected`。运动到观测位和一次普通 `scan_stale` 暂停恢复均有证据；新增相机恢复及第 4～32 次恢复资源未在本轮触发。未取得官方分数，不能把本报告解释为官方零分或完整任务验收。

## 证据完整性

- 原始回传：`runs/evidence/20261112/callback_three_point_recovery_v38.jsonl`，561,581 B、59 个事件，SHA-256 `ba8cc75e76b677ba5ef4e11861c2bc9af2c9b0033f1a08dc718b280cb1060854`；末事件 `launcher_exit` 的退出码为 0。
- 8/8 gzip 分片、无重复；压缩体 262,893 B、解压体 877,402 B，双长度与双 SHA 校验通过。独立解码对象与父任务 `bundle.json` 相同。
- mode、revision、session、终态/原因、origin、路径/转角、抽签/目标/索引、冻结限制、完成数和 current point 共 11 项绑定全部通过。
- 抽签顺序为 `ground_3 → ground_2 → ground_0`，终态 `FINISHED`，point index 0、completed 0。状态标签不等于任务完成。

## 停止位置与视觉证据边界

STOP 日志 elapsed 为 **104.776217 s**；CAPTURE 至 STOP 为 1.991041 s，RECOGNIZE_WAIT 至 STOP 为 1.282755 s。留存了 3 张原 JPEG，源时间戳为 104.195、104.529、104.862；对应真实先前 UAV odom 的偏差为 15、9、2 ms，均在 50 ms 约束内。三张图与父任务保存文件逐字节 SHA 一致。10 Hz 留存采样支持三次曝光前至少一秒停稳，但不能重建每一次高频停稳检查。

`match_results=[]`、`matcher_failures=0`、`recognition=null`，识别/声明发布的 attempted 和 returned 均为 0。源码 `ground_single_point.py:359` 的 `accept_match_result` 在 current context、expected job、error 等前置检查后调用 `bind_ground_candidate`；返回 `None` 时先停止，成功绑定后才追加 `match_results`。因此空结果不能解释为 worker 没运行；被拒结果未归档，现有回传不能进一步给出候选类别或具体几何拒绝项。上述 1.282755 s 也不是已测 worker 执行时长。完整三帧、原 K 的离线视觉重放由父任务另行负责，本独审未执行 SIFT。

## 恢复资源与停止时的新鲜度

仅一次普通 DRIVE 阶段 `scan_stale` 暂停：开始 elapsed 2.565560 s，持续 **1.104354 s**，计数 0→1；入场 scan 墙龄 0.203558 s，clock/odom 当时新鲜；恢复时 odom、scan 均新鲜。暂停内 42 条命令全零，恢复后的首条命令也为零。暂停起止差、累计账本一致，active pause 为 0、无记录丢弃。

相机恢复 events 为空、pending 为 false、recapture count 为 0。本次未触及 5 s 单次、60 s 累计或 1200 s 总任务上限，不能将新增相机恢复或 32 次额度称为已在官网验证。

STOP 日志 clock/odom/scan 均新鲜，scan 墙龄 0.122168 s。最后一次识别检查时相机墙龄 0.218867 s、clock−image 0.138 s，处于 0.5 s 门内；该检查比下一条 STOP 日志早约 55 ms。core 最终快照的 scan 墙龄 2.114303 s 属于停后快照，不能倒推为停止原因。

## 实际运动与时序

连续账本 path **1.639658 m**、rotation **2.072342 rad**；10 Hz 轨迹下界分别为 1.635554 m、2.068387 rad，未超过连续账本。两次 DRIVE 终点重算误差为 1.094 mm、5.339 mm，均小于 20 mm；最终观测位 XY 误差 **11.566 mm**、yaw 误差 **−0.024503 rad**，仍在 0.025 rad 门内，接近边界本身不能证明视觉根因。5 个 operation 账本单调，无账本错误。

2,339 条命令中非零 1,848 条、零 491 条，无负 x、相位/幅值/同时 x+w 违规。CAPTURE 至 STOP 的 78 条及 STOP 后的 85 条命令均为零。core 零计数为 490，比 bundle 少一条；两快照采集时间不同，无法仅据此确定计数差的精确原因。完整快照回退为分段传输后未传 `axis_publish_attempts`，发布失败数为未知，不能写为 0。

tick 最大 **3.678 ms**，均值 0.378 ms；spin_once 最大 68.949 ms；UAV 图像接收回调最大墙时 41.714 ms、CPU 0.434 ms；scan 接收回调最大 2.182 ms。图查询最大 0.338 ms、1,266 次总计 72.567 ms。这些数据不支持“主 tick 被 matcher 独占约一秒”的解释。全程 UAV 图像入口 gap 最大 0.790875 s 的直方图不能定位视觉阶段；未测 DDS/executor 上游分解，不能据此精确归因网络或调度。

## 可复核材料

独审脚本及六份 JSON 位于 `runs/analysis/20261112_three_point_recovery_v38_independent/`，`manifest.json` 冻结输入、依赖、分析产物和本报告的长度与 SHA-256。仅进行本地只读解码、源时间/prior、资源、运动和时序审计；未修改生产、上传或启动仿真。后续结论应与父任务的视觉重放证据合并。
