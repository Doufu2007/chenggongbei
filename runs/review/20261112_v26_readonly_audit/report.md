# v26 本队证据独立复算

2026-11-12。只读检查既有文件；没有操作浏览器、上传或修改生产源码。未读取裁判私有信息。

平台本队列表缓存 `runs/evidence/20261112/three_point_v26_list_0704.json` 中，提交 `0425d992-ddd4-4b91-9d21-b3d6a394728e`、`#90809618`、`run_20261112_070005_77bdabed` 已终态 `failed`，`score=null`，`durationMs=98393`。更新时间为 `2026-11-12 07:02:11.05099+08`。这表示本局没有提取到官方成绩，不能写成官方 0 分。

本队 callback 独立调用 `tools/analyze_ground_three_point_evidence.py` 解码到本目录 `decoded/`。原始 220310 字节，SHA256 `774b734f87b7c8de4194d9535ba9ba9d82641637360e270e369a34315a87fc03`；压缩分片、SHA、解压长度和流校验通过，`transport_complete=true`，无解析错误。155 条 10Hz 轨迹、411 条实际命令均无 drop。51 个事件，launcher exit=0。退出码和完整传输只证明本队运行/回传事实。

抽签为 `ground_4 → ground_1 → ground_3`。本队执行到首点观察位前第一条 DRIVE：运行记录只有 TURN 和 DRIVE 两项，目标 `(0.35,-0.1)`。DRIVE 原点与实际开始 pose 一致，从该原点到目标的 bearing 与 `drive_yaw_rad=-0.2760552089569863` 完全一致，未发现锚错接。未完成任一点，`completed_points=[]`、CAPTURE=0、点内识别为空、没有本点采图事件。

原 20mm 横向硬门得到逐点复算支持：

| odom stamp | 本队 wall elapsed | 横向偏差 |
|---|---:|---:|
| 17.06s | 14.244208935s | 18.976910mm |
| 17.16s | 14.346334151s | 20.152648mm |
| 17.26s | 14.446746682s | 20.264131mm |
| 最终快照 | 非同一10Hz采样时刻 | 20.263156mm |

最后非零命令在 wall 14.326993844s，仍为 `linear_x=0.1, angular_z=0`。首条 STOP 零命令在 14.348095741s，比首个越门轨迹样本晚约1.762ms，此后全部命令为零。16.34s 至17.16s 的轨迹横向偏差连续增长，说明越门已发生在停止请求之前。最终10Hz轨迹复算20.26314879mm，与快照差约0.00000714mm；二者不是完全相同的采样时刻，不伪造精确触发位置。最终状态 `FINISHED / approach_cross_track_limit`，resume=0。

既有真实 guard replay 为 `tests/test_three_point_v26_evidence_replay.py`，结果 `runs/review/20261112_v26_actual_replay.xml`：2项通过。其17.16s真实pose在原20mm合同下复现 `approach_cross_track_limit`，并显示位移bearing与body yaw不一致。本次未新增同类pytest；上述数字已由只读Python命令逐项复算。保存了可复算脚本 `independent_probe.py`，按主控停止追加验证的指令，未执行该脚本、未生成额外metrics文件。

下一项最小工作证据支持有限方向标定：在既有硬门下区分速度档位、body yaw与真实位移bearing。当前材料不能确定平台具体驱动原因，也不能支持继续上传同一斜腿或声称三点已完成。未提出放宽门限。
