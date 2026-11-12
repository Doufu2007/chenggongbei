# v19 真实短距速度及停车校准

2026-11-12；仅练习控制验收。平台 ID `17f62d65-e85d-46ed-bd78-be604d0b0de3`，runId `run_20261112_043048_33aebe21`，编号 `#31369881`。权威终态 `failed / score=null / 146.274s`，来源 `runs/evidence/20261112/speed_v19_list_after_callback.json`，原因仍为未检测到完整结束标志。没有取得有效官方成绩。

## 实际取得的证据

源码 `dist/inspection_ground_speed_v19_src.zip`，111002B，SHA256 `57106a6b23a5ce21b1c6b5e6c3b37a14b15ad09147a04cc87f0b1fa14bf3761a`；脚本 `dist/run_ground_speed_v19.sh`，2883B，SHA256 `7856a1282d8f5b8caf00113477f983b00c21cf5c30a69ba632129e264f86dc3c`。上传包从冻结 v18 包逐项校验后只新增 v19 模块，未混入开发中的地面匹配器。1095 项本地检查和 9 项独立反例通过，均非比赛成绩。

原始回传 `callback_speed_v19.jsonl` 为 532695B，SHA256 `320f3b16d0cef2925c8809585c8109de5e315d382221798faf5a11ffbf209daa`。52 个事件含 launcher_started、probe_started、launcher_exit=0。718/718 轨迹、1690/1690 命令已完整重组，两序列无冲突或丢行；分析仍标 partial，保留缺少整块 full snapshot 的事实。原图五张，CAPTURE 零张，不能称完整观察成功。

本局公开抽签 `ground_2,ground_1,ground_3`。实测总前向投影 0.200286m，累计路径 0.214053m、最大起点位移 0.206425m，恢复次数 0。

| 校准项 | 真实结果 |
| --- | --- |
| 低档请求 0.03m/s | 首段前向约 0.05005m 后请求停车，累计停车路径约 0.0000589m；静稳 wall/sim 各约 1.009/1.000s |
| 高档请求 0.10m/s | 约 0.761s 到前向增加 0.040947m 的触停点，实测速度峰值 0.059546m/s |
| 高档停车 | 触停后累计路径 0.000773m，含停车的整段路径 0.041730m；静稳 wall/sim 各约 1.009/1.000s |
| 离线独立下采样复算 | 7 个完整高档命令区间共 0.038722m/0.710985s，约 0.054462m/s；慢档 308 区间约 0.004955m/s |

原始高频 guard 样本与独立 10Hz 区间口径不同，不把两者数字互换。首次成功零速记录仅证明本队 publish 返回，尚无执行器 ACK。短高档脉冲不能直接证明长段持续响应或完整路线可达。

## 本局未完成的部分及下一步

控制停止原因 `approach_capture_wall_time_limit`，capture_count=0。末张 UAV 图 stamp=75.428，未找到足够新鲜的前序 UAV odom；`/uav/odom` 后段头时间出现约 0.16–0.18s 间隙，last_associated 停留在 stamp=53.428 的行进画面。该局 `/odom` 已用 BEST_EFFORT，`/uav/odom` 仍为原可靠订阅。这个现象支持下一轮对 UAV odom 采用同样的 BEST_EFFORT 进行验证；尚不能把 QoS 当作已证明的唯一因果。

下一轮独立观察路线将保留原曝光前序、时差、速度、净空和停车门槛，修复两路 odom 的订阅配置，并采集 callback timing。路线只验证第一公开地面点的外部观察位；识别、到点、飞行、照射、降落及官方结算均须后续实际证据，不发送虚假的任务完成信号。

离线分析：`runs/analysis/20261112_speed_v19/summary.json`；独立算术：`runs/review/20261112_speed_v19/independent_real_metrics.json`；台账：`runs/records/ground_speed_v19_17f62d65.json`。
