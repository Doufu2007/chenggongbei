# 连续三个地面点：独立练习执行器

2026-11-12。**v26已真实上传并终态 failed / score=null；第一段直行横向偏差触发原20mm硬停，尚未完成任何地面点。** 本地32项合成反馈测试不代表平台三点成功。官方已获10分的v25单点及其底层route源码均保持原SHA。本版本不发布飞行、灯光、cruise或全任务完成消息。

## 平台实测更新（07:04后）

练习ID `0425d992-ddd4-4b91-9d21-b3d6a394728e`，runId `run_20261112_070005_77bdabed`，测评编号 `#90809618`。权威列表 `runs/evidence/20261112/three_point_v26_list_0704.json` 为 failed、score=null、durationMs=98393。平台文案仍为“流程未检测到完整结束标志（未提取到成绩）”；它不能替代本队实际停止原因，也不能把null改写为裁判0分。

原始回传 `runs/evidence/20261112/callback_three_point_v26.jsonl` 为220310B，SHA256 `774b734f87b7c8de4194d9535ba9ba9d82641637360e270e369a34315a87fc03`。独立解析得到完整压缩bundle，SHA/长度/分片校验通过；155条轨迹、411条实际命令、相关drop计数均为0。抽签为 ground_4→ground_1→ground_3，第一点第一直腿触 `approach_cross_track_limit`，终末横向偏差20.2631559331779mm，completed_points为空，尚未进入首点识别。

直腿实际起锚与该操作actual_start_pose一致；实际目标(.35,-.10)对应bearing为-0.2760552089569863rad，与存储drive_yaw逐值一致。10Hz轨迹首个越门样本为stamp17.16s，横向20.15264842038747mm；终末回调更晚，故与20.263mm不矛盾。真实位移方向与车体yaw不同，且.03/.10两个请求档位的偏差不同；现有pose/速度模长不能区分接触滑移、控制桥变换或odom语义，不据此断言当前平台控制坐标系。

既有真实回放测试 `tests/test_three_point_v26_evidence_replay.py` 两项通过，JUnit `runs/review/20261112_v26_actual_replay.xml`：确认当前锚正确、保留20mm门时能重现硬停，并反驳仅改变等比例速度增益的理想车体直行模型。这些是同一局的回放，不是新增平台重复试验。诊断数据与可复算脚本位于 `runs/analysis/20261112_three_point_v26/`。后续应先做独立有限方向标定，不放宽横向或净空门以消除失败。

以下实现及合成测试记录为上传前冻结历史，平台结论以上述实测为准。

## 实现范围

新增 `ground_three_point.py`，继承v25的传感器校验、真实曝光绑定、三帧一致识别、唯一发送锁存和停止行为；新增纯 `ground_three_point_plan.py`，不含ROS、网络或本机分析路径依赖。启动脚本 `scripts/run_ground_three_point.sh`只加载已上传源码并前台运行本队程序，不启动仿真或裁判。

同一guard冻结一次真实三点抽签、一次起始pose与墙钟。永久起点、真实累计路径、绝对旋转、累计drive/turn墙钟、前序odom/yaw和resume次数跨点保留；只有局部动作锚、静稳锚、当前点视觉上下文和每点发送锁存允许重建。正常点间交接发生在进入吸收STOP之前；安全STOP不能复活。

每点顺序为：显式TURN/停稳、DRIVE/停稳、观察位朝向、三帧曝光、三次匹配一致、一次真实识别话术发送、等待1秒、从实际pose重算入口、真实末位朝向对齐、停车静稳、一次目标ID发送、等待2秒、记录该点运输事实，再计划下一点。此运输记录不是官方ACK或计分证据；后点计划拒绝时仍保留前点事实。

运动前取消旧worker上下文，并递增mission内单调epoch；active capture/jobs/results/recognition清空。已完成的静态视觉事实保存在独立 `point_visual`，之后归档，不能作为下一点识别。复用同一个coordinator，job序号跨点递增。原mission抽签与当前target分开保存。

## 预先冻结的预算

| 项目 | 新三点合同 |
|---|---:|
| 全程真实累计路径 | 12.5m |
| 全程真实绝对旋转 | 25rad |
| 距永久起点半径 | 3m |
| 全程墙钟 | 1200s |
| 累计转向墙钟 | 700s |
| 累计直行墙钟 | 500s |
| 单个转向/单次采图 | 45s/10s |

时间来自v25真实约36.0058wall s/m、22.9369wall s/rad。60种名义路线最坏约934.7s，工程预算加开销外推约1130.5s。**这是本队新合同，不是平台承诺的总时限或必成保证；外部截断上限仍未知。** 不在运行中增大额度。

原0.447m机体半径、0.641m环扫门、0.20s控制传感器年龄、0.05s严格前序配对、0.50s独立相机活性、0.02m跟踪、2mm提前请求零与1cm累计停车漂移门均保留。

每个操作列表仅最后一条直腿末0.10m请求0.03m/s，恢复v25语义；中间腿维持0.10m/s及原停车门。第一段慢速/停车实测校准只进行一次。

## 真实pose与最窄地段

每次点间路线和识别后入口均从真实当前pose规划。TURN紧接DRIVE时，根据真实当前pose重算下一航点bearing，并复核当前候选全圆弧；停稳后在同一45s分配内再核下一腿bearing，必要时重瞄。它不跳过ground_2的北半圆中间锚。

ground_2最后斜腿名义净空只比环扫门多9mm。原下一腿瞄准允许约10mm侧偏，合成反馈实际出现端点 `(0.002996848,-2.699315286)`、yaw `-3.010470481`，最终对齐圆弧已不满足0.641m。保留此反例为拒绝测试，仅将g2末斜腿前瞄准收紧至3mm侧偏目标，物理硬门和45s时限不变。收紧后两条完整合成三点序列通过；真实跟踪误差仍可能安全拒绝，不能据此宣称60种顺序实测稳定。

## 数据与回传

轨迹容量12500条（10Hz），实际命令容量30000条；相对于1200秒名义12000/24000条有余量。watchdog也会记录零指令，若实际超容量仍保留drop计数，不伪装无丢失。

`additional_events`先发送 `three_point_core` 和逐点 `three_point_point`，再发送独立压缩bundle，避免旧按数百小分片传输拖累关键证据。bundle包含完整survey、各点三帧/匹配/发送事实及最多9张JPEG。沿用原JPEG编码器；不复制一套图像编码逻辑。

压缩事件为 `three_ground_evidence_gzip_part`：`encoding=gzip+base64`、`bundle=ground_three_point_v1`、session、原始/压缩SHA256与字节数、part_index/count、至多48000字符base64。gzip的mtime为0。原始JSON `schema=ground_three_point_evidence_v1`，含survey、completed_points、current_point、capture_image_events。每包低于64KiB。

新guard最终回传墙钟上限180s；旧base fallback仍可能在关键bundle后追加重复或partial信息。网络慢或平台外部截断时不能保证全部回传；以独立解析器的SHA/长度/分片完整性结果为准。完整bundle可还原的范围为最终快照时刻之前；回传期间watchdog继续发零指令。

## 验证记录

先观察缺少新模块的RED。新反馈测试验证同一个真实guard实例跨点运行、抽签不变、永久账本不清零、每点三帧来源、唯一上报、末yaw执行、worker取消和job序号单调、剩余额度拒绝、STOP吸收、DECLARE后的PAUSE静稳、实时最终朝向复验和飞行/灯光/完成接口保持静默。

开发中的回归明确暴露并修复：下一计划拒绝前漏归档当前点；无意将所有腿末段慢速化；名义TURN与实际DRIVE bearing不一致；g2瞄准侧偏大于其名义环扫余量。曾有一轮129项中3项失败、126项通过，失败原因均为旧bearing接线；该XML保留作为历史，不作为最终通过证据。

最终验证命令：

```text
python -m pytest tests/test_ground_three_point.py tests/test_ground_three_point_plan.py -q --junitxml=runs/review/20261112_three_ground_runtime_final.xml
python -m py_compile src/inspection_mission/inspection_mission/ground_three_point.py src/inspection_mission/inspection_mission/ground_three_point_plan.py
```

另外，修复前一轮新旧关联125项通过；最后变更只针对新guard，最终结果见上述32项XML。上传后已真实运行并回传，但尚未取得平台三点计分或完整巡检完成。后续仍以平台练习的官方成绩与独立回传核验为准。

冻结SHA256：

- runtime：`6f7a7bb24f9e5269c72bafc05929085d111599a66a135f2dd3782bef5728abc2`
- planner：`d4d4ade05388de5f7435399f2092824aa291c2f1440c6b3fa3adc0941bfa6c65`
- launcher：`ef68d341bf332b9c163187d6b43e39248dbc70f2bf40b9b78804880a647b8fc6`
- v25单点仍为：`902424ca5ba3ec208a0028645d377f4ce92ff77e5ee56205c87fac95a9b349cb`
- 冻结route仍为：`5ab108c4128db8b32c18cee084fce636e8ce80700f18dd37e468006a2a58b945`
