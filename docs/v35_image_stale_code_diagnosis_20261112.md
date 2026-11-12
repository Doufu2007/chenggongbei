# v35 图像超龄与永久停止的代码诊断

## 已确认根因

v35 已到达首个地面观察位并接受两帧一致的 `landslide` 视觉结果，但等待第三帧匹配时，**当前相机流存活**的墙钟年龄变为 `.530038013s`，源时间差变为 `.543s`，均超过既有 `.50s` 门。停止是程序正确识别超龄后的**故障分类缺口**：相机超龄没有进入可恢复 freshness 分支，而被直接锁存为永久 STOP。它不是三帧原曝光必须在 `.50s` 内完成 SIFT 的要求，也没有证据说明匹配阻塞主线程 `.53s`。

原始回传 `runs/evidence/20261112/callback_three_point_axis_v35.jsonl` 为 517,910 B，SHA256 `8db889d1496ef05f358cdaf4e003c887c56050b66f1dbf8f341ae3c5a6538fc0`。复用已有解包产物 `runs/analysis/20261112_three_point_axis_v35/actual/bundle.exact.json`，SHA256 `196d4f6abcc9f8979b9d5a46b98ecf45cb0b5a0d47250ec7759fa7c83bd99200`。已有分析确认分片传输完整、core 绑定一致；bundle 标注 `partial=true`，回传来源并非独立密码学认证。本诊断不重复整条轨迹和各安全门的完整重算。

## 实际时序

| 证据位置 | 观察 |
| --- | --- |
| 原始第 43 行 `motion_log` | `98.512614524s` 进入 `RECOGNIZE_WAIT`，零速 |
| 同一序列 | `100.760668209s` 首个 STOP，原因 `single_point_uav_image_stale` |
| 同一序列 | `102.780736888s` 变为 FINISHED，原因不变 |
| `current_point.camera_liveness` | 检查墙钟 `32292.430096402`，最后递增原图到达墙钟 `32291.900058389`，原曝光 `101.757s` |
| 同一记录 | `image_wall_age_s=.5300380129992845`，`clock_minus_image_stamp_s=.5429999999999922` |
| 首个 STOP 的 timing | odom fresh，scan fresh；scan 墙钟年龄仅 `.075196382s` |
| `current_point` | 两帧已接受，recognition=None、attempts={}、matcher_failures=[]、epoch_invalid=None、recapture_count=0 |
| `survey.approach` | 全局 `resume_count=3`，已用完原恢复额度 |

两帧匹配分别花约 `1003.899/990.053ms`；第二帧子进程完成单调墙钟为 `32292.357429252`，早于最后相机检查 `.072667150s`。未得到第三帧执行/结果的完整证据，不能将它宣称为第三帧识别错误。

## 为什么超龄会永久停止

1. `ground_single_point.py:239` 的 `_current_recognition_condition` 单独检查当前相机存活；`252–255` 返回 `single_point_uav_image_stale` 或 `single_point_current_image_clock_inconsistent`，原门为墙钟/源时间 `.50s`，原负采样容差 `-.12s`。
2. `ground_single_point.py:481` 在静止等待每次控制 tick 上执行该检查，并交给 `temporal_fault`。
3. `ground_single_point.py:262` 只在 `reason in self.FRESHNESS` 时将捕获 epoch 标为失效，进入既有重采逻辑。
4. `ground_bounded_approach.py:129` 的 FRESHNESS 集合只有控制时钟、odom、scan 等原因，**不包含两种相机原因**。`303–309` 因此走 `self.stop`。
5. `bounded_ground_probe.py:94` 锁存 STOP；`ground_observation_route.py:255` 两秒后变为 FINISHED。后续收到新图也不会解除终态。`GroundSinglePointCoordinator.poll_once` 看到不再是 RECOGNIZE_WAIT 会取消 worker context，剩余识别不能发布。

这解释了“图像恢复后仍不继续”与 `epoch_invalid=None/recapture_count=0`。普通控制 freshness 有界停车恢复；静止识别的相机流 freshness 通过另一个 reason 返回后，被意外当作不可恢复故障。

## 回调、转换和 worker 的区分

- `bounded_ground_probe.py:522–541` 的图像接收回调只保存原始 ROS 消息引用并交给 guard；它不做 `message_to_ordereddict`、CvBridge、JPEG 或 SIFT。
- `ground_single_point.py:564–596` 的原图转 JPEG 在独立 daemon 线程中执行（`600–638`），保留原曝光、真实 prior odom 和原图 hash。
- `ground_matcher_worker.py:180–192` 使用 spawn 子进程计算 SIFT；独立 reader 线程反序列化结果，`poll:218–245` 使用 `get_nowait`，不等待匹配完成。poll/提交仍由控制循环串行调用，但等待计算没有占住 ROS 执行线程。
- `bounded_ground_probe.py:771–801` 确实以 `spin_once→tick` 串行调度，图查询也在 tick 中，因此 executor 排队是理论可能项；但本局计时不支持它发生了 `.53s` 长阻塞。

| 实际计时 | 最大值 s |
| --- | ---: |
| tick 总墙钟 | .008013284 |
| spin_once 总墙钟（含等待/回调） | .055359639 |
| UAV 图像回调墙钟 | .029126463 |
| UAV 图像回调本线程 CPU | .000290209 |
| 单次描述性图查询 | .000213803 |
| UAV 图像到达间隔 | .696315488 |
| UAV 图像原曝光间隔 | .668 |
| 地面图像到达间隔 | .383246147 |
| 地面图像原曝光间隔 | .336 |

299 帧 UAV 图像对比302帧地面图像，且 UAV 原曝光间隔曾约为正常 `.333s` 的两倍，支持存在流级跳帧/送达空窗。由于只保存汇总和有限曝光，不能将最大 gap 精确归到停止那一次，也不能定位源相机、公开桥接、DDS 或执行器排队中的具体层。接收入口年龄本身混合这些延迟，现有观测不能拆开它们。

## 最小可验证修复

不改变 `.50s` 相机门、`.20s` 控制门、`.05s` 曝光真实 prior 配对、`[-.12,.50]s` 源龄合同与所有几何/速度/累计预算。仅在 CAPTURE/RECOGNIZE_WAIT、识别尚未 attempt、原 header 合法且时间方向正常时，将暂时相机墙钟超龄或源时间正向超龄转换为既有零速 PAUSE；等待实时相机恢复及源时间/墙钟重新静稳，再开启新 epoch 重采三张。旧 worker context 在 PAUSE 失效，旧结果不得参与新 epoch。

非法帧、时间倒退、源龄低于 `-.12s`、REPORT_WAIT 或任意识别 attempt 之后仍永久停止；不得重报。每点最多一次重采、全局最多三次恢复与原跨点累计预算保持。记录恢复原因、墙钟/源时间、epoch、已用额度和结果。

**v35 已经用完三次恢复额度。** 即使修复分类，也不能让相同完整时序无限恢复或承诺本局完成；需要分别验证“还有额度时可恢复”和“额度耗尽时仍停”。原门在 `.530s` 报超龄本身不需要放宽。

修复应独立新增模块与启动器，冻结 v35 与父模块保持原字节。ROS 无法在本机真实运行，因此实际节点 factory 用已有 ROS 传输测试替身验证新 guard 从构造时接通；最终仍需独立审查与平台运行。
