# v35 连续三点 axis 真实回传分析

v35 停在首点 ground_1 的 `RECOGNIZE_WAIT`，原因为 `single_point_uav_image_stale`。两段直行及最终观测朝向均已完成，三帧源图已采集，但仅有两份被接受的 matcher 结果；识别与到点声明均未尝试，未进入 entry，归档完成点数为 0。官网成绩由主代理另核，本报告不推断分数或裁判接受。

提交身份：`af0fe738-c09b-4f4c-a2ed-fe51b0e0b298`，run `run_20261112_084355_23ba4d48`，页面 `#92599511`。抽签为 `ground_1 → ground_3 → ground_2`。冻结 axis 源码 SHA256 为 `e033c119790ad8a32a066cd21cd9fd841e5b967092b8d0ac0355fd5d5f602f76`，分析目录保存了相同 SHA 的源码副本。

输入为 `runs/evidence/20261112/callback_three_point_axis_v35.jsonl`，517910 bytes，SHA256 `8db889d1496ef05f358cdaf4e003c887c56050b66f1dbf8f341ae3c5a6538fc0`。确认完整后才读取。7/7 压缩分片、压缩及原文双 SHA/长度、gzip 单流、session、嵌套 axis revision/planner、独立 three_point_core 与包内状态绑定均通过；无重复分片。外层 revision 沿用父类 `ground_three_point_v1`，包内真实 revision 为 `ground_three_point_axis_v1`。

## 停止窗口

`RECOGNIZE_WAIT` 在运行 elapsed 98.512614524s 进入；STOP 在 100.760668209s，窗口 2.248053685s。以下 elapsed 为 guard 的墙钟；source 为 ROS 图像时间，不混作同一时间轴。

|证据|source / monotonic|arrival elapsed / 时长|
|---|---|---|
|capture_0|99.423s|97.821431974s|
|capture_1|99.757s|98.156770838s|
|capture_2|100.089s|98.488834642s|
|worker job 0|32290.285492236 → 32291.293262795|执行 1.007770559s；decode 3.870ms；match 1003.899ms|
|worker job 1|32291.364142520 → 32292.357429252|执行 0.993286732s；decode 3.232ms；match 990.053ms|
|最后用于活性检查的新图|source 101.757s；arrival 32291.900058389|检查时墙龄 0.530038013s|
|活性检查|monotonic 32292.430096402|clock-image 0.543s|

最后采集帧到达后 23.780ms 进入识别。接受时刻本身未被独立记录：依照保留结果与串行 coordinator，job 0 被接受位于其 worker finish 与 job 1 start 之间，区间宽度最多 70.880ms；job 1 已被接受，其 finish 距失败检查仅 72.667ms。没有第三份被接受结果；不能由前两帧推造第三次执行结果或精确 dispatch/accept 时间。

直接触发为新 UAV 图像墙龄 530.038ms 超过既定 500ms 门限；源码先检查墙龄，因此 reason 为 image_stale。source 落后也达 543ms。STOP 当刻 clock/odom/scan 墙龄分别为 32.307/12.021/75.196ms，均 fresh。全局入口间隔最大值为 odom 69.165ms、UAV odom 69.244ms、scan 196.376ms、clock 205.776ms，故不支持四流共同半秒停顿。UAV 图像全局最大入口间隔 696.315ms、最大 source header 跨度 668ms；地面图像最大入口间隔 383.246ms。聚合统计未保存这些最大间隔的具体发生时间，不能把其精确定位到失败窗口。

主 tick 全局最大 8.013ms，spin_once 最大 55.360ms。等待窗口保留 89 次归零 publish 返回、0 次非零。matcher 在独立进程每帧约 1s 串行执行，增加了识别等待时长；现有证据不支持它在 ROS 主线程独占半秒。也无法进一步区分图像源端、bridge、DDS、executor take 或调度延迟，不能声称已证明 CPU 争用或网络根因。UAV 图像随后继续到 source 104.089s，说明最终证据中并非永久停流。

## 视觉来源与几何

三个 800×800 JPEG 完整分片与 SHA 均通过，大小分别为 29874/29819/29885 bytes；三个 exposure 的 prior UAV odom 时间差分别为 3/17/9ms，均为非未来且不超过 50ms。三张图片均已实际查看，首点前景为相同滑坡/山体裸土照片，画面基本稳定；这只是图片观察，不替代裁判语义。

两个接受结果分别绑定独立 job_seq 0/1、独立 source stamp、同一 session/ground_1/stationary_epoch 2，worker JPEG SHA 与回传 RGB 源 JPEG 一致，job pose 与 exposure prior 完全一致。原始 800×800 CameraInfo/K、frame/topic 与 prior 绑定通过；两份模板 SHA/来源均为冻结的 `landslide_01.jpg`，class 为 landslide。

独立用 source_quad、K、候选外参（相机 x=0.390m、高=0.173m）与原 pose 投影到 world，两个 quad 最大误差分别为 0.247/0.158 微米；候选中心距离 ground_1 中心分别 31.951/31.933mm，均在 80mm 唯一板区绑定门内，两中心距离约 0.036mm，远低于 30mm 一致门。此为候选几何一致性，不代表候选外参或 ID 映射已被官方确认。

三个采集前的保留 10Hz 样本分别有至少 1.20/1.50/1.82s 的源时间静稳段，墙钟跨度分别 1.223/1.527/1.856s；对应区间已返回命令均零。抽样不能复原每一次高频停稳判定。第三张源图有 prior 和字节证据，但无其被接受的 job/result，不能补推其 job calibration 或识别结果。标准库审计器没有重跑 SIFT；原始 ROS pixels 未回传，不能从 JPEG 独立复算 raw ROS SHA。

## 运动与账本

5 项观测 operation 均正常结束，尚无 entry operation。两段直行的独立端点误差分别为 1.093386mm 和 5.300271mm，均低于 20mm。零请求后累计滑停路径分别为 0.599719mm 和 0.095993mm，均低于 10mm。最终观测位置为约 `(0.589174,-0.495942)`，距规划观测位置 `(0.6,-0.5)` 约 11.56mm，朝向误差约 0.02437rad；这不是任务区到点声明位置。

真实高频账本累计路径 1.113765653m、绝对旋转 2.169321339rad。operation 起止账本单调并衔接，未发现重置；最后 operation 结束后额外路径约 0.084611mm。由于首点未完成，本局没有跨点 handoff 实测证据。

保留轨迹 994 条、命令 2348 条，二者 dropped 均为 0。首次活动状态之后的 10Hz 抽样独立复算路径下界 1.109743695m、旋转下界 2.166678277rad，均不超过高频账本。1699 条非零返回命令，0 条负 X，输出幅值/状态/同时直行转动违规为 0；本局未走到倒车段，不能作新的倒车验证。

识别和声明的 point attempts 均为空，且各 endpoint 的 attempted/returned 均为 0，独立一致。`ground_motion_full` 因体积走分节回传，axis_publish_attempts 序列未被该 fallback 携带，因此发布失败数为 unknown，不能写“0 次发布失败”。`publisher.publish()` 返回也不表示执行器收到或裁判接受。

原始 bundle、SHA、三张 JPEG、轨迹/命令/operation CSV、详细时序和审计字段保存于 `runs/analysis/20261112_three_point_axis_v35/`。分析器完成 11 项内存负向自检（旧 mode、缺片、改字节、SHA/长度、嵌套版本、core 冲突、重复键、非有限数、图像缺片等），均通过；这些是审计器检查，不是平台结果，也没有改写项目测试。此次只读分析未改生产代码或测试，未再次上传。
