# 图像worker原型：独立只读审查

日期：2026-11-12。对象仅为 `runs/analysis/20261112_ground_matcher_worker_dev/` 新开发原型。未改 `src/`、生产测试、控制脚本、原平台回传或实现代理输出；独立反例位于 `runs/review/20261112_matcher_worker/`。

## 实际发现与修复

原型最初只在收到结果时判断context。独立反例先在epoch1缓存 `latest_result(matches_current_context=true)`，再调用 `set_context(...,epoch2)`；缓存结果的标记仍true，旧epoch结果可能被读取为“当前”。实际 **1 failed/4 passed**，保留 `independent_initial.xml`。实现代理在context变化后重算缓存结果的两个context标记；本审查未自行修改实现。

最终源码SHA256 `c31e007852bad751f9ec2c9393c56835b0d1da904f3c818fc538c9ccc5335da5`。独立最终 **8 passed in .39s**，结果 `independent_final_v2.xml`，覆盖缓存epoch失效、错误context不能消费序号或替换pending、结果始终非识别/非新鲜/非官方、未来pose/nonzero distortion拒绝、K与CameraInfo冲突拒绝、800×800 JPEG header门。旧重跑曾因独立fixture缺新必填CameraInfo而失败，属于审查fixture合同过时；保留旧XML，另存新fixture和新绿结果，未冒充生产缺陷。

```text
python -m pytest -q runs/review/20261112_matcher_worker/review_worker_cases_final.py --junitxml=runs/review/20261112_matcher_worker/independent_final_v2.xml
```

## 有界与上下文的准确含义

主owner最多保留1个执行中Future、1个可覆盖pending、1个latest_result；只在当前Future结束后才dispatch下一项，不为每个新帧追加executor任务。pending被新帧或context变化覆盖时有计数。已运行旧epoch任务没有被强制中断：它可正常完成并保留诊断，但收取时context=false。context改变也重新失效已缓存latest_result。这个行为是丢弃/失效旧工作，不是杀死运行中的子进程。

输入压缩JPEG限2MiB、元数据限64KiB，子进程在OpenCV全图decode前以header明确只接受800×800 JPEG。固定12模板和800×800输入限制了当前匹配规模，不能将这套槽位合同宣称为跨任意模板清单的固定RAM上界。metadata深拷贝和子进程任务回显保留同一job_seq/session/firstID/epoch/JPEG SHA/stamp/pose/K；K与原CameraInfo必须一致。这里验证字段一致性，并不认证调用方提供的元数据真伪、传感器时间新鲜性、官方类别或任务ID。

所有结果 `official_acceptance=false`、`fresh_exposure_validated=false`、`eligible_for_recognition=false`。即使context=true，仍明确不是新曝光或可上报许可；调用者不得用计算完成时间刷新原图年龄。相同旧图重做两次只是机制测试，不是两帧独立识别证据。

## 子进程与20Hz声明

实现采用`multiprocessing.get_context('spawn')`及1个`ProcessPoolExecutor` worker，SIFT/BEV在子进程运行，不执行在主owner的Python解释器/GIL中。主owner仍承担提交时SHA/JSON/deepcopy、结果反序列化/深拷贝与OS调度，因此只可报告实际观测心跳，不能宣称绝不阻塞、硬实时或平台ROS执行器的20Hz保证。

`close()`使用`shutdown(wait=True)`，会等待运行任务，只适用于停止控制后的shutdown；不应放进控制tick。异常任务被结果错误包捕获，没有数据伪装成识别成功；父进程崩溃监督、子进程卡死强杀与重启仍未实现，原型文档明确限制。没有ROS导入、控制publisher或裁判输出。

## 旧离线benchmark复算

原 `benchmark_report.json`/`heartbeat_ticks.csv` 实际记录276tick、有效 **19.999364Hz**、最大间隔 **55.5878ms**、P99 **54.189482ms**、0个跳过调度；逐CSV独立复算与报告一致。主PID32396、workerPID29204不同；实际执行job1/4，job2/3被pending覆盖；旧epoch1结果current=false，epoch2结果current=true。两个真实SIFT job分别约 **2.576s/3.024s**，冷模板加载约7.438s。保存了每job12条完整模板诊断与来源，均没有接纳候选。

这份旧报告没有worker源码hash，并早于metadata/缓存epoch修补，**不能将旧测时挂到上述最终冻结源码**。独立算术为 `runs/review/20261112_matcher_worker/benchmark_independent_arithmetic.json`，输入hash保留。最终版本另名保留如下，没有覆盖旧证据。

## 最终冻结benchmark复核通过

`benchmark_report_final.json`记录worker源码SHA=`c31e007852bad751f9ec2c9393c56835b0d1da904f3c818fc538c9ccc5335da5`、benchmark源码SHA=`2311a89c02c3ef35fcb40dd0670af9da0ad40a228640d5a9b4c0be738c1291b8`，独立核对与实际文件一致。最终report SHA256=`cd3b5939612887da83fde3e168aafb19f3c017c664853f5daf136ce74982d150`，最终CSV SHA256=`e0f0a5da8e8db49bca0941b71db0b90fcdfda5c60767cabf55d203e4171ddde7`。

`recompute_benchmark_final.py`逐行重算289tick，**19.999947Hz**，max间隔 **51.555600ms**、P99 **50.933944ms**、median **50.01155ms**、0skip；owner单tick工作max **13.857600ms**，P99 **1.292768ms**。所有数值与冻结报告相符。主PID23932、workerPID29024不同；只执行1/4，2/3覆盖；旧epoch1为context=false，job4为true，但两个job的fresh/recognition/official flags全部false。完整job计算约3.123s/2.691s，计算完成不会刷新65.379s原曝光。

机读独立复核为 `runs/review/20261112_matcher_worker/benchmark_final_independent_arithmetic.json`。最终结论：本次发现的缓存epoch缺口已修复，当前独立8项检查与冻结本地benchmark算术均通过；没有发现新增阻断原型离线使用的问题。该结论只适用于本地原型，ROS锁、真实平台资源和控制循环仍未接入验证，不能称作已上线20Hz识别、正式识别正确或有效得分。
