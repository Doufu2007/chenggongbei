# v35 真实回传只读分析

父代理于 2026-11-12 确认 59 events 回传完成后才开始读取真实输入。此目录只保存分析器、冻结源码副本和分析结果；未改生产或测试，未上传平台。

运行：

```powershell
python 'runs\analysis\20261112_three_point_axis_v35\analyze.py' --input 'runs\evidence\20261112\callback_three_point_axis_v35.jsonl' --output 'runs\analysis\20261112_three_point_axis_v35\actual' --confirmed-complete
```

`--confirmed-complete` 必须显式提供，脚本没有轮询逻辑。依赖仅 Python 标准库。执行时核验已保存的冻结 axis 源码 SHA，保留压缩包原始字节和原 JPEG。

输入 SHA256：`8db889d1496ef05f358cdaf4e003c887c56050b66f1dbf8f341ae3c5a6538fc0`，517910 bytes。上传身份：`af0fe738-c09b-4f4c-a2ed-fe51b0e0b298`，`run_20261112_084355_23ba4d48`，页面 `#92599511`。

`actual/summary.json` 含字节传输、独立 core 绑定、图像来源、prior、像素到候选 world quad、识别与声明发送、本地命令、10Hz 抽样与累计账本检查。`diagnosis_callback.json` 单独保存相机超时的时序证据和因果边界。`actual/images/` 为三个 SHA 验证后的 800×800 原 JPEG。

实际结果：首点 ground_1 在 RECOGNIZE_WAIT 因 `single_point_uav_image_stale` 停止；三帧采集、两个匹配、零识别发送、零到点声明，未进入 entry。不能写成三帧识别成功或连续三点完成。

分析器不推断裁判接受和官网分数，`official_score=null`。SIFT 未由标准库分析器重跑；仅独立核验保留的来源与候选几何。原始 ROS 像素未回传，不能从 JPEG 复算其 SHA。轨迹为保留的 10Hz 抽样，独立路径与旋转只作下界。`ground_motion_full` 未回传，axis 发布尝试序列不可见，不能把缺失解释为零发布失败。
