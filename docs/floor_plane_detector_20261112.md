# 飞行小板的曝光姿态 IPM 识别

`floor_plane_detector.py` 是独立的公开几何候选适配器。它将实际 BGR、CameraInfo K 和曝光前 50 ms 内的原始 `/uav/odom` 完整姿态映射为米制 BEV，再调用原 `TemplateMatcher.detect`。原 ORB 特征数、ratio、唯一 query、12 内点、0.55 内点比、参考支持覆盖、凸性、可见四角与跨类别歧义拒识均未修改。

缓存准备只对每张显式公开参考做 `INTER_AREA` 最长边 150/300 缩小、原 `_normalise` 和明确的水平反射变换；没有生成类别或点位答案。最终版本先对全部十参考及全部类别执行 150 尺寸完整检测；仅在这次完整检测无候选时，才执行 300 尺寸的全部参考与全部类别检测。每次均继承原跨类别冲突拒绝与同类表面合并，不因先找到某一类而跳过其余类别。输出保留原参考 SHA256、来源与派生版本，类别只能来自参考匹配。

IPM 使用完整 roll/pitch/yaw、实际 K、公开候选相机 offset `(0.390,0,0.130)` 与 optical quaternion `(-0.5,0.5,-0.5,0.5)`，世界板面 `z=0.012`。相对机体水平 forward `[1.43,2.23]`、left `[-0.4,0.4]` 是不含点名和类别的固定观察窗（480×480，600 px/m），绝不由检测四角或地图类别回填。非零畸变、错 frame、陈旧或未来姿态均拒绝。

额外物理门采用公开小板 `.35×.30 m`：面积 `.08–.13 m²`，两组对边平均边长分别距 `.30/.35 m` 不超过 20%。该合同独立于地面大板 `.15–.30 m²`，没有修改地面门。候选相机外参、尺寸与这些容差尚未被平台测量标定；公开 `.35×.30 m` 与用户口述 20 cm 规则的差异仍需实测确认。

所有中心和四角通过 `bev_to_source` 返回原相机像素。归一化中心由原图宽高计算；曝光 stamp 不重写，下游继续用原 K 与 prior 做独立 `localize_floor_pixel`，再按运行点几何约束关联。检测器本身没有任务 point ID、AIR_CENTERS、控制、ROS 或官方识别接口。

方向控制证据：`runs/review/full_flight_independent_20261112/uv_corrected_probe.py` 与 `uv_corrected_metrics.json` 排除初始单轴镜像合成错误，正向 UV 的十参考原画面生产 ORB 仍为空。独立 `bev_probe.py` 显示 IPM+缩小参考保留原门能通过；早期 `synthetic_*` 的 UV 含镜像，只作方向排查，不能据它单独归因视角。

专项 `tests/test_floor_plane_detector.py` 最终 **14/14 通过**（16.21 s），从公开参考各自独立合成几何，不使用隐藏纹理分配。十张正向曝光各只检出自身真实类别，原像素中心经生产定位回到公开几何位置；另覆盖反射与旋转、错物理尺寸、空图、陈旧/未来曝光、畸变、错 frame、同图异类歧义，以及独立矩阵的 roll/pitch/yaw/K 逆投影正控。日志 `floor_detector_fast150_fallback300_green.txt`；单独只保留 150 时原旋转正控 13 绿 1 红的证据 `floor_detector_single150_green.txt` 仍保留，增加完整 300 回退后恢复原 14 项。这些都是公开几何离线合成验证，不代表平台识别、照射或整轮完成。

非作者 ground_next_gate 对冻结生产 `c76cae66211925184bb8d921275aad7ae887275a66b63495255a5cdcd42b8cba` 的矩阵、逆投影、真实源 hash 未发现阻断；独立扩展见 `floor_independent_metrics.json`：80 个合成变体中 59 个检出、0 错类，已检出者最大定位误差约 7.3 mm。正常 0°/180°及镜像四组 40/40 检出；将纹理先旋转 90°/270°再强制拉伸回同一 `.35×.30 m` 长方板的额外变体有 21 个漏检。因而不能声称任意纹理方向均可识别，也不能将合成正控换算成平台成功率。

## 最小性能修复与时序边界

最终生产 SHA256 `a0ae1f6eb8e9950e69eb0e4a7bceffef32abeb1809c51901b6266b2bdbcdafd9`，派生版本 `orb_floor_ipm_v3:long150_then300_if_no_candidate:horizontal_mirror`。先前同时比较双尺度使单帧耗时接近 0.5 s，会碰到飞行 detection timeout 和 recognition stable 的时序预算。固定 spec0 公开几何合成、同主机、先完成加载后顺序测量：原版 warm 均值 0.469 s／最大 0.496 s；最终 fastpath warm 四次为 0.197/0.182/0.191/0.188 s，均值 0.189 s／最大 0.197 s（约 2.48 倍加速），首次检测 0.225 s。原始测量见 `floor_performance_before.json` 与 `floor_performance_fast150_fallback300.json`，脚本 `floor_performance_probe.py`。

冷加载仍约 2.807 s；异步 worker 的首个结果可能已经过期，应由原接受时序诚实拒绝，并继续接收新的原曝光。axis 与 ground 的真实 monotonic／推进 source 时钟 factory 正控已验证同一正常 150 fastpath：丢弃冷首帧，后续 warm 约 0.2–0.3 s，可完成稳定识别并只上报一次。曝光 stamp、原 prior、0.5 s detection timeout、0.5 s recognition stable 与原定位门均不因优化重写。300 回退可能较慢，不能保证每个额外姿态都在 0.5 s 预算内，也不保证平台硬实时；此处未增设更大矩阵或泛化宣称。
