# 短直行实现独立审查

日期：2026-11-12。只读审查 `ground_short_drive.py`、启动脚本、专项测试及其复用的 bounded/survey 观测路径，未改控制源码、启动脚本或测试。

冻结源码 SHA256：`d1e1c5b0528cc6fb2db70729f80bc17d6659c40e5a17e250755cbec055b32b4e`。

## 结果

当前补丁未发现新增阻断缺陷。实际运行 `python -m pytest tests/test_ground_short_drive.py -q`，**40 passed**。另以直接调用测试助手方式检验 DRIVE/PAUSE 两态下x=NaN、x=Inf、四元数z=NaN的六个瞬态坏样本，再正常替换，均保持STOP和原原因。这只验证本地状态机与模拟传输，真实平台动作另行验收。

审查中曾实际复现三条时间入口回归失败：传感器wall=1.15后，step/watchdog/handler使用1.14仅PAUSE而非硬停。实现代理已将预算判断改为比较 `max(last_wall,last_observation_wall)`，handler先检查预算/时间硬故障；新专项40项已覆盖修复。未为此改动老 survey 控制。

## 已核对边界

- 仅新guard控制DRIVE/PAUSE；发布适配器只向`/cmd_vel`发前向速度≤.03m/s、yaw=0，其他已声明输出保持静默。
- 初始真实yaw固定前向轴；回调测得投影≥.02m立即锁STOP，侧向位移不计完成，后续正常样本不能清除已达距离/硬故障。
- 最大真实总位移.06m、倾斜.03rad、实际线速.10m/s、实际角速.10rad/s、偏航误差>.05rad、反向投影<-.005m、非有限数值/frame错误/时间倒退均有硬停路径。
- 首次动起10s总预算含所有暂停，global25s/等待20s保持；PAUSE不改origin、yaw或首次时间。新鲜且静稳双时钟驻留≥1s后恢复，最多3次。
- `temporal_fault`和硬停都持同一RLock；并发PAUSE不能覆盖硬STOP。watchdog晚启动仍按首次动起的原deadline；ROS spin停滞时独立线程持续尝试发零。
- 组合的观测recorder始终WAIT/STOP；仅复用公开odom/image记录，不运行survey的step/watchdog、抽签绑定或转向控制。事件mode为`ground_short_drive`，各相机保留真实topic/frame/曝光与其独立odom源；旧transport section名字包含survey不代表实际做了巡航转向。
- 脚本只source环境及上传源码主入口，未启动仿真/裁判，真实子进程退出码保留。

## 仍须实测的边界

当前scan门禁为 `.447+.114+.06+.02=.641m`；起点最近北墙约.65m，仅约9mm保守门禁余量。短直行沿初始+x，最近北墙并非运动方向，但本次仍采用全环保守门禁，不放宽。若触发，仅说明该实验停下，需记录真实scan/位姿后定位，不把它归为控制已完成。

watchdog在取wall后、取得锁前若发生等待，其旧wall可能小于新回调wall，引起保守时间硬停。它不会扩大运动，但可能影响可用性；目前没有该原因的真实短直行运行证据，不据假设继续扩张老源。状态锁不提供ROS驱动实际制动保证；需以平台实际轨迹、末速度、停止延迟和原相机图验收。

2cm短直行即使通过，也只是控制层实验，不是地面识别得分、完整任务或“测评成功”。
