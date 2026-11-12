# 地面反向输入响应独立复核

2026-11-12。**最新结论：发布边界修复定点复核通过，可按固定协议打包 v30 有限反向请求实测。** 本 reviewer 未上传或改生产源码；以下旧 SHA 阻断和原红结果保留为历史。

## 修复后的独立复核

新源码SHA `27eccf892ffa33b2af2e7cc53460fd8fa6ad4cf6830bc4572c631175c19e00ae`；脚本保持 `2beedbed0c9d288281c9c746369127e760c3981a8a8cfd0618203c1d23f86153`。实核v28源仍为 `aeecaa1a50b585cfdd13218a94d56283f0ea93f03a71151475512e852bb21946`。

新独立 `publish_boundary_recheck.py` 和 `publish_boundary_recheck_metrics.json` 位于原独审目录，旧红metrics未覆盖。四项检查通过：

1. 同一原反例、同样有效源龄`.07s`，最后DDS返回 `28.57s` 跨 `28.55s` 脉冲期限后，phase立即转 `PULSE_SETTLE`，实际请求和记录都只为零。
2. 第二次DDS查询异常，锁STOP并成功发送零；其错误被原 `check_command_match` 转换为无匹配，不能继续负值。
3. 第二次DDS查询造成源龄`.26s`，当前请求改为零并PAUSE；没有跳过新鲜门。
4. 正常新factory→真实callbacks→替换后的tick→实际publisher 合成通路再次完成三个脉冲，648条请求/54条负值和证据逐条相同，yaw目标与initial_error仍pi/2，其余通道静默。

补丁在新reverse模块中显式接入独立tick，最后DDS后重新核预算、观测运动、源龄及phase与待发决策的兼容性。旧v28未修改；修复无动作或阈值放宽。放行只对应软件固定协议和有限实测准备，不证明平台真实反向位移、硬实时停止或全任务计分。

冻结候选源码 `7954b7e4a15c58e3416e9510fd88129f5632fd7f90a7d8c6ddfde553b2d59d83`；脚本 `2beedbed0c9d288281c9c746369127e760c3981a8a8cfd0618203c1d23f86153`，均已实核。

独立脚本 `runs/review/ground_reverse_independent_20261112/independent_probe.py`；结果 `independent_metrics.json`。没有重复作者172项关联检查。

## 正常接线通过

实际新 `create_reverse_response_node` →原订阅callbacks→实际tick→实际publisher 路径，在独立合成反馈中完成正向转到 yaw约+pi/2、三个负X请求和各自归零静稳。648条实际请求与证据逐条相等，54条负X请求，Y及其他通道全部静默；target与initial_turn_error均为pi/2，累计绝对转角1.547271808rad。两次高档段的静稳和原始独立起终姿态均有记录。限值对象相对v28默认仅旋转项改变为2rad。

这些是模拟ROS transport中的接线和软件约束验证，不能代替平台反向位移证据。

## 阻断：成功 DDS 查询跨越两秒期限

独立反例先真实运行到第一负X脉冲，再持续提交有效前进源stamp、静止位姿和完整scan，隔离“执行器对负X不响应”这一合法待测场景。脉冲起始墙钟26.55s，两秒上界28.55s。

新tick在28.52s进入；第一次DDS匹配查询及时返回，guard step仍合法。发布前第二次DDS匹配查询成功，但耗时到28.57s返回，此时输入源龄只有.07s，仍满足.20s新鲜门；继承tick随后只检查 `_condition` 和观测运动，没有再核脉冲预算。实际 `/cmd_vel` 发送了 `(-.03,0,0)`，成功记录墙钟28.57s，state/phase仍为PULSE。

这是“调用已返回后仍有检查机会却漏检”的软件边界缺口，独立于硬实时制动保证。需在最后DDS查询返回后、非零publish之前，重新检查最新预算及phase是否仍允许待发请求，跨期只能归零。保留v28历史冻结；可在新reverse适配层补接线，但修后需新SHA与同一反例定点复核。匹配查询或底层publish阻塞期间仍无法承诺硬实时归零，应继续保留该物理/调度局限。
