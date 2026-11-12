# 地面反向输入响应独立实现

最新状态：v30 已按修后源码 `27eccf892ffa33b2af2e7cc53460fd8fa6ad4cf6830bc4572c631175c19e00ae` 完成平台练习。此前独审发现最终 DDS 查询可跨过脉冲期限；主控先运行实际接缝反例得到 RED，再新增本模块独立 tick，在最后 DDS 返回后核查预算、源龄、阶段，反例 GREEN，33 项原反向专项通过，独立复核通过。冻结 v28 未改。

真实平台 ID `e5d26785-01cd-41a6-acb8-02098e088649`，runId `run_20261112_080340_fad7c927`，编号 `#11416510`，终态 failed / score=null / 38.644s。本队程序在原点正转到约 +.119963rad 后因 scan_too_close 停止，未发负 X 脉冲；因此倒车响应仍未验证。原始数据与解码在 `runs/analysis/20261112_reverse_v30/`，台账 `runs/records/reverse_v30_e5d26785.json`。下文保留首次实现阶段记录，后续不得用旧源 SHA 打包。

已完成最小独立反向输入实验和本地相关检查，保持已审 v28 源码及平台证据冻结。该模块尚待独立审查和真实平台练习；没有把负 X 请求解释为已实现反向位移。

## 实现

新增 `src/inspection_mission/inspection_mission/ground_reverse_response.py`、`scripts/run_ground_reverse_response.sh` 与 `tests/test_ground_reverse_response.py`。规格、计划分别在 `docs/superpowers/specs/2026-11-12-ground-reverse-response-design.md` 与 `docs/superpowers/plans/2026-11-12-ground-reverse-response.md`。

`ReverseLimits` 继承冻结的 `DirectionLimits`，只将累计绝对旋转上界设为 2rad；其余字段连同耦合停止预留交原验证器逐一验证，防止扩大 v28 已审合同。目标 yaw=+pi/2，请求序列为 `(-0.03,0)`、`(-0.10,0)`、`(-0.10,0)` m/s。两次高档请求是独立段，分别归零、实测静稳并重新记录原始起终姿态。

新 guard 覆写 `_enter_phase("TURN")`，在原状态机进入转向时设置 +pi/2 目标及相对真实当前 yaw 的 `initial_turn_error`；其余状态机和停止检查复用。新节点 factory 先创建已审 direction 适配器，再在 spin 前显式替换 guard、拷贝实际 endpoint/匹配状态并更新 `experiment_mode`。保留最初节点创建的墙钟原点，不因替换延后全局预算。真实订阅、tick 与发布器每次读取 `node.guard`，没有全局 monkeypatch。

仍保留转向 45s、全局 120s、每段 2s、实测位移 0.010m 请求零、0.04m 观测硬停、0.03m 停止预留、归零后累计路径 0.01m、段间源时间与墙钟各至少 1s 静稳、静稳漂移 0.002m、累计路径 0.18m、距永久起点 0.12m，以及全部源龄、严格前序配对、完整有限环扫和实时 DDS 门。硬界是软件观测与请求预算，未证明调度时延和物理停止保证。

输出仍只有 `/cmd_vel`；其他 publisher 始终静默，负 X publish 失败会独立尝试零、锁停，DDS 在 step 后丢失时也禁止非零发布。原始 pose/header/twist、真实命令和逐段响应仍使用已审 direction 的 gzip/schema 合同，mode 区分 `ground_reverse_response`，核心增加 `reverse_contract`，明确 `actual_reverse_motion_confirmed=False`。启动器为 LF，只启动本队节点。

## 验证证据

30 项最初专项在模块未实现时得到缺失模块的预期 RED：`runs/review/ground_reverse_20261112/red.xml`。实现后 29 项通过，剩余一项测试夹具将世界 X 注入恰好 0.010m，同时又积分了真实负向 body 运动，使实际净位移略低于 0.010m；将该注入改为 0.0101m，未为此修改生产阈值。

新增另两个限值边界及 factory 创建墙钟保留检查后，反向专项共 **33 项**。最终反向专项、冻结 v28 方向专项、严格同步、有界地面 ROS 传输与遥测相关检查合计 **172 passed**，无失败、跳过或错误，证据为 `runs/review/ground_reverse_20261112/final_related.xml`。按主控要求仅运行这些必要相关检查，未重复整个仓库测试。

真实 callbacks + `node.tick` 的专项使用完整 ROS 消息形状，实测正向转到 +pi/2 附近后产生三个负 X 阶段；发布命令和记录逐条相等，未发布 Y、负向转角、UAV 或计分消息。另覆盖 publish 失败归零、DDS 发布前丢失、严格前序 scan 配对、.010m 请求零、归零后累计路径锁停、.04m/2rad/45s/120s 界与 STOP 吸收。

`py_compile` 通过，已安装 `C:/Program Files/Git/bin/bash.exe -n scripts/run_ground_reverse_response.sh` 退出码 0。合成反馈只验证接线与约束，不是平台反向能力证明。

源码 SHA256：`7954b7e4a15c58e3416e9510fd88129f5632fd7f90a7d8c6ddfde553b2d59d83`。

脚本 SHA256：`2beedbed0c9d288281c9c746369127e760c3981a8a8cfd0618203c1d23f86153`。

冻结 v28 源码 SHA256 仍为 `aeecaa1a50b585cfdd13218a94d56283f0ea93f03a71151475512e852bb21946`。未修改 route/single/three/vertical。

## 下一步

交主控安排独立审查，审查通过后由主控打包并进行有限真实练习。真实负方向位移、停止后轨迹及稳定性需要平台源时间位姿证明；未验证之前不接入 g2 后的反向轴向计分路线。本子任务没有打包、上传或调用网站。
