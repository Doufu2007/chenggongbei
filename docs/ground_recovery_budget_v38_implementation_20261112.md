# v38 连续地面恢复资源实现

新增独立 `ground_three_point_recovery_budget.py`，继承冻结v36相机恢复guard。在构造时经v36受检factory创建，节点和coordinator共用该对象。冻结v35和v36及其父模块均未修改。启动器为 `scripts/run_ground_three_point_recovery_budget.sh`。

预先固定资源合同为32次成功恢复、单段PAUSE最多5秒、所有PAUSE累计最多60秒，原创建起1200秒不重置。新层只接管PAUSE，先过原预算、观测运动、新鲜度与双时钟停稳，再递增真实resume_count；恢复当tick仍为零，下个tick重新通过原发布门。未更改相机0.50秒、控制0.20秒、原prior0.05秒、负采样差0.12秒、净空、速度、累计路径或转角。

相机重采仍每点一次，且计入全局32次；重采启新epoch，旧worker结果不能复用。REPORT_WAIT或已attempt后由v36停止，不新增重试。单次与累计PAUSE预算进入 `_budget_fault`，因此watchdog、活动callback与最终发布前复核同样生效；重复故障不会重开单段计时。最多40段记录覆盖32次成功恢复及最终失败段，事件显式报告实际上限32，父层旧上限文案被覆盖。

专项先以冻结v36运行，5失败/1通过；实现后9/9通过。检查覆盖第4次真实恢复、32次后拒绝第33次、单段与累计墙钟、watchdog、每点重采耗尽、旧全局deadline、实际factory和最终DDS查询后相机变旧不得发布。相关v36、axis与新专项合计58/58通过。Git Bash `bash -n` 通过。仅本地验证，独立审查及真实平台结果另记。

其中32次测试初次假定PAUSE内phase不会变；实际父guard接收odom会将已对准TURN改为TURN_SETTLE，该transition仍保留PAUSE。测试已据真实接线核对恢复当tick仍零并恢复测量phase，不是强制固定旧phase。

冻结源码SHA256 `f50c7058d8a6727ccac83d03c228d6a2c9e01249c55171aa55758a0b272cdc1d`，原新启动器 `ae32f4c32c20b1071c0d2eb7426f50c1748a9b9cb415d4547d2662f964fd24ce`。候选包以v35为基底，50项逐字节相同，只加入v36/v38模块，清单见 `runs/review/ground_recovery_budget_v38/package_manifest.json`。临时回传URL只在运行副本，不写入本文。

本资源合同来自有限实测和有界工程余量，不保证连续三点成功率或完整巡检得分；耗尽后不得在同局扩容。
