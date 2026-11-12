# 独立轴向三地面点运行器实现

新增 `ground_three_point_axis.py`、LF启动脚本 `run_ground_three_point_axis.sh`、27项专项测试。未修改冻结v25/v26、原planner或已审axis planner，未打包、未上传。

源码 SHA256：`e033c119790ad8a32a066cd21cd9fd841e5b967092b8d0ac0355fd5d5f602f76`；脚本：`3b5251e1f4b5dcc8d48d23c59c6ce102b422af1e2c031fa5ae05f0e0032bdfd4`。

## 具体接线

新 `GroundThreePointAxisGuard` 继承v26，显式覆写 `_plan_point`、`_begin_entry`、`_handoff` 三个planner调用点，全部使用冻结axis planner。当前图像、三帧同epoch匹配、识别发送成功、真实停稳入区与到点声明门沿用原实现，没有新造目标类别或计分接口。

`DRIVE_REVERSE_TO_XY`始终保留正目标距离与机体朝向。`drive_yaw=atan2(destination-current)+pi`；仅本反向段将测量投影取负，原heading门仍检查机体朝向，原cross-track绝对界不改。请求幅度仍按原.03/.10、首次慢速/停止标定、最后.1m减速；只在反向段取负。每次开始操作先清reverse flag，TURN、空操作、STOP与下一forward都不能继承负号。TURN和TURN_SETTLE两处对后继reverse都重新瞄准机体朝向，保留转弧安全、转向符号与原锚门。

新 `AxisThreePointLimits` 是预先固定的新合同：rotation28rad；path12.5m、origin3m、global1200s、turn累计700s/单45s、drive累计500s；其他原界不变。原 `ThreePointLimits` 仍25rad，旧源SHA保持冻结。当前局不能增大合同或在handoff重置累计账本。

`create_axis_three_point_node`直接以`create_bounded_node`的guard factory创建新guard，复用原订阅、QoS、命令publisher、callback计时和watchdog，独立重建tick、`GroundSinglePointCoordinator`与真实String发送计数，不再依赖旧`original_tick`发布。全部描述性graph查询完成后，最后再次读取实际DDS匹配；返回后在命令/guard锁内用当前wall重新step，并复核预算、观察到的运动异常、fresh、phase、有限幅度与禁止同时平移转向。publish异常吸收STOP并独立尝试零；零publish失败被记录，后续tick/watchdog仍能再次零attempt。attempt证据上限1200并保留首条，成功返回才记实际command和成功计数。

真实coordinator→结果接收→dispatch→String publisher仍与同一个guard接线，当前epoch识别仅发一次，attempted先于publish增加、returned仅成功返回后增加。refresh不会清零计数。实际callback→tick→publish专项证明有符号负X经过实际适配代码发布，command evidence记录同一负值；这在模拟ROS transport下验证，不是官方平台运动证据。

## 测试与修正记录

最初15项均因新模块缺失按预期RED。第一版limits以dataclass默认`target_distance=.2m`误作段长上界，导致14项失败、1项通过；按冻结v26原有**3m**段长上界修正，没有放宽原合同。第二轮10通过、5失败，均为新测试夹具问题：图像matcher fixture固定session与新run session不一致；空操作并非处于观察位却要求CAPTURE；在活动reverse中先注入90度姿态变化再要求开始转向。修正夹具的会话/准入语境后15项通过。

补5项检查：跨PAUSE/STOP的永久总/组预算、反向归零后累计往返路径、core/gzip接收合同、脚本成功及17真实退出码。追加的evidence断言最初误读`current_point.target_id`，实际既有schema在`survey.approach.three_point.target_id`；按真实合同修正断言。脚本测试先missing launcher RED，再生成新LF脚本。

初版20项专项与关联共172项通过后交独审，独审发现两项适配器阻断：非`/cmd_vel`图查询期间DDS从1变0，旧缓存仍允许负X；负Xpublish抛错会逃逸，guard仍DRIVE且未独立归零。作者原样运行独审probe确认两项RED，旧源码和metrics保存在`runs/review/ground_three_point_axis_independent_20261112/before_adapter_fix/`。

新增六项回归先RED（6失败）：later graph失联、负publish失败、最终match查询后fresh过期、最终match查询后总预算耗尽、最终match查询后phase变STOP、一次零publish失败后仍可再次零。修复独立adapter后这六项及原实际callback专项7/7通过；补真实coordinator/dispatch/counts链路1/1通过。原独审probe文件没有修改，重跑5/5通过，包括另一三点顺序`ground_1→ground_2→ground_4`。该结果仍由原独审人复核，不以作者重跑替代最终独审。

初版运行（历史，非当前冻结结果）：

```text
python -m pytest -q tests/test_ground_three_point_axis.py tests/test_ground_three_point.py tests/test_ground_three_point_axis_plan.py tests/test_ground_three_point_plan.py tests/test_ground_single_point.py tests/test_diagnostic_receiver.py tests/test_probe_telemetry.py --junitxml=runs/review/ground_three_point_axis_runtime_affected.xml
172 passed in 107.01s
python -m py_compile src/inspection_mission/inspection_mission/ground_three_point_axis.py tests/test_ground_three_point_axis.py
exit 0
Git Bash: bash -n scripts/run_ground_three_point_axis.sh
exit 0
```

新专项包含两条完整三点合成反馈顺序：`ground_2→ground_0→ground_4`、`ground_4→ground_3→ground_2`；每点当前视觉、逐点声明、epoch递增、旧匹配拒绝、handoff轴向接线、负X运动、最终姿态与全局预算都通过。危险scan、反向的错误运动方向、横偏、heading、DDS丢失均吸收锁停；实际净空阈值仍.641m。没有跑整仓。

当前adapter关联检查：同一7文件命令输出`179 passed in 113.68s`，其中axis专项27、旧three15、axis planner5、旧planner17、single43、receiver68、telemetry4；XML为`runs/review/ground_three_point_axis_runtime_adapter_affected.xml`。当前冻结清单在`runs/review/ground_three_point_axis_runtime_20261112/freeze_manifest.json`，初版清单保留为`freeze_manifest_initial_review.json`。`py_compile`和Git Bash `bash -n`均exit0。旧single、observation、three、两份planner、vertical源SHA全部与冻结值相同。冻结后交原独审人复查，不打包上传。

## 仍需实际证据

本实现修复v26恢复斜线的运行接线回归，并支持显式负X；不能据此宣称平台三地面点完成。负X标定尚待平台证据，真实方向、制动、反向停稳和窄几何余量必须逐项验证。候选地图仍未完全确认，约9mm原名义余量不能证明实际成功率。飞行、灯、完成事件继续静默；局部流程完成、exit0、回传成功均不是官方成绩。
