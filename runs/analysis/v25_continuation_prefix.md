## 06:12 最新：首个官方地面点10分

- v25 已真实终态 **success / score=10.00 / durationMs=275179**；详情ID `c4202874-407a-4361-844a-0f95ba7b1d7f`，runId `run_20261112_060713_501f412f`，编号 `#86630201`。
- 官方UI赛后诊断明确：`ground_0 到达并计分 +10`、`总分10`；来源 `runs/evidence/20261112/single_point_v25_list_final.json`、`runs/evidence/20261112/single_point_v25_official_success.png`。
- raw `runs/evidence/20261112/callback_single_point_v25.jsonl`；62事件、885764B、launcher_exit0；独立分析由evidence_refresh收尾。父代理新目录 `runs/analysis/20261112_single_point_v25/`。
- 实际首点ground_0：三帧各自曝光/前序pose、在线一致被困人员候选；一次识别发布后收到原始String `ground_0`；实际末位 `(1.1131353,-1.3556102)` 停车并一次到点声明。ROS发布返回、原始status与官方+10证据各自保留。
- source `ground_single_point.py` SHA `902424ca5ba3ec208a0028645d377f4ce92ff77e5ee56205c87fac95a9b349cb`；ZIP `dist/inspection_single_point_v25_src.zip` SHA `6c3f39642065ee14b07775a37fa1b9eac6dce91a37977382b337ad27858bbf1e`；SH SHA `79d9cf1f4e980599c9e2ee8cae061a8ecc9828678aa92cac2c652fcb6fef8c5f`。
- 本局没有发送全任务完成消息，仍自然结算为success/10分：因此“没有完整结束标志就必然取不到任何成绩”的旧推论已被本局反证；不推定任意退出都能结算，也不把首点10分称完整巡检成功。
- v24b原始结果仍failed/null/128875ms。真实根因：相机header约3Hz，wallgap最大.384845s，而新增相机活性门.20s；修后相机独立.50s、重复/倒退曝光不刷新arrival，odom/scan .20s与原曝光prior .05s不变。43作者专项、18独立相机/传输、13边界、125解析范围通过，详见相机修复与v24b独立报告。
- 当前任务继续：route_controller设计覆盖60合法三点顺序的连续地面走廊（无生产改动）；mission_bridge在完整控制器分离水平/上下包络（不影响v25冻结包）；evidence_refresh独审本局。正式完整视频和正式PDF仍待完整平台任务证据。
- 网站仿真中心、AI开发助手、算法解析已实际操作，均提示暂未开放，详见 `docs/platform_tools_availability_20261112.md`；本轮复查既有技术求助仍0评论。

以下为历史状态，应以本节最新官方证据为准。

