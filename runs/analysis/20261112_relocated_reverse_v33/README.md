# v33 严格解码与只读分析工具

已完成真实 `runs/evidence/20261112/callback_relocated_reverse_v33.jsonl` 的严格解码和只读分析：155 事件、113/113 分片、启动器退出码0。唯一session、双长度/双SHA和独立core完整键值集合均一致。详细报告为 `docs/ground_relocated_reverse_v33_platform_analysis_20261112.md`。

官网终态是未通过、未提取成绩、页面0分（341.2s）；本地有限实验 `FINISHED` 不代表官网测评成功。真实位姿独立确认三次短负X响应；南向预定位148.180mm，三段转向均完成，三次数据陈旧停车后恢复；十次零请求观测窗内累计路径均小于10mm。本局不验证识别、计分或完整巡检。

脚本不生成平台证据；缺少/未收齐真实回传时返回2，不创建成绩或测量结果。

```powershell
python runs/analysis/20261112_relocated_reverse_v33/selftest.py
python runs/analysis/20261112_relocated_reverse_v33/analyze.py --platform-id 87e16c7f-bc1c-4329-aef2-bfff61a13e85 --run-id run_20261112_083208_b88d3f13
```

解码严格限定新mode/schema/bundle/sections，校分片/重复冲突、双长度、双SHA、唯一session和独立core完整键值集合。原始解压字节保留为`evidence.json`，压缩字节保留为`evidence.json.gz`。

分析复用冻结v28的源时间位姿差分/实际命令逻辑，并额外重算预定位实际南向进度与横偏、三次转向、各次归零后累计路径、三个负X的世界/初始机体系位移，以及阶段起始path/angle与永久全局账本的一致性。首拒绝scan完整payload保留在原始evidence，另提取真实prior配对与最小有限束供几何诊断。

本工具检查内部证据一致性。它不独立认证回传来源，不把本地FINISHED当网站测评成功，也不把有限响应实验当完整巡检得分。
