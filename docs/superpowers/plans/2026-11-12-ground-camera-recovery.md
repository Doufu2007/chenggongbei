# v36 有界相机恢复实施计划

设计：`docs/superpowers/specs/2026-11-12-ground-camera-recovery-design.md`，授权来自根代理给定完整合同。当前工作区无 Git 元数据，保持原文件不动，新增独立文件并记录 SHA。

1. 先写专项测试，将新 guard 暂指向冻结 guard，复现 v35 .530038s 图像超龄永久停；同时验证源龄正超、非法负差/倒退、报告后、恢复额度和发布边界。
2. 新增独立 guard：时向/帧结构验证，合法超龄有界PAUSE，实时图像恢复后经原停稳/新epoch，worker身份隔离，有界事件；不改旧基类。
3. 新增独立完整轴向节点 factory 和启动器，factory 从一开始构造新guard；唯一节点行为差异为新guard的相机恢复合同。
4. 跑新专项和必要关联（冻结 axis/单点/worker），compile与bash-n；记录红绿证据和冻结SHA，交根代理独审，禁止自行上传。
