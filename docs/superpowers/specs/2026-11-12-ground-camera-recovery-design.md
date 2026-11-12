# 独立 v36 相机存活恢复设计

依据 v35 实测和根代理已明确授权的合同实施；使用系统调试与 TDD。只新增 `ground_three_point_camera_recovery.py`、新启动器和专项测试，不改冻结轴向路线、单点、worker、控制基类。

唯一功能变化：CAPTURE/RECOGNIZE_WAIT、识别未 attempt 时，合法当前图像暂时正向超龄转为有界零速 PAUSE；相机恢复并重新满足原双时钟静稳后由父逻辑开启新 epoch 重采。原每点一次重采、全局三次恢复、全部跨点累计和几何预算保持。原 .50s 图像/.20s 控制/.05s prior 和[-.12,.50]源龄保持。非法帧、倒退、delta<-.12、报告期/attempt后永久停止。恢复期间旧worker context取消，旧结果不得进入新epoch。保存有界恢复事件：wall/source/reason/epoch/已用额度/恢复结果。

完整复制冻结 axis 节点 factory 到独立模块，仅将构造 factory 接至新 guard，完整保留发布前 DDS、最终时钟/状态/预算核验、异常零请求、计数和单次话术派发。这样避免全局 monkeypatch 和运行后替换 guard，代价为约150行 wrapper 重复；冻结来源 SHA 和复制差异进入交付记录，交独审校验。

本改动不证明官网成功；v35已用完全局三恢复额度，相同时序仍必须停。
