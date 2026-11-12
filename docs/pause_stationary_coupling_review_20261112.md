# PAUSE恢复：静稳证据与传感器许可耦合审查

日期：2026-11-12。本报告为本机源码只读审查与后续修复设计；未改控制代码、未生成新诊断提交、未把设计作为平台成功证据。

## 问题已由源码与真实样本共同确认

`ground_bounded_approach.py`的`temporal_fault()`（303–309）对所有FRESHNESS原因都立即清空`stable_anchor`，然后进入PAUSE。`step()`（379–386）在调用`_stable_dwell()`之前先检查完整`_condition()`，失败即返回零速；因此scan短暂stale、pairedscan锚点暂缺、缓存odom在clock推进间短暂stale都能反复删除一秒静稳证据。`_check_active_timing()`和watchdog会从回调/独立线程触发同一路径，修复只改step而不处理这两个调用入口仍会被清空。

`_stable_dwell()`本来仅依据odom：速度≤.01m/s、角速度≤.03rad/s，相对锚点位置≤.005m、姿态角≤.02rad，且wall/header都推进至少1秒。但它现在只有“全部输入在同一tick均fresh”时才运行，实际上把“机器人已经静止”和“此刻有足够传感器许可运动”绑成了同一事实。

v17保留日志恰好呈现此循环：wall86.4197双fresh且等待驻留→86.5208scan wallage=.24467s→86.5713odom simage=.24s→86.6723双fresh再次驻留→86.7733再次odom/scan stale。最新STOP边界双fresh但预算已耗尽。该因果路径证明计时器确实会被这种序列清空；现有压缩回传并未保存每一条原始odom，因此不能反过来宣称v17全程已存在足够的连续低速odom证书，或新设计必定在该局成功。

## 建议的最小结构：独立静止odom链，完整许可仍在命令前

保留原`_condition()`、同步配对、watchdog、安全几何和全部预算。新增一个常数空间的静稳记录器，不把现有`stable_anchor`简单改成“永不清空”。记录器只证明：同一时钟/坐标epoch、同一原任务phase、实际零速命令时期内，持续接收并通过验证的odom样本符合原静稳阈值。

记录最少包含：零速时期起点、首/末有效odom原始header和arrival wall、首pose/quaternion、有效样本数、最大相邻wall/header间隔、最大速度/角速/锚点偏差、失效原因。不能靠step反复读取同一缓存pose累计时长，也不需要存储全量历史。

1. **在真实命令记录路径建立零速时期。** 保守地在非零命令决策/发送时立刻清空证书；停止后至少成功执行一次publish零速并进入原有command记录路径，才允许新的静止链开始。仅把state设置成PAUSE不能证明已发停止，实际停车仍由后续低速odom证据确认。watchdog重复发零不重置已在建立的同一时期。
2. **每条实际odom回调推进链，而非每个tick。** 必须先通过原frame、child_frame、数值、单位四元数、时间戳/epoch检查；样本在接收时满足原odom/clock新鲜度、clock仍有效推进，再验证速度、角速以及相对静稳首点的位移/姿态均在原边界内。wall与header严格推进的独立样本才可增加覆盖；至少两个有效样本，`min(last_arrival-first_arrival,last_stamp-first_stamp)≥1s`，并且每一对相邻样本的wall/header间隔都≤原`.2s`上限。不得使用后来的样本填造先前缺失区间。
3. **短scan stale/暂缺pairedscan只关闭此刻运动许可。** 在PAUSE且已发零、odom链仍有效时，scan新鲜度短缺或等待真实prior配对不清除静止链。它们仍立即触发/保持PAUSE、watchdog仍发零；原scan stamp/arrival/skew/.05s配对与净空边界不变。
4. **缓存odom短stale本身不能伪装成新的静止样本。** 此刻仍必须零速。可保留已收真实样本链等待下一条合格odom；若下一真实样本与前一合格样本的arrival或header间隔>.2s，或等待过程中wall已超过这个无观测上限，则清空。不要继续用“clock减缓存header短时>.2s”单独抹去尚未超相邻样本间隔的历史链，否则原耦合仍在；但它始终阻止当前非零命令。任何新收到的odom若本身超原年龄/未来容差，不能加入链。
5. **准备好不等于可以发非零。** PAUSE恢复点仍在同一锁内先检查原预算、硬运动边界、实际DDS匹配，以及完整`_condition()`（clock/当前odom/当前scan/pairedscan全部fresh、倾斜及净空正常）。只有“静止链证书ready AND 完整许可此刻通过”才消费一次resume；最后发布前仍沿用原命令锁及匹配核验，任何失败保持零或按原规则硬停。证书不代替实时odom、scan或配对。

恢复不重置`turn_started_wall`、`drive_started_wall`、全局起点、累计路程/转角/恢复计数，暂停仍耗原预算。证书不能把等待时间转成额外任务时间，不能允许第4次恢复，不能使STOP/FINISHED复活。

## 必须清空或硬停的边界

| 事件 | 静止链处理 | 原控制处理 |
| --- | --- | --- |
| 任意实际非零速度/角速命令或改变任务phase | 清空；新phase需新证据 | 原阶段/预算继续 |
| odom速度>.01、角速>.03、首点位移>.005或姿态>.02 | 清空；低速后重新起链 | 保持零；超原硬运动限仍硬停 |
| 任意中间odom非法、frame/child改变、非有限值/坏四元数 | 清空 | 按原规则硬停，不靠下一好样本覆盖 |
| wall回退/非法、clock回退、odom或scan stamp回退、epoch失效 | 清空并禁止跨epoch复用 | 原硬停保留 |
| 相邻有效odom arrival或header gap>.2s，或无新odom的wall等待>.2s | 清空；下一合格样本仅作新起点 | 当前零速；原预算继续 |
| clock停止推进/无clock超过原时限 | 清空；恢复后重建 | 原clock门控/零速或硬停保留 |
| 重复header | 不累计覆盖，不刷新链的末有效样本时间；有不一致内容则清空 | 仍按原当前新鲜度门控 |
| 当前收到odom已旧或超过原clock未来容差 | 不加入，保守清空 | 当前零速；不改stamp/arrival |
| 外部匹配丢失/endpoint类型冲突/发布异常 | 清空 | 原硬停保留，不能靠证书恢复 |
| scan仅短stale或真实prior暂未到 | 已发零且odom链连续时可保留 | 保持PAUSE零速，禁止非零 |
| scan实际非法、碰撞净空不足、局部运动储备耗尽 | 清空 | 原硬停保留，不能混进“只是stale”分支 |
| 原全局/阶段/路径/转角预算到界、STOP/FINISHED、public draw非法/改变 | 清空/证书失效 | 原终止保留 |

必须区分名称`scan_stale_or_unsafe`中的实际来源：有效scan仅变旧可等待；验证器遇非法返回值或too_close已硬停，不能因为字符串带stale就宽泛保留/恢复。

## 零速阶段的衔接

若恢复的原phase是TURN或DRIVE，一旦发非零即清空静止链，不跨实际运动借用。若原phase为TURN_SETTLE/DRIVE_SETTLE/CAPTURE，恢复仍是零速，可以在**同phase、同零速时期**将已验证的静止首点/时间交给现有静稳检查消费，防止刚解除PAUSE又机械清空、再要求同一段静止重复等一秒。不得跨phase转移证书；图像capture仍需原相机、prior pose、曝光freshness和空间姿态检查。

更窄的第一步也可以只在PAUSE恢复TURN/DRIVE时消费独立链，WAIT及其他零速阶段保留原实现；但这仍可能留下零速阶段的相同饥饿问题，不能宣称完整任务已修复。建议实现前明确这一处范围，不顺带改速度、时间、配对或QoS。

## 实现前必须具备的本地验收

- 持续合格低速odom覆盖一秒，期间scan每隔短时间stale：证书可ready，但所有不fresh时刻命令严格为零；仅在原完整许可恢复的tick允许恢复一次。
- 中间任何一条移动/非法odom即重置，即使两个tick所见首末pose相同也不能遗漏；有界小漂移需相对首点判定，不得逐帧累积容差漂移。
- 仅wall推进、clock冻结、重复header、缺失odom不能生成证书；arrival/header任一gap越界均重置。边界容差沿用原值，不加新的隐含宽限。
- 证书ready时当前scan旧、缺prior、错误skew、clock/odom旧、endpoint无匹配仍零；非法scan/预算到界/epoch回退仍终止。
- 首次零速尚未实际发送前的低速样本不能计入；重复watchdog零速不反复重置；恢复非零清空；零phase同phase交接不得绕过freshness/图像验证。
- PAUSE期间各原计时器、路径/角度总量和恢复次数不变；watchdog、命令锁、最后一刻匹配检查继续有效。

上述仅为设计验收清单，尚未实施或跑这些新测试；不增加平台诊断局，先让当前仅改变/odom reliability的v18保持单变量。
