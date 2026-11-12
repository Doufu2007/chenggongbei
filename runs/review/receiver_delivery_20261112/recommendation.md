# 接收器可选投递账本：最终实现与历史诊断

## 最终实现（覆盖后附历史建议）

主代理随后明确授权最小实现及本地 loopback 测试。已只修改 `tools/diagnostic_receiver.py`、`tests/test_diagnostic_receiver.py` 及本地审查材料；未启动公网隧道、未上传、未改 dist。

新增默认关闭的 `--delivery-log PATH`，或 Python 参数 `create_receiver(output, port=0, delivery_log=...)`。原 accepted JSONL 的 `{received_at,payload,source}` 三字段、事件校验、状态码、loopback绑定、随机路径、5秒 body 超时和100ms/64KiB拒绝drain保留。accepted与delivery的resolve路径相同会拒绝初始化，防止混写。

只覆盖现有POST、GET及其已有方法别名、这些分支的响应写入失败及服务器 `handle_error`。未重写HTTP parser、未扩展未知方法、未新增summary文件。以下历史HMAC、默认sidecar、统一生命周期、summary、未知方法/parser钩子的建议均未实施。

账本固定字段为：UTC `received_at`、固定 `event`、已知方法或 `OTHER`、target的SHA256前16hex或null、`exact_path`、尝试的 `http_status` 或null、固定 `reason`、`response_completed` 和 `evidence_appended`。不保存完整URL、随机路径、query、body、header、IP或原始异常文本。

当前reason为 `path_mismatch`、`method_not_allowed`、`framing_invalid`、`body_too_large`、`content_type_invalid`、`body_timeout`、`event_invalid`、`evidence_io_error`、`accepted`、`response_write_error`、`handler_error`。不完整body延用原400/校验失败分支，记为 `event_invalid`。

accepted append/flush成功后 `evidence_appended=true`。202应答失败时保留 `request_result`/`accepted` 且 `response_completed=false`，另写 `response_delivery_failed`/`response_write_error`；异常仍交回原服务器流程。`handle_error`只补通用错误行，路径/状态为null。`response_completed=true`只代表本机write/flush返回，不能证明客户端收到、平台启动或官方得分。

固定最多4096条、每条含换行最多1024字节、总追加预算最多4MiB。首次写入有界读取已有账本，既有行数计入上限；已有文件超限或尾部不完整时不继续追加，不截断原文件。超限只增加内存 `delivery_dropped_count`。与accepted独立锁；账本IO失败只增加 `delivery_write_errors`，不回滚accepted、不改状态码、不输出原异常。

账本在本机响应flush后写入，因此客户端读完响应时记录可能仍在handler中完成；测试用有界等待观察记录。

正式receiver与telemetry测试 **82 passed**。新增覆盖404/405/400/413/500原因、accepted隔离、202响应失败双记录、账本IO失败不改202、并发cap、已有行数、1KiB行界限、handler_error隐私、CLI开关。真实body/原schema及原drain测试均保留。

历史离线回归额外运行时发现旧fixture通过`__new__`绕过构造却人为设置非空delivery_path，未初始化账本锁。该次报告完整保存于 `offline_fixture_failure.xml`（82 passed/1 failed）。仅将fixture改为实际默认关闭配置后，受影响离线测试 **1 passed**，见 `offline_final.xml`；未为伪造对象扩展生产逻辑。

`implementation_red.xml`是新增默认关闭API的明确断言RED；`implementation_green.xml`是首轮77项GREEN；后续82项正式测试通过也完整包含于 `offline_fixture_failure.xml`。本目录`final.xml`是加入旧fixture后发生的历史失败报告，最终联合回归由主代理统一运行。未覆盖RED或失败证据。

最终生产SHA256：`ef7d0d752d4981de9dae468b2e80f48ea41213538e4403012f99d2e37537a18d`。最终测试SHA256：`cb5d96a58025e2da9f7a755cbad41719043948d5040bb8f4bca05d6c9fd25d79`。

`test_delivery_contract_red.py`和`red.xml`仅为授权实施前的拟议合同，不是最终实现的验收合同，不应加入默认测试集。

v41旧receiver没有账本，无法事后恢复未到达/已拒绝/回包失败的细分。新增功能只为下一次完整流程增添本地诊断证据，不补造启动证据。

## 以下为实施前的只读诊断及历史建议

2026-11-12，ground_next_gate。范围仅本队 `tools/diagnostic_receiver.py` 与 `tests/test_diagnostic_receiver.py`。未改生产代码或现有测试，未创建socket、未联网、未上传。v41尚无 `launcher_started`；本审查不能据此归因算法，也不改变下一局完整任务范围。

当前生产SHA256：`1621108f3b1688a8e177240970dd2620525a683aac05ed665119697622211d6a`。测试SHA256：`fb7857a27695f0412559d5d8f3234f037723bf4d4396b6db43f06bf2ce8e1a87`。

## 已确认的信息缺口

`append_event`（114–124行）是唯一持久化通路，且仅在原事件校验通过后执行。`log_message`、`handle_error`均刻意静默以避免泄露随机路径。结果是请求未到达、路径不匹配、方法不合、HTTP framing不合法、载荷过大、JSON/schema无效、读body超时/不完整、证据文件IO失败这几类情况，在 accepted JSONL 中均不可区分。

另一个可复现情况是：真实 `append_event` 已成功，随后202响应的 `wfile.write` 发生 `BrokenPipeError`；accepted证据中已有1条 `launcher_started`，HTTP调用方却未收到完整202，异常交给空的 `handle_error`。因此“证据保存”和“应答写完”必须分列，不能用一个 success/failed 字段混淆。

`offline_receiver_probe.py` 将**真实生产Handler**接到内存HTTP字节流，不调用 `create_receiver` 或socket API，复现13种结果，见 `observed.json`。`test_current_behavior_offline.py`确认现有状态码与accepted证据隔离；`test_delivery_contract_red.py`的7项拟议合同全部在“缺少投递记录”处正确失败，见 `red.xml`。这些RED是设计验收样例，尚未实现。

## 最小改动边界

保留现有 `--output` 的accepted证据JSONL及 `{received_at,payload,source}` 三字段原语义，GET/探测/拒绝/异常请求永不插入此文件。旁路默认写 `<output>.delivery.jsonl`；如果需要CLI指定位置，只增加可选 `--delivery-output`，不得用命令行打印完整请求路径或异常文本。

在 `_Handler.handle_one_request` 的一次请求生命周期建立内部摘要，`finally`恰好写一条旁路结果。`do_POST`各现有return前仅标记固定 reason；`_reply`在响应写入完成后才置 `response_completed=true`。`send_error`或解析失败也标记固定状态/reason，覆盖未知方法501和请求行/头解析错误。保留现有状态码、5秒body超时、100ms/64KiB拒绝drain预算、严格secret path和事件校验规则。不要为了诊断改接收合同。

建议每条旁路字段：

| 字段 | 语义与上界 |
| --- | --- |
| `schema` | 固定 `receiver_delivery_v1` |
| `request_seq` | 本receiver单调请求序号，持锁分配 |
| `started_at`, `finished_at`, `elapsed_ms` | UTC时间、单调钟耗时；不取客户端时间 |
| `method` | 已知HTTP方法原值；未知且非 `[A-Z]{1,16}` 标 `OTHER`，防请求内容注入 |
| `path_fingerprint` | 原请求target的HMAC-SHA256前16hex；每receiver随机key，仅内存保留，不保存完整路径、token或query |
| `path_exact_match` | 与当前随机secret path逐字匹配bool；不改变当前query也404的规则 |
| `http_status` | 已尝试状态码或null；不能代表对端收到 |
| `reason` | 以下固定枚举，不拼接用户输入 |
| `evidence_appended` | 原 `append_event`写入并flush成功后true |
| `response_completed` | `_reply`/错误应答完整写入并flush返回后true |
| `exception_type` | 从小枚举取 `TimeoutError`/`OSError`/`BrokenPipeError`/`ConnectionResetError`/`OtherError` 或null；不保存 `str(exc)` |

固定reason建议：`accepted`、`path_mismatch`、`method_not_allowed`、`unsupported_method`、`request_parse_invalid`、`http_framing_invalid`、`body_too_large`、`content_type_invalid`、`body_incomplete`、`body_timeout`、`event_invalid`、`evidence_io_error`、`response_write_error`、`handler_error`。若是应答失败覆盖主reason为 `response_write_error`，可另保留 `decision_reason` 字段，避免丢失原拒绝分支；仍限制为同一枚举。无需保存HTTP请求body、原header、client IP、User-Agent或任意异常消息。

账本必须有硬上界。最小建议：最多4096行、每行≤1024B，前4095请求逐条写，最后保留 `ledger_limit_reached` 固定行；随后只累计内存 `delivery_dropped_count`。另一个≤2KiB的summary文件在正常退出时原子覆盖计数（total/accepted/rejected/response_errors/dropped/ledger_write_errors），不逐请求无限增长。计数与旁路写入用独立锁，不能持accepted写锁进行第二次文件IO。账本IO失败只递增计数并继续既有HTTP行为，不回滚已接受证据，也不返回新状态。失败摘要只输出固定文字/计数。

初始化时检查accepted路径与delivery/summary路径的resolve结果不相等，防误配置混入或覆盖。所有旁路写入仍属于本地接收器观察，不能当作上传平台已启动、裁判确认或官方得分。

## 实施验收的最小测试集合

现有68项receiver测试覆盖HTTP拒绝、接受隔离、并发、秘密路径不落盘、drain边界和CLI生命周期，未覆盖独立投递账本；本次为遵守“不网络”仅阅读，未运行其socket/CLI项。实施后保留这些原合同测试，再运行本目录离线RED合同（适配正式构造配置即可）。

应补的独立旁路测试：

1. 404/405/400/413/500分别有一条ledger，但accepted文件无新字节；成功202仍仅保持原accepted schema。
2. 正确路径与“正确token加query”有不同fingerprint，后者404；日志内找不到fixture token、请求query、恶意headers/body、异常消息。
3. accepted append后BrokenPipe：`evidence_appended=true`，`response_completed=false`；不能丢原证据或误计作未接收。
4. framing重复长度、short read、body timeout、未知方法501及解析错误被账本明确区分；异常仍有单一最终行。
5. 连续超过4096请求后文件总行数/大小仍有界，summary有准确丢弃计数；并发无破行、无重复序号；ledger IO拒绝不污染accepted或改变原状态码。

## 对v41的可下结论

当前只能说“本地accepted证据未见 `launcher_started`”。如果旧接收器没有旁路投递记录，事后无法恢复“未到达/已被拒绝/回包失败”的细分。新增账本能缩小下一次全流程的排查范围，不能补造v41启动证据。下一轮继续同一完整任务，不用独立局部probe替代。
