# v33 南侧预定位后的有限反向响应实现

作者预检完成，尚未上传平台；是否准许运行由独立审查与 root 决定。所有合成证据仅证明软件合同，不能证明实际反向运动或官方任务得分。

新增模块 `src/inspection_mission/inspection_mission/ground_relocated_reverse.py`，新增启动器 `scripts/run_ground_relocated_reverse.sh`；旧 v28/v30 和地面任务源码保持冻结。源码 SHA256 `ab04e3648637d45981373a82da47b13bdd5c259d3f06a0a9e427a610f950809a`，启动器 SHA256 `b9305b0771d4be3645237ed9f1daf4d868e90ba52a737070fc3c1762f667256a`。

## 运行顺序与截止

初始负转 `-pi/2`，归零后按源和墙钟各静稳1s；以正 X `.03m/s` 请求实测向南移动 `.15m`，在目标前2mm请求零，停车后静稳且实际端点误差≤`.02m`；再分别正转到0和`+pi/2`，每段真实重新锚定、归零并静稳。最后执行 `-.03/-.10/-.10` 三次负 X 请求，每次沿用2s、`.010m`实测请求零、`.04m`硬停、`.03m`停车保留和归零后`.01m`累计路径门。

新预先合同 path`.35m` / radius`.25m` / rotation`5rad` / global180s / 单turn45s / 总turn120s / preposition40s。预定位截止在上传前由 root 根据真实 v28 速率从30s修订40s，并先观察默认40s断言RED，再更新新模块。单次、总转向时间包括静稳；每段只重锚局部pose，永久原点、路径、绝对转角和创建时间不重置。半径`.25m`允许常规约`.18m`目标路径，但不承诺允许三次脉冲都到`.04m`硬上限；全局半径可能更早停止。

`.641m`全环净空、源/arrival`.2s`、真实prior odom≤scan/skew≤`.05s`、`.03m`扫描锚移动、`.06m`局部扫掠、`.05rad`朝向和`.06m`转向漂移门保持冻结。预定位按真实world南向进度推进，横偏`.02m`、回退`.005m`；不假定命令与响应1:1。候选静态地图不能代替实时传感器准入。

## 传输与诊断

新适配器直接通过 `create_bounded_node(...,guard_factory=...)` 创建新guard，不替换正在运转的旧状态机。所有graph查询后最终读取实际DDS匹配，再使用当前墙钟复核预算、phase、源和arrival龄；旧decision不能跨过deadline发布非零。publish异常STOP并独立零尝试。唯一允许非零输出为`/cmd_vel`，其他publisher静默。

首个被拒scan完整payload只保留1帧，并记录原因、阶段、arrival和最近收到的真实odom（该odom明确不自动等同有效prior配对）。合法raw odom/scan、请求零、实际发布成功/失败的命令、阶段和累计预算保留。

证据schema `ground_relocated_reverse_evidence_v1`，core section `ground_relocated_reverse_core`，分片section `ground_relocated_reverse_evidence_gzip_part`，bundle `ground_relocated_reverse_v1`。仍为gzip+base64、每片48000字符、原始/压缩双长度和双SHA；解码器应使用新section，而不能只筛旧v28标签。

## 作者验证

先观察缺模块RED；随后专项36项通过。最终必要关联208项通过，`py_compile`与Git Bash `-n`通过。XML与冻结manifest在 `runs/review/ground_relocated_reverse_20261112/`。

首轮当前版本关联测试为207通过、1个冻结v28的Y阶段集成测试失败。该测试使用真实watchdog线程和虚拟时钟，原样单独复查1通过；同组原样重跑208通过。没有修改冻结源码或测试，初次失败XML保留为`related_results_first_attempt.xml`，不能据此宣称不存在调度波动。

专项覆盖完整factory→订阅回调→tick→实际唯一cmd_vel发布、三个转向与真实预定位、每个交接双时钟静稳、最终DDS查询跨global/turn/preposition/pulse截止的零输出、publish异常独立零、危险首帧保留与分片完整性。独立审查另出报告；作者预检不能替代独审。
