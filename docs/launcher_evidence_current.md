# 当前启动与日志证据审计（2026-11-06）

## 2026-11-11 正式启动器补充修复

下文为历史排查。最新复核在正式 `scripts/run_inspection.sh` 中确认另一个路径问题：启动时当前目录若含无关 `install/setup.bash`，旧查找会在加载它后直接停止，即使脚本所属工作区已有正确安装。此问题已由真实 Bash 与临时 overlay 测试复现并修复；不能据此认定它是平台不回传首行日志的原因。

正式脚本现在优先显式工作区、其次脚本父目录与脚本目录，最后使用调用方当前目录。每个 overlay 加载后检查是否能解析 `inspection_mission`，不能解析则继续；source 本身失败不被吞掉，按实际退出码结束。新增测试覆盖无关 cwd、显式优先、cwd 后备、缺包后备及 return/exit 失败码。相关测试 23 项通过，全项目 296 项通过；未运行真实 ROS/Gazebo 或网站测评。

结论：现有本地证据不能确定平台未启动脚本、构建失败，还是未采集用户输出。没有发现足以解释全部运行“连首行 shell 标记都没有”的 Python 包结构或 ROS 2 Humble 语法错误。可以确认标签探针的工作区定位弱于官方示例；这是可修的启动假设，但尚无平台日志证明本轮命中了它。

## 已核实的证据

| 检查点 | 本地证据 | 能得出的结论 |
| --- | --- | --- |
| 源码 ZIP 层级 | 两个 ZIP 都含 `src/inspection_mission/package.xml`、`setup.py`、`setup.cfg`、`resource/inspection_mission` 和 Python 包目录 | 与缓存前端校验正则 `^src/[^/]+/package.xml$` 一致；不应改成 ZIP 顶层直接放包 |
| 构建类型 | `package.xml` 的 buildtool 与 export 都为 `ament_python` | 声明内部一致；没有证据需要改成 `ament_cmake` |
| ROS 可执行入口 | `setup.py` 注册 `interface_probe = inspection_mission.probe:main`；`setup.cfg` 安装到 `$base/lib/inspection_mission` | 是 `ros2 run inspection_mission interface_probe` 所需的标准结构 |
| Humble 的 Python 基线 | 两个 ZIP 内所有 Python 文件均用 `ast.parse(..., feature_version=(3, 10))` 解析成功 | 排除 Python 3.10 语法不兼容；不能替代真实 Humble/rclpy 运行 |
| Shell 文件格式 | 当前前台/标签探针和最小 echo 均 LF、无 UTF-8 BOM；Git Bash `bash -n` 返回 0 | 排除这些文件的 CRLF/BOM 与 shell 语法问题 |
| 最早启动标记 | 前台脚本先 `printf INTERFACE_PROBE launch_start...`，之后才 source ROS；最小 echo 脚本不依赖 ROS/本队包 | Python 导入、`ament_python` 构建、找不到 ROS 包，均不能单独解释首行标记未回传 |
| 服务端可见结果 | `docs/platform_probe_result_20261106.json`：`score=null`、`status=failed`、`tabLogs` 仅裁判、`terminalLogs=[]` | 没有有效算法成绩，也没有用户脚本执行/退出证据；不能将网页 0 分当作实测成绩 |

## 官方参考实际说明了什么

- `.guide1.png` 明确要求上传工作空间 `src` 文件夹的压缩包。缓存前端 JS `.baf13c38c0939482.js` 进一步要求 ZIP 内为 `src/<package>/package.xml`，当前 ZIP 满足。
- `.guide2.png` 指向工作区 `scripts/run_example_mission.sh`。
- `.guide3.png` 的示例路径是 `~/ican_fly/ican_shoot_ws/scripts`，内容是 shoot 示例。它先用 `BASH_SOURCE[0]` 求 `SCRIPT_DIR`，用父目录求 `WORKSPACE_DIR`；`SETUP_CMD` 显式 `cd` 到工作区，source `/opt/ros/humble/setup.bash` 和绝对的 `${WORKSPACE_DIR}/install/setup.bash`。它没有证明 inspection 平台强制要求某个 tab 名字。
- `.guide3.png` 的红字只要求删除仿真与裁判启动命令、保留自己的包；没有说必须通过 `gnome-terminal` 才能收集用户日志。
- `course_frames/005040.jpg` 显示巡检示例先运行 `scripts/01_setup_inspection_workspace.sh`，编译用户示例包 `ican_inspection_examples`，再 source `scripts/setup_env.sh`。
- `course_frames/006480.jpg` 显示环境检查分别检查 ROS Humble setup、仿真/Nav2 二进制环境 setup、工作区 local setup。该本地示例具有 `binary_install` 与 `install` 两层；当前缓存没有 `setup_env.sh` 源码，不能猜出平台二进制 underlay 的路径。
- `course_frames/008640.jpg` 显示官方本地 `gnome-terminal --command` 只是 deprecated 警告，随后返回 shell；这不能证明评测容器里也支持 GUI。当前须知明确评测无图形界面，并要求仅启动自己的节点。
- `course_transcript*.txt` 是识别质量不稳定的语音转写；其中没有可据以还原平台启动路径或编译命令的可靠新增信息。涉及精确命令时以上述画面为准。

## 当前脚本中可直接验证的问题

1. `scripts/run_interface_probe_tab.sh` 没有计算 `SCRIPT_DIR/WORKSPACE_DIR`，只在启动时的当前目录尝试 `install/setup.bash` 与 `../install/setup.bash`。即使脚本位于正确工作区的 `scripts/` 中，从其他目录调用它也可能完全漏掉工作区；后面的 `ros2 run` 会找不到本队包。
2. 标签脚本没有 `set -e`，source 或 `ros2 run` 失败后仍 `exec bash`，会掩盖子命令退出码。若平台只看到外层 `gnome-terminal` 客户端返回，它也未必能获得节点退出状态。
3. 前台脚本虽会按 `INSPECTION_WORKSPACE/PWD/SCRIPT_DIR/父目录` 查找，但找到任意 `install/setup.bash` 就停止，不检查是否找到了本队 `inspection_mission`。若当前目录是另一个工作区，它可能提前选择无关安装区。
4. 前台脚本在找不到任何 overlay 时继续 `ros2 run`，没有显式的“未找到已编译工作区”诊断。不过即使如此，脚本首行仍应该已经输出；这条问题不能解释首行也缺失。

以上是代码路径上可复现的问题，不等于本次平台运行的已确认根因。

## 建议的最小修复与验证顺序

下一次启动边界验证应继续前台执行，保持公开只读探针行为，只调整启动定位及错误呈现：

1. 保留 source ROS 前的唯一启动标记，并把 `pwd`、脚本绝对路径写到 stderr；为 ERR/EXIT 写出阶段与退出码。
2. 优先使用调用方提供的 `INSPECTION_WORKSPACE`；否则按官方模式以脚本父目录定位工作区，并以现有有限候选补充，不扫描官方私有目录。
3. 找到候选后 `cd` 到该工作区再 source 绝对 `install/setup.bash`；用 `ros2 pkg prefix inspection_mission` 与 `ros2 pkg executables inspection_mission` 核实本队包和 `interface_probe`。找不到则明确非零退出，不静默继续。
4. 不改变 ROS domain/RMW 隔离设置，不复制或自启官方 underlay/仿真/裁判。二进制 underlay 若有缺失，应取得平台公开启动合同后按其路径加载。

这次改动只能修复路径假设并改善可观测性。若平台仍不返回 source 前的标记，继续更改 Python 算法、包构建类型或 tab 名字不能验证任何假设；需要针对同一 run 获取平台的编译命令/返回码、脚本实际路径/工作目录、用户 stdout/stderr 与脚本退出码。也可由平台提供一个已知成功的 inspection 最小源码包与其启动脚本，作为相同环境的对照。

## 本次审计的验证边界

初次审计只读检查了现有文件、源码 ZIP、官方参考画面和缓存网页脚本，没有提交新测评、运行真实 ROS 节点、读取私有裁判文件/隐藏随机答案或改变 ROS 域。

## 启动边界修复与本地回归

后续按任务要求修复了 `scripts/run_interface_probe.sh` 并复制到 `dist/run_interface_probe.sh`：

- 启动前打印标记，EXIT trap 打印实际退出码；允许 `INSPECTION_ROS_SETUP`，缺失时明确失败。source ROS/overlay 时临时关闭 nounset，保留 source 失败退出码。
- 按脚本目录、脚本父目录、当前目录、显式 `INSPECTION_WORKSPACE` 的顺序定位本队上传源码。先找 `src/inspection_mission/inspection_mission/probe.py`，找到后仅加载同一工作区的可选 install，把本队包目录加入 `PYTHONPATH` 并用 `python3 -u -m inspection_mission.probe --duration 30` 前台执行。
- 若没有源码但存在已编译 install，保留 `ros2 run` 后备；两者均不存在时给出明确错误。没有引入 gnome-terminal，没有修改 ROS domain/bridge，也没有裁判结束标志。

新增 `tests/test_probe_launch.py` 用真实 Bash、临时本队源码、假 ROS setup 验证工作目录变化、带空格路径、显式工作区、源码来源优先级、可选未定义环境变量、ROS/overlay/用户程序失败退出码及 install 后备。先观察新增启动测试在旧脚本上失败，再修复；最终全项目 `python -m pytest tests -q` 为 **177 passed**（11 项为新增启动边界测试）。这些测试没有 rclpy/Gazebo，不能证明平台已接通。

`dist/inspection_probe_src.zip` 已重新打包，仍保留 `src/` 根目录并排除 Python 缓存；ZIP 完整性检查通过。平台提交与结果仍须单独验证。
