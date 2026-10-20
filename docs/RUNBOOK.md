# 操作与验证记录

本文保留历史验证证据，包含后来撤销的方案及当时的路径、章节编号。当前安装与运行请参阅[根目录说明](../README_zh.md)及[学习流程](../Learning_Model/README_zh.md)；历史日志仅保留在本地，不随仓库发布。

## 2026-10-21：统一根目录 Python 环境

按用户要求以 Learning_Model 为准，根依赖的 NumPy 从 `1.26.4` 改为 `>=2.0,<2.3`，与本地 LeRobot 声明一致。examples 和学习流程统一使用根目录 `.venv`；策略启动器及中英文 README 路径同步修改。学习依赖与根依赖在同一安装命令中解析；Joy-Con 包仍按需安装。旧 `.venv-rebot` 未删除。

本轮在 Windows 主机使用 `uv pip compile` 针对 Linux x86_64 / Python 3.12 分别解析基础依赖、基础依赖加 LeRobot 的 `rebot-joycon,hardware,training,test` extras、以及追加本地 Joy-Con 包，三组均成功；合并结果包含 NumPy 2.2.6、MuJoCo 3.16.0、roboticstoolbox-python 1.4.4、SciPy 1.17.1。已有 Linux 学习环境元数据也记录 NumPy 2.2.6 与相同版本的运动学及 MuJoCo 包。

策略启动器使用模拟文件检查及模拟子进程验证：选择根目录解释器、保留 LeRobot 工作目录、缺文件时不启动子进程。双语命令一致，本地文档链接检查通过。当前主机不能运行工作区内的 Linux 虚拟环境，且无可用 WSL，因此没有升级实际 Linux 环境、执行完整控制回归或操作真机。Linux 上须按新版安装命令更新根 `.venv`，执行依赖检查及两套离线测试后再使用；本轮依赖解析成功不等于运行验证。

## 2026-10-20：遥操命令缺少 `teleop` / `robot` 配置

用户旧命令使用 `"${REBOT_ARGS[@]}"`。若当前 Bash 未定义该数组或数组为空，该处展开为零个参数，Python 只收到 `--fps`、`--teleop_time_s`，无法构造必填的 `TeleoperateConfig.teleop` 和 `.robot`。激活虚拟环境不会定义该数组；未读取用户交互终端的变量，因此“当前终端数组未定义/为空”是最符合命令与错误的原因判断。

已将 [Learning_Model README](../Learning_Model/README.md) 第 3 节改为包含全部 `--robot.*`、`--teleop.*` 参数的独立命令，清理旧数组引用，增加缺字段排查说明。没有修改解析器或控制代码。

离线验证：提取 README 实际命令，经 `bash -n` 检查；在隔离 Bash 中将 `python` 替换为参数输出函数，捕获展开后的参数，再用 `.venv-rebot` 的 `draccus.parse(TeleoperateConfig, ...)` 仅解析配置。确认机器人为 `rebot_b601_follower` / `examples`、遥操为 `joycon` / `use_example6=true`、两端校准路径一致且文件存在、fps=30、时长=60。仅传 fps/时长时复现完全相同的缺字段错误。验证退出码 0，未实例化机器人/手柄、未连接设备或启动实机遥操。

## 2026-10-20：LeRobot 回中离线自检失败修复

用户提供的三文件 pytest 日志为 1 failed、127 passed；失败发生于 `test_calibrated_robot_home_gains_and_timeout`，不是实机遥操启动日志。单独运行该用例已复现：模拟 `_read_feedback` 只返回 `SimpleNamespace(pos=...)`，而 `_send_gripper` 必须检查 `pos/vel/torq`，缺少 `vel` 触发 `AttributeError`，经既有失能路径转为 `RuntimeError("Invalid gripper command or torque feedback")`。本地安装的 MotorBridge `models.MotorState` 和 `core.get_state()` 均提供这三个字段，缺字段来自该测试替身。

另一个过期断言将七路都视作电机端位置增益：当前旧 MotorBridge 后端的 MIT 夹爪在主机计算带回中倍率的 PD 并限制合成力矩，发送电机端 Kp/Kd=0；六轴仍发送非零增益。此处与默认 examples 后端的 SDK 夹爪控制不同。本轮仅修复测试，不改变任一后端的运行控制、保护阈值或校准。

`test_joycon.py` 补齐零速度/零实测力矩反馈，将回中用例扩为到位、误差 0.05 rad 与速度 0.2 rad/s（预期力矩 0.76 N·m）、触及 3 N·m 命令限幅三种情况，保留六轴回中增益、超时失能、非有限位置拒绝检查。`test_rebot_b601_follower.py` 增加三种夹爪模式分别缺少 `vel`/`torq` 的六项回归，确认只发送既有停止命令并失能，不把缺失字段默认为零。

在 `Learning_Model/lerobot/` 执行：

```bash
uv run --no-project --python .venv-rebot/bin/python python -m pytest \
  tests/teleoperators/test_joycon.py \
  tests/teleoperators/test_rebot_example6.py \
  tests/robots/test_rebot_b601_follower.py -q
```

结果：**136 passed，3 warnings，11.11 秒，退出码 0**。警告为既有 pgraph `DVertex/UVertex` 弃用提示。完整测试使用模拟硬件与窗口，未启动实机遥操、连接机械臂或修改校准；结果只证明这组离线自检通过。

## 2026-10-20：用户实机验证成功后统一所有实机入口

用户报告上一轮 SDK 保护测试成功，并明确选择保留 1–3 仿真预览。将已验证算法提取为 `examples/gripper_protection.py`，SDK 和 LeRobot MotorBridge 复用，阈值仍为 1 N·m。4/5 每次控制与回中后回读 SDK 目标，避免键盘输入积累未执行目标。LeRobot examples 后端继续直接调用 SDK；MotorBridge 的 MIT、MIT_IMPEDANCE、FORCE_POS 在下发前处理力矩反馈 `state.torq`，返回修正后动作，阻抗目标被修正时清除旧目标速度历史，双臂锁存状态独立。原模式/增益/命令限幅保持，反馈无效触发失能。1–3 仅补充边界说明，没有改成动力学仿真。

验证：根目录 `.venv/bin/python -m unittest discover -s examples -p 'test_*.py' -q`：24 项通过。LeRobot 目录 `uv run --no-project --python .venv-rebot/bin/python python -m pytest tests/robots/test_rebot_b601_follower.py tests/teleoperators/test_rebot_example6.py -q`：108 项通过，3 条既有第三方弃用警告。新增覆盖键盘持续输入/立即换向、所有后端正负超限、低于阈值仍锁存、反向卸力、无效力矩、双臂状态独立、LeRobot 实际返回目标及 IK 失败时 Joy-Con 保护后插补起点。未连接真机；用户成功反馈针对上轮实现，不能代替此次 MotorBridge 接入的实机验证。重启相关实机控制/遥操进程生效。

## 2026-10-20：夹爪 1 N·m 反馈触发的位置目标保护

共享 SDK 第七路在 `abs(feedback_torque)>1.0` 时将目标收回当前反馈位置，锁存阻止继续向受阻方向运动；持续超限只允许边界向卸力方向移动，反向指令允许卸力并在力矩回落后解除。未改变 Kp/Kd 或六轴控制。手柄 `last_sent` 已从 SDK 实际目标更新，无需修改插补器。回中同样不能覆盖保护。无效力矩反馈触发既有失能路径，新增保护/解除日志。

验证：根目录 `.venv/bin/python -m unittest discover -s examples -p 'test_*.py' -q` 23 项通过；LeRobot 目录 `uv run --no-project --python .venv-rebot/bin/python python -m pytest tests/teleoperators/test_rebot_example6.py -q` 28 项通过（3 条既有弃用警告）。覆盖正负超限、双方向校准映射、低于阈值保持锁存、超限期间反向松开、目标边界不前移、精确阈值、NaN/Inf、回中与六轴隔离。未启动真机；需重启控制进程。此为主机反馈监督，不能保证瞬时输出≤1 N·m，也未取消 Kd 产生的阻尼力矩。

## 2026-10-20：按用户要求回退两轮，恢复插补位置控制

撤销上一轮 6 号夹爪 1 N·m 纯力矩分支及再上一轮的 0/1 直接端点控制。Joy-Con 重新输出校准开闭角；6 号以全行程/秒插补，单步 dt≤0.1 秒，失败 IK 保持六轴并继续夹爪插补。SDK 七路统一电机端位置闭环：夹爪 Kp=2、Kd=1、tau=0，回中 Kp=1.6。原 3 N·m 主机力矩限幅未恢复。保留运行日志、当前校准和已修正的键盘测试动态行程时长，不回退无关用户数据。

验证：项目根 `.venv/bin/python -m unittest discover -s examples -p 'test_*.py' -q`：19 项通过；`Learning_Model/lerobot/` 下 `uv run --no-project --python .venv-rebot/bin/python python -m pytest tests/teleoperators/test_rebot_example6.py -q`：28 项通过，3 条既有第三方弃用警告。覆盖双向插补、换向、限位、长停顿、失败 IK、SDK 报文增益/位置、生命周期与日志。修改 Python 文件 AST 检查通过；运行源码中不再含二态纯力矩配置/分支。未操作真机，需重启控制进程生效。

下方两态/纯力矩记录均为已撤销的历史状态。

## 2026-10-20：6 号夹爪切换 1 N·m 纯力矩

按用户“tau=1NM，kpkd=0”，6 号 `attach` 为该 SDK 实例设置 `gripper_binary_torque_nm=1.0`。第七路按校准开闭方向发送力矩，Kp=Kd=0；方向只依赖二态端点，不根据实时位置误差翻转。启动及正常退出回中保持纯力矩闭合，显式回中 Kp 也不会覆盖第七路的零增益。六轴位置控制保持原逻辑，普通 SDK/4/5/独立策略默认仍为位置模式。LeRobot 手柄遥操/采集共用 attach，因此同步生效。

力矩编码：DM4310 12 位 ±10 N·m 量程采用向下取整，发送前从幅值预留一格 `20/4095`，本次 API 值约 ±0.995116 N·m，解码后正向约 +0.993895、负向约 −0.998779 N·m，均不超过 1 N·m。未修改底层 CAN 编码或电机参数。日志 `gripper_command_attempt` / `joint_command_attempt` 记录真实发送的零增益、前馈力矩与 `binary_torque` 模式。

运行行为：选定开/闭状态后持续对应方向施力，到校准端点不自动撤力，状态切换才反向，失能后停止驱动。端点角度用于表示状态、预览和动作数据，不再是夹爪位置伺服目标；力矩命令限幅不是速度限制，也不保证接触瞬时峰值/指尖力。反馈等待与超时失能保留；没有恢复旧主机位置 PD/滤波。

离线验证：

- 项目根 `.venv/bin/python -m unittest discover -s examples -p 'test_*.py' -q`：**22 项通过**。
- `Learning_Model/lerobot/` 下 `uv run --no-project --python .venv-rebot/bin/python python -m pytest tests/teleoperators/test_rebot_example6.py -q`：**35 passed**，3 条既有第三方弃用警告。

覆盖模拟串口真实 MIT 帧解码、两个校准方向/开闭/越端点反馈均保持预期力矩符号和零增益、双向 ≤1 N·m、启动/退出回中、显式 Kp 不覆盖、非法力矩/非二态目标拒绝，及真实 SDK 与 6 号控制器接入；原默认位置控制与日志回归通过。未连接真机、使能电机、写校准或改 Flash。实际速度/抓持效果未验证。

## 2026-10-20：6 号 Joy-Con 夹爪改成 0/1 两态

`Example6Controller` 配置 Joy-Con 的闭合值为 0、张开值为 1、初始状态为 0；ZR 沿用原有切换行为。取消 6 号的一秒角度插补，正常控制阶段每次直接映射到校准闭合/张开端点。状态变化写入 `joycon_gripper_state` 日志。SDK 仍接收角度并采用电机端位置闭环，未切换纯力矩模式。六轴 IK、启动 40 轮和退出回中保持原规则；失败 IK 仍允许夹爪开闭。共享控制器的 LeRobot 遥操/采集同步使用二态输入，返回动作和数据仍为角度。3 号仿真保留原插补。

离线验证：

- 项目根 `.venv/bin/python -m unittest discover -s examples -p 'test_*.py' -q`：19 项通过。
- `Learning_Model/lerobot/` 下 `uv run --no-project --python .venv-rebot/bin/python python -m pytest tests/teleoperators/test_rebot_example6.py -q`：当时的 33 项全部通过；随后新增启动期间张开请求测试，使用同一命令追加 `-k binary_open_request_waits`：1 项通过、33 项未选。两次均有 3 条既有第三方弃用警告。

覆盖二态端点立即切换/重复状态保持、非法中间值与非有限值拒绝、启动回中时序、IK 失败时六轴保持和夹爪两向目标、独立示例/LeRobot 一致性、共享 SDK 与日志。额外回归初次有 3 个失败断言：当前校准闭合角已为 3.017662287 rad（文件时间 19:13:47），旧键盘测试写死 300 帧不足以完成行程；保留校准，测试改为按实际行程与每帧增量计算时长后全部通过，4/5 运行逻辑未改。

未连接串口/HID、操作真机或修改校准；端点目标直接切换不等于物理瞬间到位，实际速度与接触力未验证。

## 2026-10-20：参考 BoxArm SDK 恢复电机端位置闭环与自动日志

用户确认使用位置闭环、原 3 N·m 软件限幅不再适用。夹爪从主机 PD/滤波/限幅改成发送 q/Kp/Kd；普通 Kp=2、回中 Kp=1.6、Kd=1。六轴增益、校准、模式确认、反馈超时失能未改。运行 4 号的原命令即可；下次进程启动会在项目根 `logs/日期_时间_微秒_PID/` 生成 `console.log`、`events.jsonl`。真实启动未执行。

项目根执行：

```bash
.venv/bin/python -m unittest discover -s examples -p 'test_*.py' -q
```

结果：**19 tests，全部通过**（17 项控制/生命周期 + 2 项日志）。旧主机力矩滤波/3 N·m 限幅用例随实现移除，新增实际 MIT 帧解码验证双向端点、方向、非零位置增益和阻尼、回中倍率、不同主机周期/反馈下位置报文不变。保留键盘目标、限位、反馈超时和非有限状态失能测试。日志测试使用临时目录及独立子进程，验证中文输出、未捕获异常堆栈、一次性初始化、退出排空及目标快照；测试日志不冒充现场日志。

`Learning_Model/lerobot/` 执行：

```bash
uv run --no-project --python .venv-rebot/bin/python python -m pytest tests/teleoperators/test_rebot_example6.py -q
```

结果：**28 passed**，3 条既有第三方弃用警告。共享参数对照测试更新为位置/Kp/Kd/零前馈检查，验证直接 SDK、采集遥操与推理封装一致。没有连接串口/HID、使能真机或写电机 Flash；报文验证不能替代实机振荡验证。

本节取代此前所有共享 SDK 主机力矩闭环、3/6 N·m 限幅、滤波时间常数的当前行为描述；旧记录保留为历史。

## 2026-10-20：持续输入顿挫与 Joy-Con 夹爪暂停修复

共享 SDK 将 2 N·m/s 慢增力/立即减力改为 20 ms 对称指数力矩滤波。正常换向与长循环间隔不再将有效力矩清零；反馈检查仍在发送前执行，缺失/超时仍失能。6 号在 IK 失败时保持六轴最后发送目标，继续夹爪插补与发送，并把 SDK 发送移至目标预览之前。Kp/Kd、6 N·m 限幅、校准和底层 CAN 未改；大误差启动/回中的力矩响应比旧实现更快。程序仍使用同线程串口/IK/窗口，本轮不保证实时发送频率。

在项目根运行：

```bash
.venv/bin/python -m unittest discover -s examples -p 'test_rebot_keyboard_real.py' -v
```

结果：**22 tests，全部通过**。覆盖持续输入示意摩擦负载、换向和减力连续性、10–100 Hz 相同响应时间、长循环后有效反馈接续、反馈过期拒绝运动/失能、非有限状态拒绝、七路报文和双向 6 N·m 限幅。示意模型参数及采样区间见测试源码，不作为实物辨识结果。将新增回归加载到修改前 SDK，可复现速度二次峰值与间隔后清零等失败。

在 `Learning_Model/lerobot/` 运行：

```bash
uv run --no-project --python .venv-rebot/bin/python python -m pytest \
  tests/teleoperators/test_rebot_example6.py -q
```

结果：**28 passed**，3 条既有第三方弃用警告。包含持续 IK 失败时逐轮发送并完成双向夹爪目标，六轴保持、无 NaN 下发，以及遥操/采集/推理对共享 SDK 的一致性。所有测试采用模拟硬件与窗口，未操作真机；真实顺滑度和夹持负载仍需现场确认。原有启动 `t=40` 交接轮不发送的规则未改。

变更 Python 文件的 AST、文档新增本地链接和新增行空白检查通过；两份校准、校准加载器与 `Motor_CAN.py` 的 SHA-256 与本轮开始一致。尝试 `uv run --no-project --python .venv-rebot/bin/python python -m ruff check tests/teleoperators/test_rebot_example6.py`，环境返回 `No module named ruff`，因此 Ruff 未执行，未额外安装依赖。

## 2026-10-20：实测闭合端点与夹爪制动修复

`ReBotCalibration` 原来把诊断开始时的实测闭合角 +0.797093 rad 裁成名义上限 0，现以实测闭合角和配置张开角确定夹爪限位。共享 SDK 修正制动力矩反复清零、减力延迟和慢循环渐变不足；不改 6 N·m 上限、Kp/Kd 或六轴控制。4/5/6 与 LeRobot examples 后端重启后加载共享修改，1/2/3 仿真也使用修正后的端点。

在项目根运行：

```bash
.venv/bin/python -m unittest discover -s examples -p 'test_rebot_keyboard_real.py' -v
```

结果：**19 tests，全部通过，约 2.9 秒**。新增用例在旧实现上先复现了 13 个失败子例，修复后通过。覆盖实测正/负闭合角、反向电机、非法端点、4/5 按键完整开合与松键保持、SDK 回中/限位、持续制动、立即减力、反向渐变、10/20/100 Hz 力矩一致性与长停顿重启；原七路报文顺序及双向 6 N·m 编码限幅仍通过。

在 `Learning_Model/lerobot/` 运行：

```bash
uv run --no-project --python .venv-rebot/bin/python python -m pytest \
  tests/teleoperators/test_rebot_example6.py -q
```

结果：**27 passed，约 10.9 秒**，3 条第三方弃用警告。涵盖示例 6 目标插补、共享窗口控制、采集及推理封装的一致性；LeRobot 源码和测试未改。

末次核对发现 `.encoders.json` 在工作期间被外部更新（文件时间 2026-10-20 17:09:53 +08:00），闭合角变为 +0.957312882 rad，六轴零偏也发生变化。保留新文件，并重新运行上述两组测试；闭合角始终从文件加载，未写死旧值。

两组均使用模拟设备与窗口，未连接串口/HID、使能电机或运行 REAL 主程序。实际夹爪动力学、机械摩擦及现场 GUI/串口周期未验证，不能据此保证实机顿挫已完全消失。本次修复未写校准文件；`rebot.json` 内容和 `Motor_CAN.py` 的 SHA-256 与开始时一致。源码证据与边界见 [夹爪控制报告](research/gripper_control.md)。

## 2026-10-20：4 号退出发送 100 次回中命令

`examples/4_ReBot_B601_DM_Key_qpos_REAL.py` 的退出清理在停止键盘监听后，连续调用 100 次 `Go_Home_Pose()`，再失能并关闭串口；关闭窗口、Ctrl+C 及进入该清理段的运行异常均适用。回中抛错仍尝试失能，失能抛错仍关闭串口。保留固定次数方式，不增加到位判断。

在项目根执行以下离线回归，10 项全部通过：

```bash
.venv/bin/python -m unittest discover -s examples -p 'test_rebot_keyboard_real.py' -v
```

另通过 AST 提取实际退出清理段，用模拟对象验证正常退出、回中失败、失能失败、机械臂未初始化四种情况；正常路径恰好 100 次回中后失能/关闭串口，异常路径仍执行后续清理。语法检查通过。检查未连接串口、操作真机或打开真实窗口，不证明实际回中到位。

## 2026-10-20：3 号夹爪插补与 4 号顿挫诊断

工作目录：`1_rebot_control_joycon/`。

已执行：

```bash
.venv/bin/python /tmp/rebot_gripper_check_20261020.py
```

该文件为本轮临时验证脚本，系统清理后可能不存在，不是项目长期测试入口。当前 `examples/` 中没有历史 `test_rebot_*.py` 源文件，未用空的 unittest discover 结果代替验证。

验证方法：运行完整 3 号脚本，替换 Joy-Con、窗口、时钟与 IK 返回值，保留真实 MuJoCo 模型及步进；与 6 号控制器跳过启动回中阶段后的目标逐帧比较。另用 `ReBot(if_connect_directly=False)`、模拟发送器及固定反馈执行真实 SDK 控制函数。没有连接 HID/串口、使能电机或打开真实窗口。

六组检查全部通过：

1. 完整张开、闭合各约 1 秒，支持反向、超范围输入裁剪及端点保持；3/6 号逐帧一致。
2. 零/负时间差不前进；长停顿后单步最多前进全行程 10%。
3. IK 失败保持上次成功目标；恢复后不补跳失败期间的位移。
4. 拒绝 NaN、Inf 夹爪目标。
5. 固定大误差、10 ms 循环时力矩连续渐变至约 −2.995116 N·m，没有周期停机；50 ms 循环时 2 秒仅达到 −1.6 N·m。
6. 固定 −0.1 rad 误差、零速度反馈时，3/4 N·m 两种限幅均输出约 −0.18 N·m；仅提高限幅未改变轨迹。

另外，3/4/6 号及共享 SDK 的 AST 语法检查通过。以上未模拟真实夹爪动力学，不能确认现场顿挫原因，也不能作为上调真机力矩的验证。

可重复的最小语法检查（只读源码，无设备副作用）：

```bash
.venv/bin/python - <<'PY'
import ast
from pathlib import Path
for name in (
    "3_ReBot_B601_DM_Joycon_Gpos_SIM.py",
    "4_ReBot_B601_DM_Key_qpos_REAL.py",
    "6_ReBot_B601_DM_Joycon_Gpos_REAL.py",
    "ReBot_SDK.py",
):
    path = Path("examples") / name
    ast.parse(path.read_text(), filename=str(path))
print("4 个控制文件语法检查通过")
PY
```

结论与后续观测项见 [夹爪控制诊断](research/gripper_control.md)。

## 2026-10-20 第二轮：4/5 号控制修复与 6 N·m 上限

在 `1_rebot_control_joycon/` 执行：

```bash
.venv/bin/python -m unittest discover -s examples -p 'test_rebot_keyboard_real.py' -v
```

结果：10 tests，全部通过，约 2.1 秒。使用真实 MuJoCo 模型与脚本主循环，模拟键盘、窗口、时钟与 SDK；连续小步测试使用真实 IK。SDK 测试使用真实 `Motor_CAN` 编解码及内存串口，验证每轮七次查询 / 七次 MIT 发送，双向编码力矩不超过 6 N·m 且能达到 5.98 N·m 以上。测试拦截真实 `serial.Serial`，不连接硬件。

在 `1_rebot_control_joycon/Learning_Model/lerobot/` 执行：

```bash
uv run --no-project --python .venv-rebot/bin/python python -m pytest \
  tests/teleoperators/test_rebot_example6.py \
  -k 'actual_sdk_arguments_are_used_without_gain_reimplementation' -q
```

结果：2 passed，25 deselected，3 条既有第三方弃用警告。更新此测试的共享默认上限预期为 6.0，并将对照长度从 200 帧延长至 400 帧，覆盖 2 N·m/s 渐变到新上限的过程。LeRobot 源码未修改，旧 MotorBridge 的独立 3.0 默认值未修改。

用户回中误差 0.1003 rad（第 4 轴）、0.1553 rad（夹爪）现可通过分路容差；额外测试确认六轴 0.2 rad 或夹爪 0.3 rad 仍超时失能。5 号在连续 30 次 IK 失败时仍产生 30 次七路控制调用，夹爪继续运动；25 次真实 IK 上移小步连续通过。

控制脚本没有进行真机运行验证。4/5 正常循环设定 100 Hz、窗口最多约 30 Hz；实际频率由新增每秒日志观察。5 号正常退出回中 100 次并按 10 ms 间隔发送，异常/中断直接失能。配置和修改内容见 [诊断报告](research/gripper_control.md)。


## 2026-10-20 examples 以 Learning_Model 为准对齐

基准：`Learning_Model/lerobot/src/lerobot/scripts/lerobot_teleoperate.py` 的正常结束回中/异常直接清理流程，以及默认加载的 `Example6Controller.attach/step`。4/5 号引用共享回中轮数，移除各自的误差门槛或时间门槛；4/6 号异常不再追加回中；5 号退出回中移除独立 sleep。SDK、校准和 Learning_Model 源码未改。

验证（全部模拟设备，未连接真机）：

- 项目根 `.venv/bin/python -m unittest discover -s examples -p 'test_rebot_keyboard_real.py' -q`：21 项通过。
- `Learning_Model/lerobot` 下 `uv run --no-project --python .venv-rebot/bin/python python -m pytest tests/teleoperators/test_rebot_example6.py -q`：28 项通过，3 条第三方弃用警告。

本节取代旧记录中 4 号到位门槛、5 号按时间回中、4/6 号异常退出也回中的描述。固定回中轮数并不验证真实到位。

## 2026-10-20 夹爪上限调回 3 N·m

按用户最新要求将 `examples/ReBot_SDK.py` 的 `GRIPPER_MAX_TORQUE_NM` 从 6.0 改为 3.0；真机 examples 4/5/6 与 Learning_Model 默认 examples 后端共用。滤波、反馈超时处理和校准不变，需重启控制进程生效。上文 6 N·m 记录为历史状态。

离线验证：项目根 `.venv/bin/python -m unittest discover -s examples -p 'test_rebot_keyboard_real.py' -q`：21 项通过，覆盖双向 MIT 报文限幅及 10–100 Hz 响应。Learning_Model/lerobot 下 `uv run --no-project --python .venv-rebot/bin/python python -m pytest tests/teleoperators/test_rebot_example6.py -k actual_sdk_arguments_are_used_without_gain_reimplementation -q`：2 项通过、26 项未选，3 条既有第三方弃用警告；验证直接 SDK 与策略封装使用相同 3 N·m 上限。未连接真机。
