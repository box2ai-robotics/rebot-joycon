# reBot B601-DM 模型

2026-10-17，按 BoxArm/URDF/BoxArm_B1 的模型目录布局整理。

```text
reBot_B601_DM/
├── reBot_B601_DM.xml       # 机械臂本体，6 轴 + 2 个滑动指爪
├── scene.xml              # 机械臂本体 + 地面、灯光和背景
├── urdf/ReBot_Arm_DM.urdf  # SDK URDF，已替换 100mm 指爪
├── meshes/                # MJCF STL，另含 URDF 的 visual/collision 子目录
├── dh_parameters.json     # 标准 DH，含必需的基座/工具变换
├── dh_parameters.csv
├── LICENSE.mujoco-source
└── validation.json
```

`scene.xml` 包含可控机械臂、地面、灯光和背景；桌面、任务物体、IK 目标及任务相机已移除。整个文件夹可搬移，保留子目录布局。已安装 MuJoCo 的 Python 环境可运行：

```bash
python -m mujoco.viewer --mjcf=1_rebot_control_joycon/URDF/reBot_B601_DM/scene.xml
```

命令工作目录为项目根。本次项目 `.venv` 为空，验证使用 `uv run --no-project --with mujoco==3.16.0 --with numpy` 临时环境；没有恢复或修改项目虚拟环境。

来源：原提取包 `3_模型参数/rebot_b601_dm/`；URDF 及其依赖来自 `00_1History/reBotArm_control_py/urdf/DM/`。URDF 初始为原样复制，网格按原相对路径补齐；现已同步替换左右指爪的视觉/碰撞引用。XML 从原完整场景拆为本体与场景，动力学参数保留。未照搬 BoxArm 的 ROS launch/config、导出日志或控制器配置。

拆分时的历史验证（场景清理前）：MuJoCo 3.16.0 本体/场景加载成功，整目录搬移后再次加载成功；URDF 的 44 处网格引用全部存在。新旧场景各运行 10 步，按物体名称比较的位置最大差为 0。场景拆分会改变关节在 qpos 中的排列，应按关节名查地址，不沿用旧场景的固定索引。本轮未启动 GUI。

本体包含 7 个位置驱动器：`joint1_position` 至 `joint6_position` 及 `gripper_position`。MuJoCo viewer 的 Control 面板可调目标，程序可写入 `data.ctrl`；六轴单位为 rad。夹爪只需一个目标：`data.ctrl[model.actuator("gripper_position").id] = 0.025` 表示每指移动 25 mm、两指开口约 50 mm；0 为闭合，0.05 为最大开口约 100 mm。`parallel_gripper` equality 约束保证 `finger_right = -finger_left`。控制维度 `nu=7`，关节位置维度仍为 `nq=8`，旧的 8 项控制数组需改为 7 项。URDF 已有相同的反向 mimic 关系，保持不变。

六轴 kp=100、力矩范围 ±35 N·m；合并后的夹爪 kp=400、广义驱动力范围 ±40 N（承担双指负载，不是单指夹持力），均使用 dampratio=1。这些是仿真伺服参数，不代表真机额定值。重力下纯 PD 控制会存在静态误差。

2026-10-17 单控制夹爪验证：MuJoCo 3.16.0 本体和场景各运行 8000 步，依次测试闭合、半开、全开、闭合；目标误差小于 1 mm，全过程左右同步误差小于 0.021 mm，无数值警告。证据见 [gripper_control_validation.json](gripper_control_validation.json)。

历史验证（夹爪控制合并前）：2026-10-17 场景恢复地面及驱动器后验证：MuJoCo 3.16.0 本体和场景均加载成功，8 个驱动器对目标 `[0.2, -0.3, -0.3, 0.2, 0.2, 0.2, 0.02, -0.02]` 运行 3000 步（3 s）无数值警告，六轴最大目标误差约 0.0753 rad，双指最大误差约 0.0002 m；地面碰撞已启用。未启动 GUI。

DH 是从 URDF 推导的标准 DH；末端为 end_link，需同时应用 JSON 中的 base_transform/tool_transform。MJCF tcp 另偏移 [-0.105, 0, 0] m。URDF 指爪行程 ±0.0285 m，MJCF 为 ±0.05 m，保留来源差异。DH 推导与 101 组姿态验证详见原提取包说明。

## 100 mm 指爪替换（2026-10-17）

使用项目根 `夹爪-100mm.STL` 替换两侧活动指爪，保留夹爪底座、滑块、齿条和关节；位置驱动器后续合并为 7 个。原 STL 不改写，生成网格使用 `meshes/finger_{left,right}_100mm*.stl`，MJCF 和 URDF 共用。源网格尺寸 32 × 99 × 40，按毫米换算；长轴 Y 对齐模型 X，安装面固定于 X=-0.06480932235717773 m，厚度居中，右侧为对称件。新指尖 X=0.03419067764282227 m，比旧指尖向前延伸约 34.19 mm。对齐依据是原指爪安装面与闭合平面，尚未验证实物安装孔配合。

MJCF 指爪碰撞采用沿长度分成三段的凸包近似，保留滑块和限位块碰撞；URDF 更新指爪组件碰撞网格。原质量/惯量及关节行程不变，未测量新件质量。`end_link`、DH 与现有 `tcp` 坐标保持原定义，`tcp` 不是新指尖坐标。

指爪替换时的历史验证（控制合并前）：MuJoCo 3.16.0 本体和场景各运行 6000 步，测试张开、闭合、最大张开，双指目标误差小于 1 mm，无数值警告；URDF 44 处网格引用有效。已检查离屏装配预览，未启动 GUI。来源哈希、变换矩阵及验证结果见 [gripper_100mm_validation.json](gripper_100mm_validation.json)，预览见 [gripper_100mm_preview.png](gripper_100mm_preview.png)。
