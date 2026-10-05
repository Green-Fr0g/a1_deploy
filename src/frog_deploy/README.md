# frog_deploy

宇树 A1 四足机器人部署运行时。分层参考 M20 部署代码（机器人 I/O 与用户指令解耦），
多机器人加载采用 rl_sar 的自注册工厂模式：新增机器人或后端只需增加文件 + 汇总头一行，
无需修改 `main.cpp`。

硬件层只做两件事：读回机器人状态，采集手柄按键与摇杆数值。
终端键盘在这一层被虚拟成手柄（`0` → 手柄 `X`，方向键 → DPad…），
因此上层只有“手柄”一种输入：状态机把按键解释成目标状态，观测构造把按键/摇杆解释成策略命令。

## 目录结构

```text
include/frog_deploy/
├── types.hpp                       # 通用数据类型（RobotState/RobotCommand/VelocityCmd）
├── config/
│   ├── config_types.hpp            # 配置结构（RobotConfig/PolicyConfig/ModelSpec）
│   └── config_loader.hpp           # SkillConfigLoader（base.yaml + 技能 config.yaml）
├── hardware/
│   ├── gamepad.hpp                 # 手柄按键/摇杆定义（Input::Gamepad / Axis / ButtonCursor）
│   ├── keyboard.hpp                # 终端键盘驱动 + 键盘→手柄映射（Input::key_to_button）
│   ├── robot_hardware.hpp          # RobotInterface + RobotRegister + FROG_REGISTER_ROBOT
│   ├── input_hardware.hpp          # UserCommand + InputHardware + InputRegister + FROG_REGISTER_INPUT
│   ├── sim_interface.hpp           # SimRobot + SimInput（ROS2 仿真）
│   ├── unitree_interface.hpp       # UnitreeRobot + UnitreeInput（UDP 真机）
│   └── all.hpp                     # 硬件汇总注册（新增后端在此加一行）
├── fsm/
│   ├── fsm_base.hpp                # FsmBase / FsmContext / FsmState（含按键游标）
│   ├── fsm_manager.hpp             # FsmManager（定频状态机驱动）
│   ├── fsm_register.hpp            # FsmFactory / FsmRegister + FROG_REGISTER_FSM
│   ├── all.hpp                     # FSM 汇总注册（新增机器人在此加一行）
│   └── robot/a1/a1_fsm.hpp         # A1 状态机：每个技能一个状态类，各自写 CheckChange()
├── policy/
│   ├── policy_manager.hpp          # 技能生命周期（start/run/stop）+ 多模型推理调度
│   ├── observation/
│   │   ├── obs_collection.hpp      # 观测项 + 历史缓冲
│   │   └── command_profile.hpp     # 手柄命令 → 15 维 WTW 命令与步态时钟
│   └── runtime/                    # policy_backend.hpp / policy_register.hpp
├── tui/
│   └── display.hpp                 # 终端输出：message / progress
└── utils/
    ├── utils.hpp                   # 向量辅助（expand/gather/scatter/scale/clip）
    ├── math.hpp                    # 四元数/插值数学
    ├── thread_timer.hpp            # 定频定时器线程
    └── triple_buffer.hpp           # 无锁三缓冲
src/main.cpp                        # 组合根：按名称从注册表创建各组件
```

## 命名约定

- 目录与文件：小写 `snake_case`；文件名与主类名一致（一个文件含多个协作类时用领域名，如 `sim_interface.hpp`）。
- 不在文件名中重复目录名：`tui/display.hpp` 而不是 `tui/tui_display.hpp`。
- 类名 `PascalCase`；接口以 `Interface` 结尾；工厂/注册表以 `Register` / `Factory` 结尾。
- 方法与变量 `snake_case`；宏使用 `FROG_` 前缀 + 全大写下划线。
- 头文件统一 `#pragma once`；项目内 include 统一使用 `<frog_deploy/...>` 且只有 `include/` 一个根目录。
- 汇总头统一命名 `all.hpp`（每个模块目录一个）。

## 运行

```bash
ros2 run frog_deploy frog_deploy_main --robot a1 --mode <sim|real>
```

运行前需 `source install/setup.bash`（`liba1_sim_msgs__rosidl_typesupport_cpp.so`
由消息包提供）。

仿真侧接口约定（由 `a1_mujoco_sim` 提供，名字都带机器人前缀）：

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/a1/command` | `a1_sim_msgs/msg/RobotCmd` | 部署程序下发关节命令 |
| `/a1/state` `/a1/imu` `/a1/depth` | `RobotData` / `ImuData` / `DepthImage` | 仿真回传状态、IMU、深度图 |
| `/a1/reset` | `a1_sim_msgs/srv/Reset` | 复位到 home 关键帧（`R` / 手柄 `Select`） |
| `/a1/pause` `/a1/resume` | `std_srvs/srv/Empty` | 暂停/恢复物理步进（`Enter` / 手柄 `Start`） |

服务不可用时只告警一次（不会刷屏），不影响其它功能。

仿真场景位于 `src/sim/a1_mjcf/mjcf/`，用 `--model` 选择（缺省用 `sim.yaml` 里的 `empty.xml`）：

| 场景 | 内容 |
| --- | --- |
| `empty.xml` | 平地（默认，用于调速度跟踪） |
| `obstacles.xml` | 障碍路线：踩点方块(0.05) → 4 级台阶(0.06~0.24) → 斜坡(11°)+平台 → 横杆(0.10) → 圆柱(r=0.06) → 0.35 m 沟渠；两侧散布乱石 |

```bash
ros2 run a1_mujoco_sim a1_mujoco_sim --model obstacles.xml   # 也可给完整路径
ros2 run a1_mujoco_sim a1_mujoco_sim --headless --model obstacles.xml   # 无显示环境
```

障碍物沿 +x 摆放（机器人初始朝向），起点 1 m 内留平地方便站起与稳定。

## 扩展方式

- 新增机器人：
  1. `include/frog_deploy/fsm/robot/<name>/<name>_fsm.hpp`（文件末尾 `FROG_REGISTER_FSM(...)`）
  2. `policy/<name>/base.yaml`（与各技能子目录）
  3. `include/frog_deploy/fsm/all.hpp` 增加一行 include
- 新增硬件/输入后端：
  1. `include/frog_deploy/hardware/<backend>.hpp`（机器人派生 `RobotInterface`，
     输入派生 `InputHardware`，末尾 `FROG_REGISTER_ROBOT` / `FROG_REGISTER_INPUT` 自注册）
  2. `include/frog_deploy/hardware/all.hpp` 增加一行 include

## 模型输入引用

`models[].inputs` 里每一项是一个运行时张量引用，支持：

| 写法 | 含义 |
| --- | --- |
| `obs` | 按 `observations` / `observations_history` 展平的观测张量 |
| `obs[a:b]` | 观测切片（半开区间，支持负索引）。如 `obs[180:225]` = 最新一帧（45 维） |
| `concat(a, b, ...)` | 把多路张量按顺序拼成一个 |
| `<name>` | 上游模型的 `output`，或硬件注入的输入（如 `depth_image`） |

`concat` 是必需的：除 `extreme_parkour` 的两个模型是真多参数（`forward(obs, latent)`），
其余 TorchScript 的 `forward` 都只接受一个 tensor：
dreamwaq 的 actor 吃 `[vel(3), latent(16), obs(45)]`、walk_these_way 的 body 吃
`[history(2100), latent(2)]`，都要先拼好再喂进去。

## 按键映射

硬件层把键盘映射成手柄按键（`keyboard.hpp` 的 `key_to_button()`），
因此上层（状态机、命令构造）只看到手柄，不需要知道有键盘：

| 键盘 | 手柄 | 说明 |
| --- | --- | --- |
| `0` / `P` | `X` | Passive |
| `1` | `A` | GetUp |
| `2` / `↑` | `DPadUp` | Locomotion |
| `3` / `→` | `DPadRight` | DreamWaQ |
| `4` / `↓` | `DPadDown` | WalkTheseWay |
| `5` / `←` | `DPadLeft` | ExtremeParkour |
| `6` | `LStick` | AmpForHardware |
| `B` | `B` | Disabled |
| `7` `8` `9` | `LB` `RB` `Y` | 命令模式切换（非状态） |
| `W/S` `A/D` `Q/E` | 左/右摇杆 | 无手柄时的速度暂存（`Space` 清零，映射为 `None`） |
| `R` `Enter` | `Select` `Start` | 仿真复位 / 暂停 |

映射表覆盖了状态机用到的每个手柄按键，所以键盘能完整地操作机器人。

## A1 状态机

每个技能一个独立状态类（`RLFSMState<Skill>`），切换条件写在各自的 `CheckChange()` 里，
只看手柄（键盘已由硬件层映射）：

```cpp
std::string CheckChange() override
{
    poll_input();
    if (current_gamepad == Input::Gamepad::X)
        return "RLFSMStatePassive";
    else if (current_gamepad == Input::Gamepad::A)
        return "RLFSMStateGetUp";
    ...
    return state_name_;
}
```

每个状态持有自己的按键游标，`poll_input()` 只做一件事：
`current_gamepad = ctx_.input.poll_button(cursor_)`（取最近一次按下的手柄按键）。

| 状态 | 手柄 | 键盘 | 技能目录 |
| --- | --- | --- | --- |
| `RLFSMStatePassive` | `X` | `0` / `P` | 阻尼模式（初始状态） |
| `RLFSMStateGetUp` | `A` | `1` | 插值到 `default_dof_pos`，完成前忽略技能请求 |
| `RLFSMStateLocomotion` | `DPadUp` | `2` / `↑` | `locomotion` |
| `RLFSMStateDreamWaQ` | `DPadRight` | `3` / `→` | `dreamwaq`（4 模型，5 帧历史） |
| `RLFSMStateWalkTheseWay` | `DPadDown` | `4` / `↓` | `walk_these_way`（2 模型，30 帧历史） |
| `RLFSMStateExtremeParkour` | `DPadLeft` | `5` / `←` | `extreme_parkour`（深度编码器） |
| `RLFSMStateAmpForHardware` | `LStick` | `6` | `amp_for_hardware` |
| `RLFSMStateDisabled` | `B` | `B` | 增益与力矩全零 |

每个技能在推理前采集一次观测并推进历史，推理周期为
`base.yaml` 的 `dt × decimation`。`observations_history` 给出一帧包含的历史帧编号，
`observations_history_priority` 决定按时间还是按观测项拼接。

## 命令观测

`observation/command_profile.hpp` 把“手柄按键 + 摇杆数值”变成各策略需要的 command：

- 速度：左摇杆 Y → `vx`，左摇杆 X → `vy`，右摇杆 X → `yaw`（键盘用 `W/S/A/D/Q/E` 暂存）。
- `commands`(3)：`[vx, vy, yaw] × commands_scale`，供 locomotion / dreamwaq / parkour 使用。
- `walk_these_ways/commands`(15)：完整命令向量
  `[vx, vy, yaw, height, freq, phase, offset, bound, duration,
    footswing, pitch, roll, stance_width, stance_length, aux]`。
  标称值取自训练侧部署脚本（`freq=3`、`trot` 的 `phase=0.5`、`duration=0.5`、
  `footswing=0.08`、`stance_width=0.33`、`stance_length=0.40`）。
- `clock_inputs`(4)：`sin(2π·foot_index)`，`foot_index` 为
  `[g+phase+offset+bound, g+offset, g+bound, g+phase]`，`g` 按 `dt × freq` 滚动，
  与训练侧完全一致。

按键切换命令模式（边缘触发，与训练侧遥控器约定一致）：

| 按键 | 作用 |
| --- | --- |
| `LB`（键盘 `7`） | 循环左摇杆模式：机身高度 → 站立宽度 → 侧向速度 |
| `RB`（键盘 `8`） | 循环右摇杆模式：步频 → 抬腿高度 → 机身俯仰 |
| `Y`（键盘 `9`） | 循环步态：trot → walk → pace → bound（决定 phase/offset/bound） |

按键事件用带序号的队列保存，状态机每个状态一个游标、命令构造再持一个
（`Input::ButtonCursor`），互不影响；进入新状态/重新配置策略时游标对齐到队尾，
不会重放历史按键。

编译需要 `thirdparty/interface_runtime/libtorch`（必需）与 `onnxruntime`，
缺失时可运行 `bash scripts/download_inference_runtime.sh`。真机后端还依赖
`thirdparty/unitree_legged_sdk` 与系统的 `lcm`。
