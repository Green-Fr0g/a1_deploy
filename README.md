# A1 MuJoCo 仿真环境

这是 Unitree A1 的第一版 MuJoCo + ROS 2 仿真环境。模型文件位于 `src/sim/a1_mjcf/mjcf`，仿真节点位于 `src/sim/a1_mujoco_sim`。

## 环境要求

- Ubuntu 20.04/22.04
- ROS 2（建议使用与当前系统匹配的发行版）
- Python 3.10、`uv` 和 `colcon`
- 支持 OpenGL 的桌面环境，用于 MuJoCo viewer

ROS 2 的 `rclpy` 和本地消息包不是 PyPI 包，因此不放入 `pyproject.toml`。它们必须来自已安装并 source 的 ROS 工作区。

在仓库根目录执行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
uv venv --python 3.10 --system-site-packages --clear
uv sync
```

ROS 2 Humble 的 `rclpy` 使用 Python 3.10 编译，因此不能使用 Python 3.11/3.12 的 uv 环境。`--system-site-packages` 用于让 uv 环境访问 ROS 2 的 `rclpy`。如果系统还没有 Tk/Pillow 的桌面支持，可以安装：

```bash
sudo apt install python3-tk
```

依赖版本由 `uv.lock` 固定，其他机器安装时直接执行 `uv sync` 即可。

## 构建 ROS 2 工作区

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
uv run python -c "import mujoco, numpy, yaml; print(mujoco.__version__)"
colcon build --merge-install --symlink-install --packages-select a1_sim_msgs a1_mjcf a1_mujoco_sim
source install/setup.bash
```

如果之前已经创建过 Python 3.12 的 `.venv`，必须重新创建：

```bash
uv venv --python 3.10 --system-site-packages --clear
uv sync
```

## 启动仿真

推荐直接使用源码入口启动：

```bash
uv run python src/sim/a1_mujoco_sim/scripts/a1_mujoco_sim.py --camera --depth
```

也可以用构建后的入口：

```bash
source install/setup.bash
ros2 run a1_mujoco_sim a1_mujoco_sim --camera
```

参数说明：

- 默认会打开 MuJoCo 窗口，并且是自由视角，不会自动跟随。
- `-c, --camera`：只改变视角行为，让镜头从机器人后方跟随 trunk 位置和 yaw 朝向。
- `-d, --depth`：额外打开深度图窗口，显示 0 到 5 米（越近越亮）。窗口位于屏幕右下角并保持置顶，避免被 MuJoCo 窗口遮挡。它只影响可视化，`/a1/depth` 话题始终发布。

启动时终端会直接打印当前状态，例如：

```text
Viewer: open, camera follows the robot
Depth window: open, bottom right corner and kept on top
```
- `--headless`：不打开 MuJoCo 窗口，适用于无显示器的服务器。不能与 `--camera` 同时使用。

仿真频率由 `config/sim.yaml` 中的 `sim_frequency` 控制，当前为 500 Hz。该值会覆盖 XML 的 `model.opt.timestep`，因此仿真步长为 `1 / sim_frequency`。

`sim.yaml` 只列出真正需要按环境调整的参数。深度窗口开关（`publish_depth`）、IMU 开关与坐标系（`publish_imu`、`imu_frame_id`）、复位关键帧（`reset_keyframe`）、深度相机名（`depth_camera`）等保持代码默认值（见 `a1_mujoco_sim/config.py`）；需要覆盖时，在 `sim.yaml` 中显式添加同名键即可。

## 退出行为

以下三种方式等价，都会走同一条优雅退出流程（停止物理循环、关闭窗口和渲染器、关闭 ROS、结束执行器线程）：

- 直接关闭 MuJoCo 窗口
- 在终端按一次 `Ctrl+C`
- 向进程发送 `SIGTERM`

`Ctrl+C` 只需要按一次，不会卡住终端。
关闭深度图窗口只停止该窗口的刷新，仿真会继续运行。

## 代码结构

```
src/sim/a1_mujoco_sim/
  a1_mujoco_sim/
    config.py   配置解析、路径解析与校验
    model.py    MuJoCo 模型封装（执行器映射、状态读取、reset）
    node.py     ROS 话题、订阅与服务
    viewer.py   跟随相机行为
    depth.py    深度图窗口
    app.py      主循环、信号处理与退出流程
    cli.py      命令行参数入口
  scripts/
    a1_mujoco_sim.py  启动器，兼容源码和安装两种运行方式
  config/
    sim.yaml    可调参数，其余保持代码默认值
```

## 相机参数

XML 中深度相机定义为：

```xml
<camera name="depth_camera" pos="0.27 0 0.03" xyaxes="0 -1 0 0 0 1" fovy="56.4849"/>
```

`xyaxes` 显式指定相机的 X/Y 轴，避免用 euler 角度时图像被滚转 90°：

| 图像方向 | 机器人坐标轴 |
|---|---|
| 图像向右 | `-Y`（机器人右侧） |
| 图像向上 | `+Z`（世界上方） |
| 光轴向前 | `+X`（机器人前方） |

MuJoCo 的 `fovy` 是垂直视场角，因此垂直视场范围为 **56.4849°**，光轴上下各 **28.24245°**。当前深度图分辨率为 `106 x 60`，水平视场角约为 **87.00°**。

因此深度图应当：下边缘为近处地面（亮），上边缘为远处（暗），地平线大致在画面中部。

## ROS 接口

默认接口如下：

- 订阅 `/a1/command`：`a1_sim_msgs/msg/RobotCmd`
- 发布 `/a1/state`：`a1_sim_msgs/msg/RobotData`（12 个关节的位置/速度/力矩）
- 发布 `/a1/imu`：`sensor_msgs/msg/Imu`（默认 200 Hz）
- 发布 `/a1/depth`：`a1_sim_msgs/msg/DepthImage`
- 服务 `/a1/reset`：`a1_sim_msgs/srv/Reset`

### IMU 话题

`/a1/imu` 使用 ROS 2 标准的 `sensor_msgs/Imu`，数据来自 XML 中 trunk 上的 `imu` site：

| 字段 | 对应 MuJoCo 传感器 | 说明 |
|---|---|---|
| `orientation` (w,x,y,z) | `imu_quat` | IMU 在世界系下的姿态 |
| `angular_velocity` | `imu_gyro` | 角速度，IMU 坐标系，rad/s |
| `linear_acceleration` | `imu_acc` | 线加速度，IMU 坐标系，m/s²，**包含重力** |

模型传感器无噪声，因此三个 covariance 字段保持为 0。发布频率由 `sim.yaml` 的 `imu_publish_frequency` 控制；坐标系 `imu` 与发布开关属于代码默认值，需要覆盖时在 `sim.yaml` 中显式添加 `imu_frame_id` 或 `publish_imu`。

命令中的 `torque` 始终生效；`kp` 或 `kd` 非零时，额外使用位置/速度 PD 项。
`command_timeout` 为控制看门狗：超过该时间没有收到新命令就会将力矩清零；设为 0 或负数则关闭看门狗。

## 常见问题

### 找不到 `rclpy` 或 `a1_sim_msgs`

确认执行过：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

并确认 uv 环境是通过以下方式创建的：

```bash
uv venv --python 3.10 --system-site-packages
```

### viewer 无法启动

MuJoCo viewer 需要图形桌面和 OpenGL。程序会在启动前检查 `DISPLAY`/`WAYLAND_DISPLAY`，缺失时会直接提示使用 `--headless`（而不是让 MuJoCo 硬退出进程）。

### 深度窗口无法启动

深度窗口使用 Tkinter + Pillow。如果运行在无桌面的 SSH 环境中，不要加 `--depth`；ROS 深度话题仍会继续发布。
