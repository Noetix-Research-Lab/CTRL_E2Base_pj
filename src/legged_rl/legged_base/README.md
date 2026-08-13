# legged_base

## Introduction

`legged_base` 基于 `legged_control` 的硬件接口组织方式，提供一套只依赖 ROS / ros_control 的腿式机器人基础通信框架，不依赖 OCS2 等复杂控制栈。

该目录主要包含三部分：

- `legged_common`：公共硬件接口与工具脚本。
- `legged_hw`：真实硬件侧的 `RobotHW` 抽象和控制循环。
- `legged_gazebo`：Gazebo 仿真侧的硬件接口插件、启动文件、仿真配置和世界文件。

## Directory

```text
legged_base/
├── legged_common/
├── legged_hw/
└── legged_gazebo/
```

## legged_common

公共 ROS 包，主要提供自定义 `hardware_interface`，供真实硬件和 Gazebo 仿真共同使用。

| 文件 | 作用 |
| --- | --- |
| `CMakeLists.txt` | 定义 `legged_common` catkin 包。该包主要是头文件接口库，并安装公共头文件和 `generate_urdf.sh` 脚本。 |
| `package.xml` | 声明 ROS 包元信息和依赖，例如 `roscpp`、`hardware_interface`。 |
| `include/legged_common/hardware_interface/HybridJointInterface.h` | 定义混合关节控制接口。`HybridJointHandle` 继承 `JointStateHandle`，在关节状态基础上额外提供 `pos_des`、`vel_des`、`kp`、`kd`、`ff` 五类命令字段，用于位置/速度/PD/前馈混合控制。`HybridJointInterface` 使用 `ClaimResources`，控制器加载后会声明占用对应关节资源。 |
| `include/legged_common/hardware_interface/ContactSensorInterface.h` | 定义足端/连杆接触传感器接口。`ContactSensorHandle` 提供 `isContact()` 查询，`ContactSensorInterface` 使用 `DontClaimResources`，适合作为只读传感器资源给控制器访问。 |
| `include/legged_common/output_color.h` | 定义终端输出颜色字符串，例如 `normal`、`green`、`yellow`、`red`。 |
| `scripts/generate_urdf.sh` | 调用 `rosrun xacro xacro` 根据输入 xacro/URDF 路径和 `robot_type` 生成 `/tmp/legged_control/<robot_type>.urdf`，用于启动时生成调试或中间 URDF 文件。 |

## legged_hw

真实硬件侧 ROS 包，封装 `ros_control` 所需的 `hardware_interface::RobotHW` 和周期性控制循环。

| 文件 | 作用 |
| --- | --- |
| `CMakeLists.txt` | 构建 `legged_hw` 动态库，依赖 `roscpp`、`legged_common`、`controller_manager`、`urdf`、`Eigen3`。 |
| `package.xml` | 声明 `legged_hw` 包信息和依赖。 |
| `include/legged_hw/LeggedHW.h` | 定义 `legged::LeggedHW`，继承 `hardware_interface::RobotHW`。类中注册 `JointStateInterface`、`ImuSensorInterface`、`HybridJointInterface`、`ContactSensorInterface`，并负责从参数服务器加载 `legged_robot_description` URDF。该类是接入真实电机、IMU、触地传感器的基础抽象。 |
| `src/LeggedHW.cpp` | 实现 `LeggedHW::init()` 和 `loadUrdf()`。当前实现会加载 URDF 并注册各类接口，具体电机和 IMU 驱动接入位置保留在注释中。 |
| `include/legged_hw/LeggedHWLoop.h` | 定义 `legged::LeggedHWLoop`，持有 `controller_manager::ControllerManager` 和 `LeggedHW`，负责以固定频率运行 `read -> update -> write` 控制周期。 |
| `src/LeggedHWLoop.cpp` | 实现控制循环线程。读取私有参数 `loop_frequency`、`cycle_time_error_threshold`、`thread_priority`，可选读取 `cpu_affinity`；循环中调用硬件 `read()`、controller manager `update()`、硬件 `write()`，并监测周期误差。 |

`LeggedHWLoop` 需要的私有参数：

| 参数 | 作用 |
| --- | --- |
| `loop_frequency` | 控制循环频率，单位 Hz。 |
| `cycle_time_error_threshold` | 控制周期超时告警阈值，单位秒。 |
| `thread_priority` | 控制线程的 `SCHED_FIFO` 优先级。 |
| `cpu_affinity` | 可选参数，指定控制线程绑定到的 CPU 核心；小于 0 表示不绑定。 |

## legged_gazebo

Gazebo 仿真侧 ROS 包，提供可被 `gazebo_ros_control` 加载的 `RobotHWSim` 插件，并附带仿真世界、模型和启动文件。

| 文件 | 作用 |
| --- | --- |
| `CMakeLists.txt` | 构建 `legged_hw_sim` 插件库，依赖 `roscpp`、`legged_common`、`gazebo_dev`、`gazebo_ros_control`，并安装插件描述文件、头文件和配置目录。 |
| `package.xml` | 声明 `legged_gazebo` 包信息和依赖。 |
| `legged_hw_sim_plugins.xml` | pluginlib 插件描述文件，将 `legged_gazebo/LeggedHWSim` 注册为 `gazebo_ros_control::RobotHWSim`。 |
| `include/legged_gazebo/LeggedHWSim.h` | 定义 `legged::LeggedHWSim`，继承 `gazebo_ros_control::DefaultRobotHWSim`。类中增加混合关节接口、IMU 接口、接触传感器接口，以及仿真指令延迟缓冲。 |
| `src/LeggedHWSim.cpp` | 实现 Gazebo 硬件仿真插件。`initSim()` 注册混合关节、IMU、接触传感器；`readSim()` 从 Gazebo 读取关节状态、IMU、接触状态；`writeSim()` 将混合命令转换为力矩命令：`kp * (pos_des - pos) + kd * (vel_des - vel) + ff`，再交给默认仿真接口写入。 |
| `config/default.yaml` | Gazebo 仿真参数。当前包含 `gazebo.delay` 指令延迟、`gazebo.imus` IMU frame 和协方差配置、`gazebo.contacts` 触地检测 link/joint 名称列表。 |
| `launch/empty_world.launch` | 通用 Gazebo 空世界启动文件，提供 `paused`、`gui`、`debug`、`physics`、`world_name` 等参数，并启动 `gzserver` / `gzclient`。 |
| `launch/e1_empty_world.launch` | E1 机器人仿真启动入口。加载 `legged_robot_description`，运行 `generate_urdf.sh`，加载 `config/default.yaml`，启动 Gazebo 空世界，并通过 `gazebo_ros spawn_model` 将 E1 机器人生成到仿真中。 |
| `worlds/empty_world.world` | 基础空世界，常用于默认仿真启动。 |
| `worlds/15.world` | 自定义 Gazebo 世界文件，用于特定地形或测试场景。 |
| `worlds/competition.world` | 竞赛/综合测试用 Gazebo 世界文件。 |
| `worlds/star.world` | 星形或特殊场景测试世界文件。 |
| `baylands/model.config` | Gazebo 模型元信息，描述 `baylands` 模型。 |
| `baylands/model.sdf` | `baylands` Gazebo 场景/模型的 SDF 定义。 |
| `baylands/media/scripts/*` | `baylands` 模型使用的材质和 shader 脚本。 |
| `baylands/thumbnails/baylands.jpg` | `baylands` 模型缩略图。 |

## Runtime Flow

### 真实硬件

1. 节点创建并初始化 `LeggedHW`。
2. `LeggedHW` 从参数服务器读取 `legged_robot_description`，加载 URDF。
3. `LeggedHW` 注册关节状态、混合关节、IMU、接触传感器等接口。
4. `LeggedHWLoop` 创建 `controller_manager`，按 `loop_frequency` 周期调用：

```text
hardware.read() -> controller_manager.update() -> hardware.write()
```

### Gazebo 仿真

1. `e1_empty_world.launch` 加载机器人 URDF 和 Gazebo 参数。
2. Gazebo 通过 `legged_hw_sim_plugins.xml` 加载 `legged::LeggedHWSim`。
3. `LeggedHWSim` 在 `initSim()` 中注册混合关节、IMU、接触传感器接口。
4. 每个仿真周期执行：

```text
readSim() -> controller_manager.update() -> writeSim()
```

控制器看到的接口与真实硬件侧保持一致，因此同一套基于 `HybridJointInterface` / `ContactSensorInterface` 的控制器可以在真实硬件和 Gazebo 中复用。
