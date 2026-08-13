# E2 23DOF RL 控制工作区

本工作区面向 Noetix E2 23DOF 机器人。机器人模型位于：

```text
src/legged_rl/legged_robot/e2/legged_e2_description/urdf/e2_23dof.urdf
```

驱动自由度组成是 12 个腿关节、3 个腰关节和 8 个手臂关节。

## 构建

在 ROS Noetic/catkin 环境中：

```bash
./build.sh
```

## 仿真

首先将 E2 专用策略和动作文件放入：

```text
src/legged_rl/rl_controller/rl_controllers/motion_files/e2/yinggewu_23dof.json
src/legged_rl/rl_controller/rl_controllers/policy/e2/policyDown_e2_10w.onnx
src/legged_rl/rl_controller/rl_controllers/policy/e2/policyUp_e2_10w.onnx
src/legged_rl/rl_controller/rl_controllers/motion_files/e2/e2_down.json
src/legged_rl/rl_controller/rl_controllers/motion_files/e2/e2_up.json
```

ONNX `joint_names` metadata 必须和 `config/e2_23dof_ac.yaml` 的
`policy_joint_names` 完全一致；该顺序可以与面向硬件的
`control_joint_names` 不同。仓库原有 E1 23DOF 策略拓扑不同，不能复用。

启动：

```bash
./simulation_23dof.sh
```

也可以通过 `POLICY_FILE`、`POLICY_FILE_DANCE`、`MOTION_FILE`、
`POLICY_FILE_DOWN`、`POLICY_FILE_UP`、`MOTION_FILE_DOWN` 和
`MOTION_FILE_UP` 覆盖默认路径。DOWN/UP 也可分别向 `/down_mode`、
`/up_mode` 发布正数触发；默认手柄组合为 RB+A 和 RB+Y。

## 真机

```bash
./real_23dof.sh
```

注意：URDF、控制器和启动链路已经切换到 E2；EtherCAT PDO、电机方向、零偏和
通道布局仍必须根据 E2 实机电气映射核验后才能安全下发。

## 主要配置

- `rl_controllers/config/e2_23dof_ac.yaml`：E2 关节顺序、策略维度和推理参数；
- `rl_controllers/config/e2_kd.yaml`：通用阻尼参数；
- `legged_gazebo/config/default.yaml`：E2 IMU 和足端接触 link；
- `legged_e2_description/launch/`：模型显示及 Gazebo 启动入口。
