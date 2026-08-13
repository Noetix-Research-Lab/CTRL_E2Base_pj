# E2 RL 控制器配置

`e2_23dof_ac.yaml` 是唯一机器人策略配置。`control_joint_names` 必须与
`legged_e2_description/urdf/e2_23dof.urdf` 中的硬件关节一致；
`policy_joint_names` 必须与 ONNX `joint_names` metadata 及 motion 文件的
张量顺序一致。两者包含相同的 23 个关节，但顺序可以不同；控制器
会分别按名称取得关节句柄，并在启动时校验策略数量和顺序。

E2 的 23 个驱动自由度由 12 个腿关节、3 个腰关节和 8 个手臂关节组成：

- 腰：`waist_yaw_joint`、`waist_roll_joint`、`waist_pitch_joint`；
- 每条腿：hip pitch/roll/yaw、knee、ankle pitch/roll；
- 每条手臂：shoulder pitch/roll/yaw、elbow。

旧 E1 的 23DOF 策略使用“1 腰 + 每臂 5 关节”的拓扑，不能用于 E2。
部署的 walk/dance ONNX 必须是按上述 E2 拓扑训练和导出的模型。

## 实时参数

`async_inference` 控制异步推理；启用 `require_realtime` 时还必须配置有效的
`inference_cpu_affinity` 和 1..94 范围内的 `inference_thread_priority`。
硬件循环参数位于 `legged_e2_hw/config/e2.yaml`。

构建并 source 工作区后可运行：

```bash
rosrun rl_controllers detect_cpu_topology.py
```

同步/异步时序比较使用 `record_inference_timing.py`，并通过启动参数
`record_timing:=true` 开启控制器与硬件时序发布。
