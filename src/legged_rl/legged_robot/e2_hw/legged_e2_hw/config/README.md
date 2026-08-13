# legged_e2_hw config 参数说明

本目录包含 `legged_e2_hw` 实机硬件接口相关配置。启动入口 `launch/legged_e2_hw.launch` 会加载 `ethercat_config.yaml` 和 `e2.yaml`；`config.yaml` 由 `E2HW::init()` 通过 `RemoteUserParameter` 直接读取，默认路径为本目录下的 `config.yaml`，也可以通过 ROS 参数 `/user_param_path` 覆盖。

## 文件作用

- `e2.yaml`：ROS 硬件控制循环参数，位于节点私有命名空间 `legged_e2_hw` 下。
- `ethercat_config.yaml`：EtherCAT 网卡、电机类型、限幅和方向配置。
- `config.yaml`：测试状态机使用的用户参数，例如测试目标角度、测试力矩和插值时间。

## e2.yaml

根节点为 `legged_e2_hw`，这些参数会被 `LeggedHWLoop` 和 `legged_e2_hw` 节点按私有参数读取。

| 参数 | 含义 |
| --- | --- |
| `loop_frequency` | 硬件控制循环频率，单位 Hz。当前值 `500` 表示 `read -> controller update -> write` 以 500 Hz 运行。 |
| `cycle_time_error_threshold` | 控制循环周期误差报警阈值，单位 s。当实际周期相对期望周期的超时超过该值时打印 ROS warning。 |
| `thread_priority` | 实时控制线程的 `SCHED_FIFO` 优先级。需要系统允许实时优先级，例如 `rtprio` 或对应 capability。 |
| `cpu_affinity` | 实时控制线程绑定的 CPU 核心编号。`-1` 表示不绑定；大于等于 0 时会尝试把 RT 线程固定到指定核心，同时让非 RT 线程避开该核心。 |
| `contact_threshold` | 足端接触判断阈值预留参数。当前 `E2HW` 中有成员变量，但接触传感器初始化逻辑未启用。 |
| `enable_ankle_log` | 是否启用踝关节解算调试 CSV。默认 `false`；实时线程只组装固定大小记录并写入 SPSC 队列，CSV 格式化和文件写入由独立非实时线程完成。队列满时丢弃调试行并累计报警，不阻塞控制循环。 |
| `ankle_log_path` | 踝关节解算调试 CSV 路径，支持 `{timestamp}` 占位符；兼容全局参数 `/ankle_log_path` 覆盖。启用 `enable_ankle_log` 时启动阶段截断文件并写入表头，后台线程每秒刷新并在退出时排空队列。 |
| `enable_motor_error_log` | 是否记录电机故障。默认 `true`；实时线程通过无锁 SPSC 队列提交事件，只记录首次故障、故障码变化、每秒持续故障汇总和恢复。文件写入及终端输出由后台线程完成。 |
| `enable_check_and_print` | 是否启用 `CheckAndPrint()` 每秒一次的终端温度汇总打印。默认 `false`；该功能直接在硬件读取线程中锁互斥量并输出终端，`require_realtime=true` 时禁止开启。 |
| `enable_motor_non_error_log` | 是否记录无故障电机的温度/状态。默认 `false`；开启后每个关节最多每秒记录一次，但实时路径仍需逐帧检查时间，因此 `require_realtime=true` 时禁止开启。 |
| `enable_thermal_protection_limit_release` | 是否解除 INKEX 电机/MOS 温度反馈的热保护转换限制。`true` 时直接使用原始温度值；`false` 时按保护转换公式 `(raw_temp - 50) / 2` 计算。 |
| `motor_error_log_path` | 电机错误/温度日志文件路径。相对路径基于 launch 启动时的工作目录；父目录不存在时会自动创建。后台线程批量写入并每秒刷新一次。设为空字符串、`none` 或 `off` 时禁用异步文件日志。 |

注意：`cpu_affinity` 绑定隔离核时，通常需要系统启动参数和权限配合，例如 `isolcpus`、`nohz_full`、`rcu_nocbs`、`CAP_SYS_NICE`。

`can_error_detection` 根据 Medulla PDO 中 9 路累计 CAN 错误计数判断通道状态。任意启用通道进入 Fault 后，实时线程会立即锁存硬件故障；同一控制周期的 `write()` 将覆盖控制器输出并进入安全命令。通道恢复只用于更新诊断状态，不会自动恢复控制，必须检查硬件后重启硬件节点。`enabled_mask` 的 bit 0..8 对应 `node1/can1` 到 `node3/can3`。故障终端和 CSV 日志会根据当前固定 PDO 槽位映射输出该通道的候选关节组；它用于缩小排查范围，不表示已经确认其中某一台电机故障。

## ethercat_config.yaml

该文件通过 `drake::yaml::LoadYamlFile<EthercatOptionsFromYaml>()` 读取。字段名必须与 `EthercatParameter.h` 中的 `EthercatOptionsFromYaml` 保持一致。

### 通信与机器人信息

| 参数 | 含义 |
| --- | --- |
| `net_card` | EtherCAT 主站使用的网卡名，例如 `enp2s0`。`E2HW::init()` 会传给 `rt_ethercat_init()`。 |
| `ctr_freq` | EtherCAT 控制频率配置，单位 Hz。当前代码会解析该字段，但实际控制循环频率由 `e2.yaml` 的 `loop_frequency` 决定。 |
| `robot_type` | 机器人类型编号。当前文件中为 `1`；在当前硬件初始化路径中主要作为配置保留字段。 |
| `node_motor_num` | 各电机组包含的电机数量。当前值 `[4, 4, 6, 6, 3]` 对应右臂、左臂、右腿、左腿、腰部；EtherCAT 从站创建仍由 `rt_ethercat_config()` 固定完成。 |

### 电机类型限幅表

以下数组是“按电机类型索引”的限幅表。`*_motor_type` 中的值不是 `MotorType::DMBOT/INKEXBOT` 枚举，而是这些限幅数组的下标。

| 参数 | 含义 |
| --- | --- |
| `motor_max_torque_rec` / `motor_min_torque_rec` | 反馈解包时使用的各电机类型最大/最小力矩映射范围。 |
| `motor_max_torque_send` / `motor_min_torque_send` | 命令打包发送时使用的各电机类型最大/最小力矩限幅和映射范围。 |
| `motor_max_current` / `motor_min_current` | 各电机类型允许的最大/最小电流。会写入 `ControlFSMData::joint_max_current_` / `joint_min_current_`。 |
| `motor_max_velocity` / `motor_min_velocity` | 各电机类型允许的最大/最小速度。发送命令和反馈解包时会使用该范围做限幅和数值映射。 |

当前类型索引与限幅值如下：

| 类型索引 | 接收最大力矩 | 发送最大力矩 | 最大电流 | 最大速度 | 当前主要用途 |
| --- | --- | --- | --- | --- | --- |
| `0` | `28` | `28` | `10` | `20` | 双臂电机 |
| `1` | `126` | `90` | `60` | `18` | 髋部部分电机、腰部电机 |
| `2` | `175` | `150` | `70` | `18` | 髋/膝部分电机 |
| `3` | `84` | `70` | `30` | `18` | 踝关节电机 |

关节对应的电机型号参考如下：

| 关节 | 型号 | 数量 | 供应商 |
| --- | --- | --- | --- |
| `arm` | `DM-JA4340-2EC` | 10 | 达秒 |
| `hip_yaw` / `hip_roll` | `EC-A8112-P1-18` | 4 | 因克斯 |
| `hip_pitch` / `knee` | `EC-A10020-P1-12` | 4 | 因克斯 |
| `ankle` | `EC-A4315-P2-36` | 4 | 因克斯 |
| `waist` | `EC-A4315-P2-36` | 1 | 因克斯 |
| 总计 |  | 23 |  |

最小值数组一般应与最大值对称，例如 `motor_min_torque_rec` 和 `motor_min_torque_send` 为负值。

### 各部位电机类型

这些数组决定底层 EtherCAT 通道分别使用哪一组限幅。当前实机通道映射仍需根据 E2 电气拓扑核验，不可直接用于下发。代码中的通道顺序为：

| 关节编号范围 | 部位 |
| --- | --- |
| `0-4` | 右臂 |
| `5-9` | 左臂 |
| `10-15` | 右腿 |
| `16-21` | 左腿 |
| `22-23` | 腰部 |

| 参数 | 长度 | 含义 |
| --- | --- | --- |
| `right_arm_motor_type` | 5 | 右臂 5 个电机对应的类型索引。 |
| `left_arm_motor_type` | 5 | 左臂 5 个电机对应的类型索引。 |
| `right_leg_motor_type` | 6 | 右腿 6 个电机对应的类型索引。 |
| `left_leg_motor_type` | 6 | 左腿 6 个电机对应的类型索引。 |
| `waist_motor_type` | 2 | 腰部底层电机通道对应的类型索引，当前映射到通道 `22-23`。 |

修改这些字段时，类型索引必须落在限幅表数组范围内。例如当前限幅表长度为 4，则合法索引为 `0-3`。

### 各部位电机方向

方向数组会写入 `ControlFSMData::joint_dir_`。这是电机坐标到 URDF 关节坐标的最终方向系数；在反馈解包和命令打包时，位置、速度、力矩都会乘以该方向系数。

| 参数 | 长度 | 含义 |
| --- | --- | --- |
| `right_arm_motor_dir` | 5 | 右臂电机方向，通常为 `1` 或 `-1`。 |
| `left_arm_motor_dir` | 5 | 左臂电机方向，通常为 `1` 或 `-1`。 |
| `right_leg_motor_dir` | 6 | 右腿电机方向，通常为 `1` 或 `-1`。 |
| `left_leg_motor_dir` | 6 | 左腿电机方向，通常为 `1` 或 `-1`。 |
| `waist_motor_dir` | 2 | 腰部电机方向，映射到关节 `22-23`。 |

如果实机出现某关节反馈方向与期望相反，或正向命令导致反向运动，应优先检查对应 `*_motor_dir`。

### 关节位置偏置

| 参数 | 长度 | 含义 |
| --- | --- | --- |
| `bias_motor` | 23 | 23 个关节的位置偏置，顺序为右臂、左臂、左腿、右腿、腰部。反馈读取时会加到关节位置上，命令下发前会从期望位置中减掉；只作用于 position，不作用于 velocity / torque。 |

## config.yaml

该文件的集合名为 `e2_user_parameters`，必须与 `RemoteUserParameter` 构造函数中的名字一致。

| 参数 | 含义 |
| --- | --- |
| `__collection-name__` | 参数集合名称，当前必须为 `e2_user_parameters`。 |
| `MainThreadPeriod` | 测试 FSM 的主线程周期，单位 s。插值器用它计算轨迹采样。 |
| `Kp` | 测试状态机位置控制阶段使用的统一比例增益。 |
| `Kd` | 测试状态机位置控制阶段使用的统一微分增益。 |
| `NonZeroAngle` | 测试状态机 `ROATE_TO_NONZERO_POS` 阶段的目标关节角度。按当前代码原样传入命令，单位应与控制链路使用的关节位置单位保持一致。 |
| `TorCmdValue` | 测试状态机 `EXEC_TORQUE_CMD` 阶段下发到各测试关节的目标力矩。 |
| `InerpolateTime` | 位置插值持续时间，单位 s。字段名在代码中拼写为 `InerpolateTime`，配置中也必须保持该拼写。 |
| `TorExecTime` | 力矩命令执行持续时间，单位 s。超过该时间后测试状态机回到等待状态。 |

## 修改配置注意事项

- 改 `net_card` 前先确认系统网卡名，例如使用 `ip link` 查看。
- 改 `*_motor_type` 后，要同步确认对应类型索引在 `motor_*` 限幅表中存在。
- 改 `*_motor_dir` 后，建议先低增益、低力矩验证单关节方向。
- 改 `loop_frequency` 时，要同时评估 EtherCAT 通信周期、控制器周期和实时线程负载。
- 这些配置会直接影响实机动作和限幅，实机测试前应确认机器人处于可急停状态。
