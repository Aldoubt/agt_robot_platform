# Bunker / YHS / 自研履带接入

Bunker 默认组合及 CAN 驱动保持现状。YHS 和自研履带现在可以通过实测配置接入；现有 YHS 模板仍为 blocked，不能仅修改 `support_status` 绕过缺失数据。

```text
typed research command -> 50 Hz epoch smoother/guard
 -> /agt/base/cmd_vel
 -> agt_base_runtime/tracked_twist_adapter（标准 SI 边界）
 -> 专用 driver Twist endpoint
 -> YHS protocol bridge（仅 YHS） / 自研 SI driver
 -> CAN / 底盘

实际 driver 反馈 -> 配置化 BaseState normalizer -> /agt/base/state
```

通用 adapter 和状态 normalizer 由导航/研究启动入口管理，hardware base launch 不重复启动它们。每个物理底盘只能有一个标准速度边界；driver 必须不发布 odom TF。wheel odom 为实际测量，不代替选定 LIO 的 odom TF。

`devices.yaml.base` 记录 field_verified、50 Hz、publish_odom_tf=false、命令 owner、专用 command_topic、实际 odom_topic/state_topic。对应 Robot Profile 必须包含实测几何、运动上限、skid_steer/differential、Twist 命令以及匹配的 odom_source。

## YHS

已知为履带车；未确认的信息仍为空。需填写 drive_gear、实际 yaw_rate 单位 deg_per_sec 或 rad_per_sec、m_per_sec、协议证据、物理遥控优先级、命令丢失 watchdog。桥不支持把 steering angle 假装成 yaw_rate。倒车仅允许经确认的 signed_speed_same_gear；否则禁止。

启动后的协议桥消费 `/agt/yhs/cmd_vel`，在 `/yhs/ctrl_cmd` 发布实际 CtrlCmd；它自身也校验 BaseState 的 profile、实际 source stamp、steady receipt、急停/遥控/故障。50 Hz steady timer 在仿真时钟暂停时仍做 stale gate；field_verified=false 时不创建物理命令 publisher。硬件 watchdog 和遥控仲裁仍须实车确认。

当前 vendor driver 的 odom frames 固定为 odom/base_link，且不发布 TF；profile 必须匹配，不能通过修改 frame_id 假装完成刚体变换。vendor chassis_info 的 angular 测量语义也要按实际协议确认，再填 StateNormalizer 的 measured_velocity 映射。

vendor driver 已在 checksum 有效的实际 CTRL/IO CAN 帧处分别发布 `/yhs/ctrl_feedback_source`、`/yhs/io_feedback_source`（`std_msgs/msg/Header`，source 字段 `stamp`）。StateNormalizer 的 required_freshness_sources 必须分别订阅两项；CTRL composite header 更新不再能替代 IO 新鲜度，且 composite 时刻必须覆盖最新 IO 证据。IO 含急停、遥控等字段，不能只把普通控制反馈新鲜当作这些字段新鲜。

实际 composite 类型为 `yhs_can_interfaces/msg/ChassisInfoFb`，topic `/yhs/chassis_info_fb`、时间字段 `header.stamp`。若故障映射使用 `bms_flag_fb.*`，可使用 `fault_active.any_of` 明确组合所有已确认叶子字段，并加入 `/yhs/bms_flag_feedback_source`（真实 checksum 有效 BMS_FLAG 帧发布的 Header）作为 required_freshness_sources。每项设置 `require_primary_at_or_after: true`，要求 composite header 覆盖最新 CTRL/IO/BMS_FLAG 证据；缺任一叶子或所需源时刻时不得授权。无需臆测故障字段含义。

## 自研

`custom_tracked` 位于 templates，未注册为支持的整车。复制到 `config/robots/<id>/`，填写实际 Robot Profile 和 driver package/相对 launch_file/arguments。driver 必须消费 geometry_msgs/msg/Twist，linear m/s、angular yaw rate rad/s；参数里明确关闭 TF、控制频率、输入/反馈话题、硬件 watchdog 和遥控优先级。

`argument_bindings` 将 command_topic、odom_topic、publish_odom_tf、control_rate_hz 映射到实际 launch 参数名；loader 检查 arguments 中对应值与整车声明完全一致，避免只在说明中写 no TF/50 Hz 却未传给驱动。

运行前将驱动参数、profile、状态映射、车体几何、外参纳入冻结实验 manifest。代码编译成功不代表协议已验收。
