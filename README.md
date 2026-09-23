# AGT Robot Platform

ROS 2 Humble 机器人硬件层，独立于 `agt_navigation_v3`。`agt_robot_bringup` 是 MID360、Bunker 和 robot_state_publisher 的单一硬件启动所有者；`bunker_ros2` 提供 Bunker 驱动与消息；`ugv_sdk` 是底盘 SDK。ROS 包名与迁移前保持一致。

`bunker_ros2` 和 `ugv_sdk` 来自本机原先没有 Git 根的工作副本，原始 Apache 2.0 LICENSE 保留在各自目录。此仓库固定当前实际使用的源码，不要在新工作空间再导入旧的 `drivers/agt_bunker_base` 或另一份 `ugv_sdk`，以免重复包。

`agt_robot_bringup` 还依赖独立的 `agt_robot_description` Profile、Livox 驱动、C1 和 INS 包。导航与建图均通过 ROS 包名 include 此硬件入口，不引用本仓库的绝对源码路径。
