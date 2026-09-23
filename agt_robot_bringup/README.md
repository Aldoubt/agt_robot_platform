# agt_robot_bringup

`robot_hardware.launch.py` owns the physical driver lifecycle and the single
`robot_state_publisher`. `robot_bringup.launch.py` is a compatibility alias.
The selected `robot:=bunker_v1` profile is schema checked before hardware
starts. Its calibration is loaded from `agt_robot_description` without changing
any field accepted transform.

The field navigation entry includes this owner through
`agt_system_bringup/hardware.launch.py`. Live mapping includes the same owner
with only the MID360 enabled, using `mid360_driver_mode:=mapping_custom` and
the mapping session's validated Livox JSON. Offline bag mapping uses no driver.

Bunker publishes `/wheel/odom` with `publish_odom_tf=false`. The selected LIO
adapter owns navigation odometry TF, and `agt_localization_manager` owns
`map → odom`. The robot owner launches one description, one MID360 and one
Bunker driver per session. Do not start navigation and live mapping hardware
sessions in the same ROS domain concurrently.

Inspect available launch arguments without starting hardware:

```bash
ros2 launch agt_robot_bringup robot_hardware.launch.py --show-args
```
