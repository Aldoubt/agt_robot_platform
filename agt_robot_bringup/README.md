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

## Whole-robot configs (`config/robots/`)

One workspace, several physical robots. A robot is selected **at launch time**
by a whole-robot config; there is no hot switching — stop the running stack
before starting another robot.

```text
config/robots/
├── bunker_inspection/   support_status: supported   (reference robot / template)
│   ├── robot.yaml       which base adapter, sensors, payloads; robot_profile id
│   └── devices.yaml     deployment values: CAN port, device paths, camera mode
└── yhs_harvesting/      support_status: blocked     (reserved; refuses to start)
    ├── robot.yaml
    └── devices.yaml
```

### Who owns which value (single source each)

| Value | Owner |
| --- | --- |
| Model/URDF, frames, extrinsics, calibration, footprint, motion limits | `agt_robot_description/config/robot_profiles/<robot_profile>.yaml` (+ files it references) |
| Which devices are installed / enabled by default, base adapter, task handlers | `robot.yaml` |
| CAN port, `/dev` paths, camera resolution/FPS, gimbal serial port, MID360 driver mode | `devices.yaml` |
| Per-run exceptions (`--rtk`, navigation mode disabling the camera, mapping using `mapping_custom`) | explicit launch arguments, validated |

`robot.yaml` must not contain geometry (a unit test rejects `footprint`,
`extrinsics`, `frames`, `calibration`, velocity limits …).

### robot.yaml fields (schema_version 1)

| Field | Meaning |
| --- | --- |
| `robot_id` | must equal the directory name |
| `support_status` | `supported` or `blocked`; `blocked` requires `blockers:` (list of reasons) |
| `robot_profile` | Robot Profile id in agt_robot_description; its `base.driver` must match the adapter |
| `base.adapter` | `bunker` (available, `bunker_base`) or `yhs` (**missing ROS 2 driver → always blocked**) |
| `sensors.navigation_lidar` | `{model: mid360, installed, enabled}`; only `mid360` has a hardware integration |
| `sensors.navigation_imu.source` | documentation of the IMU source (`mid360_internal`) |
| `sensors.rtk` | `{installed, enabled, role: metadata_only}`; RTK never moves `map→odom` |
| `payloads.camera_gimbal` | `{installed, enabled}`; requires profile `capabilities.acquire_view` |
| `payloads.arm` | reserved, see below; requires profile `capabilities.manipulation` |
| `mission.task_handlers` | Mission handlers this robot can serve (`camera.acquire_view`, `arm.pick_kiwi`) |
| `devices_file` | file name in the same directory (default `devices.yaml`) |

`enabled: true` requires `installed: true`.

### Resolution rules (`tools/robot_config.py`)

1. `robot_config` given → use it. Only `robot:=<profile>` given (legacy) → the
   single *supported* config using that profile (`bunker_v1 → bunker_inspection`).
   Nothing given → legacy default `bunker_v1`.
2. Blocked config, unknown adapter, missing driver or missing profile →
   `RobotConfigBlocked`, exit code 3 from the CLI. **No fallback to Bunker.**
3. `robot` and `robot_config` both given and different profiles → error (exit 2).
4. Managed launch arguments (`enable_mid360`, `mid360_driver_mode`,
   `enable_bunker_can`, `bunker_can_port`, `enable_rtk`, `enable_camera_gimbal`,
   `camera_*`, `gimbal_port_name`) default to `""` = take the config value. A
   non-empty value is an override: enabling something not installed, enabling
   the Bunker CAN on a non-Bunker robot, or setting a device value for a device
   the robot does not have is an error. Applied overrides are logged.

```bash
# inspect without starting anything
ros2 run agt_robot_bringup robot_config.py list
ros2 run agt_robot_bringup robot_config.py resolve --robot-config bunker_inspection
ros2 run agt_robot_bringup robot_config.py resolve --robot-config yhs_harvesting   # exit 3, BLOCKED
```

Compatibility note: calling `robot_hardware.launch.py` directly previously
defaulted to `enable_rtk:=true`, contradicting the profile (`gnss.enabled:false`)
and `run_field_stack.sh` (RTK off unless `--rtk`). The single default now comes
from `bunker_inspection/robot.yaml` (`rtk.enabled: false`); pass
`enable_rtk:=true` (or `--rtk`) to record RTK. All other Bunker defaults are
unchanged (verified by `test_bunker_defaults_equal_previous_hardware_launch`).

### Arm payload (reserved, not implemented)

`payloads.arm` is a reserved integration point. The arm is not started by this
launch file; it is `agt_arm_capability` (`/arm/pick_kiwi`) started via
`agt_mission_bringup enable_arm:=true`. Before any robot with an arm may be set
to `supported`, the following contract must be implemented and field-verified:

- A drive-permission state published by the arm side, true only when the picking
  script is back at the verified home joints, waiting, and all joints are
  within the verified tolerance (values to be sourced from the original picking
  code, not guessed). Action success, process exit and cancel acknowledgement do
  **not** imply drive permission.
- While the arm is working or the state is unknown/stale, the chassis command
  guard (`agt_base_control/cmd_vel_guard`, the single `/cmd_vel → /mux/cmd_vel`
  arbiter) forces zero velocity and the navigation capability rejects new goals;
  Mission does not advance (including `on_failure: skip`) without permission.

#### Audited material for drive permission (2026-09-25, static review only)

Source: `ros2_ws/机械臂/jaka_camera_ws/src/demo/scripts/kiwipickingandmove.py`
(not under version control).

- Return pose `injstep_neg` is assigned twice at lines 1558–1559 inside the
  `__main__` block; the second value is effective:
  `[-1.670573, -0.054178, 1.425672, 0.337107, -1.561190, 1.478611]` rad.
- Lines ~2052–2095: after a cycle the script polls `rc.get_joint_position()`
  every 0.2 s, up to 100 times, and accepts when
  `sum(|q - injstep_neg|) < 0.03` rad; only then does it block in
  `wait_for_q()`. Otherwise it prints `未满足条件，终止监测` (a failure marker).
- `开始监测关节位置...` — the `picker_ready_marker` used by
  `agt_arm_capability` — is printed **before** the check, so the capability's
  success contract relies on the subsequent `wait_for_q()` block, not the marker.
- The 0.03 rad threshold is a *sum over six joints* chosen for the script's own
  sequencing; it is not an approved per-joint drive tolerance, and the state is
  not published anywhere. It is usable as audit evidence, not as the
  drive-permission contract. Still open: a published, fresh joint state; an
  approved per-joint tolerance; default-deny guard/Capability/Mission interlock.

## YHS TK-mid base adapter (R3)

* Driver: official `YUHESEN-Robot/TK-mid-ros2` pinned at
  `6d002bddec319a9175a7ada9703f9e7bcb2a7d71` (2026-08-10), cloned to
  `src/drivers/yhs_tk_mid_ros2` (packages `yhs_can_interfaces`, `yhs_can_control`).
  **No LICENSE file upstream — licence unknown; clarify with YUHESEN before redistribution.**
* Adapter: `agt_yhs_adapter/yhs_cmd_vel_bridge` — `/mux/cmd_vel` (Twist) →
  `/yhs/ctrl_cmd` (CtrlCmd) at 50 Hz, zero command when input is older than
  0.25 s, reverse refused (`allow_reverse:=false`) until the reverse semantics
  are confirmed, rad/s → deg/s. High levels never see CtrlCmd.
* Driver topics are pinned: `ctrl_cmd, io_cmd, free_ctrl_cmd, chassis_info_fb`
  → `/yhs/...`; its integrated odometry → `/wheel/odom` (diagnostics only).
  The driver broadcasts **no TF**.
* Units from source: linear m/s (×1000 on the wire), angular deg/s (×100;
  feedback documented as °/s in manual V1.1.1).
* **Gear is not guessed**: README says `4 = D`, PDF manual V1.1.1 says
  `3 = kinematic control`. `devices.yaml base.drive_gear: null` keeps
  `yhs_harvesting` BLOCKED; the launch also refuses an empty gear.
* Launch args (managed): `enable_yhs_can`, `yhs_can_port`, `yhs_drive_gear`.
  `enable_yhs_can:=true` on a Bunker robot (or `enable_bunker_can:=true` on YHS) is an error.

## Payloads and drive interlock (R3)

* `bunker_inspection`: camera gimbal (hardware launch) + RTK (installed,
  metadata only, default off; `--rtk` or `enable_rtk:=true`).
* `yhs_harvesting`: arm (`agt_arm_capability`) + `picking_camera`
  (Orbbec TOF, opened by the picking script via pyorbbecsdk — no ROS driver).
* An installed+enabled arm makes the resolver print `payload_interlock=true`;
  `navigation.launch.py`/`mission.launch.py` then require a fresh
  `/agt/payload/drive_permission` in `agt_cmd_vel_guard`, the navigation
  capability and the mission. Contract: `agt_arm/docs/ARM_TASK_INTEGRATION.md`.
