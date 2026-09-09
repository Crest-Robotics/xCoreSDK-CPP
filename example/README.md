# xCore SDK Examples

> **Note:** This document was written by Claude Code to accompany the translated example sources.

Each `.cpp` file below builds to an executable target of the same name (see `CMakeLists.txt` / `rt/CMakeLists.txt`). Build with `cmake --build build --target <name>`, or `--target all` / `--target install` to build everything. Every example connects to a real (or simulated) xCore controller over Ethernet — see the top-level [README](../README.md#hardware-setup) for hardware setup.

Waypoints and parameters in these examples are reference values for API usage and may not be reachable on every robot model — check the applicable model noted for each example, and confirm parameters against the xCore Control System Manual before running on a different model.

## Non-real-time examples (`example/`)

| Example | Description | Notes |
|---|---|---|
| `sdk_example` | Menu-driven tour of the SDK: reachability, forward/inverse kinematics, jog, drag teaching, singularity-avoidance jog, tool/workpiece/base frame calibration, NTP config, collision detection, e-stop reset, parallel-base mode, teach-pendant mode, reboot/shutdown. | Good starting point — covers most non-motion interfaces in one place. |
| `move_example` | Non-real-time motion commands: MoveAbsJ, MoveJ, MoveL, MoveC, MoveCF, MoveSP, speed/blending-zone settings, rail (external-axis) motion, singularity avoidance, keyboard-driven speed scaling. | Largest example file; waypoints are illustrative only. |
| `path_record` | Collaborative-robot drag teaching, plus path recording and playback. | Collaborative robots only. |
| `rl_project` | Load, run, and manage an RL project (a Rokae robot-program project file) on the controller. | |
| `read_robot_state` | Reads robot state data (pose, status, etc.) on a background thread while motion runs on the main thread. | Demonstrates the state-data queue/polling model. |
| `force_control_commands` | Cartesian and joint force/torque control commands. | Applicable model: xMateCR series (parameters are model-dependent). |
| `communication_example` | Reads the end-effector keypad state (digital I/O). | |
| `modbusRTU_endtool_control` | Transparent-transmission protocol for reading/writing an end-effector tool over Modbus RTU (e.g. a gripper). | |
| `controller_upgrade` | Controller firmware upgrade and backup export. | Links the separate `Rokae::Upgrade` library. |
| `xmatemodel_er3_er7p` | Kinematics/dynamics computation via the xMateModel library, using xMate3 and xMateEr7Pro as examples. | Only built when `XCORE_USE_XMATE_MODEL=ON`; Linux x86_64 and Windows 64-bit only. |

## Real-time examples (`example/rt/`)

Real-time mode supports control at up to 1 kHz and requires a direct, low-latency Ethernet connection to the controller.

| Example | Description | Notes |
|---|---|---|
| `move_commands` | Real-time-mode S-curve planned MoveJ, MoveL, and MoveC. | |
| `joint_s_line` | Real-time joint-space S-curve planning. | Applicable model: xMateER7 Pro. |
| `cartesian_s_line` | Real-time Cartesian-space S-curve planning. | Applicable model: xMateER7 Pro. |
| `joint_position_control` | Real-time joint-angle position control. | Applicable model: xMateER3. |
| `joint_impedance_control` | Real-time joint-space impedance control. | Applicable model: xMateER7 Pro. |
| `cartesian_impedance_control` | Real-time Cartesian-space impedance control. | |
| `rt_industrial` | Real-time position control for six-axis industrial robot models. | |
| `servoj_demo` | Real-time ServoJ (per-cycle joint servo command) demo. | |
| `torque_control` | Real-time direct joint torque control. | Only built when `XCORE_USE_XMATE_MODEL=ON`. |
| `follow_joint_position` | Real-time joint-space waypoint following. | Only built when `XCORE_USE_XMATE_MODEL=ON`. |
| `follow_cart_position` | Real-time Cartesian-space waypoint following (sine-wave target). | Only built when `XCORE_USE_XMATE_MODEL=ON`. |

## Shared helper headers

| File | Purpose |
|---|---|
| `print_helper.hpp` | Stream-operator formatting for SDK types (robot state, Cartesian position, load, tool/workpiece settings, etc.) used by console output across the examples. |
| `function_helper.hpp` | Small shared utility (`waitRobot`) used across the non-real-time examples. |
| `rt/rt_funtion_helper.hpp` | Real-time-mode equivalent helper (state queue draining / non-real-time fallback). |
