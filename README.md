# UAV OMMPC ROS 2 / PX4 bridge

Minimal ROS 2 Jazzy **C++17 / rclcpp** communication packages for PX4 v1.17. This repository does **not** implement the user's OMMPC algorithm. An external controller supplies total thrust acceleration and body angular rates; the bridge relays telemetry and translates commands to PX4 offboard inputs. The Python bridge has been removed; `ros2 run uav_ommpc_ros px4_bridge` and all public topics, messages, QoS and startup parameters remain unchanged. Only the SITL test utilities use Python.

## Build and environment

Dependencies on this Jetson: ROS 2 Jazzy (`/opt/ros/jazzy`), `px4_msgs` release/1.17 in `~/ros2_px4_ws`, PX4 v1.17.0 in `~/PX4-Autopilot`, Gazebo Harmonic and Micro XRCE-DDS Agent v2.4.3. Build the PX4 message workspace before this overlay. Message definitions must match PX4.

`uav_ommpc_ros` uses `ament_cmake` and links against `rclcpp`, `px4_msgs` and `uav_ommpc_interfaces`. When migrating an already-built Python workspace, remove its obsolete generated `build/uav_ommpc_ros` and `install/uav_ommpc_ros` directories before rebuilding; source the ROS/px4_msgs underlays, not the old project overlay.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_px4_ws
colcon build --symlink-install --packages-select px4_msgs
source ~/ros2_px4_ws/install/setup.bash
cd ~/uav_control_ws
colcon build --symlink-install --packages-select uav_ommpc_interfaces uav_ommpc_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/uav_control_ws/install/setup.bash
```

In every ROS terminal, source the underlays **before** this workspace:

```bash
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/setup.bash
```

`~/ros2-px4-env.bash` supplies the existing ROS/PX4-message environment; it does not replace the new overlay. Use `~/px4-env.bash` in the separate PX4 simulator terminal.

```bash
ros2 run uav_ommpc_ros px4_bridge --ros-args \
  -p hover_thrust:=0.728744 -p max_body_rates:="[3.0, 3.0, 1.5]" -p max_throttle:=0.9 \
  -p rate:=50.0 -p command_timeout:=0.5 -p target_system:=1 -p target_component:=1
```

Parameters are read at startup; restart to change them. `rate` is outgoing setpoint/heartbeat frequency in Hz. `command_timeout` is maximum accepted-input age in seconds, measured with a monotonic clock. `hover_thrust` and `max_throttle` must be in (0, 1], `max_body_rates` must contain three positive roll/pitch/yaw limits in rad/s, rate and timeout must be positive, and target IDs in [1, 255].

## Public interface

All topics use volatile durability and depth 1; no commands are retained for future subscribers.

| Topic | Message | Reliability | Meaning |
| --- | --- | --- | --- |
| `/uav/control_input` | `uav_ommpc_interfaces/msg/ThrustRates` | reliable | External controller input; stream at least 50 Hz |
| `/uav/state` | `px4_msgs/msg/VehicleOdometry` | best effort | Unmodified PX4 odometry |
| `/uav/status` | `px4_msgs/msg/VehicleStatus` | best effort | Unmodified PX4 vehicle status |
| `/uav/command_ack` | `px4_msgs/msg/VehicleCommandAck` | best effort | Unmodified PX4 command acknowledgements |
| `/uav/vehicle_command` | `px4_msgs/msg/VehicleCommand` | reliable | Explicit mode/arm/land/etc. request |

`ThrustRates` fields:

- `float64 thrust_acceleration`: finite, nonnegative **total** thrust acceleration in m/s², along body **−Z**. Includes gravity compensation: nominal level hover is `9.80665`, not zero.
- `float64[3] body_rates`: finite roll, pitch, yaw angular rates in rad/s, in body **FRD** (forward, right, down). These are rates, not Euler angles.

Telemetry remains in raw PX4 frames, without an ENU/FLU transform: position/velocity are NED (north, east, down) when the frame fields report NED; `q` is **w, x, y, z**; angular velocity is body FRD. Consumers must check `pose_frame`, `velocity_frame`, estimator validity and freshness. PX4 timestamps are forwarded unchanged.

The bridge subscribes with best-effort QoS to `/fmu/out/vehicle_odometry` (message version 0), `/fmu/out/vehicle_status_v1`, and `/fmu/out/vehicle_command_ack`. It publishes reliable volatile `/fmu/in/offboard_control_mode`, `/fmu/in/vehicle_rates_setpoint`, and `/fmu/in/vehicle_command`. Only `body_rate` is enabled in `OffboardControlMode`; all other flags remain false.

```text
roll, pitch, yaw = clip(body_rates, -max_body_rates, +max_body_rates)   # per axis
throttle = min(hover_thrust * thrust_acceleration / 9.80665, max_throttle)
thrust_body = [0, 0, -throttle]
```

Defaults are `max_body_rates=[3.0, 3.0, 1.5]` rad/s and `max_throttle=0.9` (keeping attitude-control headroom). Limiting is silent to PX4 but logs a throttled warning once per second; the controller is not otherwise notified.

For example, in a sourced ROS terminal (this publishes input only; it does not arm):

```bash
ros2 topic pub -r 50 /uav/control_input uav_ommpc_interfaces/msg/ThrustRates \
  '{thrust_acceleration: 9.80665, body_rates: [0.0, 0.0, 0.0]}'
```

This is an **approximate normalized-throttle mapping**, not a physical motor/thrust model or guaranteed acceleration tracking. The node's generic default is `hover_thrust=0.60`; explicitly use **0.728744 for this installed Gazebo x500**. Its model has mass 2.06430769 kg, four motor constants 8.54858e-6, and linear ESC speed range 150–1000 rad/s (`THR_MDL_FAC=0`): `omega_hover=sqrt(mass*g/(4*k))=769.4326`, so normalized hover is `(omega_hover-150)/850=0.728744`. PX4's `MPC_THR_HOVER=0.60` is an initial controller estimate, not a sufficient calibration for this external feedforward path. Other vehicles, battery conditions and payloads require calibration. The mapping is exact only at hover; with this motor model thrust grows with the square of rotor speed, so non-hover accelerations deviate. There is no slew-rate or tilt limiting.

Forwarded vehicle commands preserve their command and parameters but replace timestamp, `source_system=250`, `source_component=190`, configured target IDs, and `from_external=True`. Generated command/setpoint timestamps use ROS clock microseconds (wall clock by default; do not enable simulated ROS time without arranging its clock source).

## Control ownership and safety

The bridge **never automatically arms or changes flight mode**. External software must explicitly request and confirm these actions, satisfy PX4 offboard entry requirements, and monitor status/acknowledgements. Stream setpoints before requesting offboard mode. PX4 must be connected through a running Micro XRCE-DDS Agent and expose the expected DDS topics.

Nonfinite input values and negative thrust are rejected without refreshing the timeout or replacing the previous command. Otherwise the last accepted, limited input is repeated at the configured rate.

| Condition | `offboard_control_mode` heartbeat | `vehicle_rates_setpoint` |
| --- | --- | --- |
| No accepted input yet, or input older than `command_timeout` | stopped | stopped |
| Fresh input, PX4 `nav_state` is not Offboard | published | **not published** |
| Fresh input, PX4 `nav_state` is Offboard | published | published |

The heartbeat must flow before PX4 can enter Offboard, so it is not gated by mode. Rate setpoints are gated because PX4's `mc_rate_control` uses the newest `vehicle_rates_setpoint` from any publisher; outside Offboard they would compete with PX4's own attitude controller. The gate follows the latest received `VehicleStatus` (PX4 publishes it immediately on a mode change).

The bridge does not send zero thrust, land, disarm, retry, or choose recovery modes. After the input times out, PX4 holds the last setpoint until its offboard-loss timeout (`COM_OF_LOSS_T`, default 1.0 s) and then applies its configured offboard-loss failsafe. Telemetry and explicit vehicle commands continue. This is not a complete safety system. Ctrl-C does not itself land or disarm a vehicle.

## Unit tests

Input validation, limits and timeout logic live in `uav_ommpc_ros/src/command_state.hpp` and are tested with injected time (no ROS or PX4 needed):

```bash
cd ~/uav_control_ws
colcon test --packages-select uav_ommpc_ros
colcon test-result --verbose --test-result-base build/uav_ommpc_ros
```

## SITL flight test

Use only the simulator for these utilities. Start these prerequisites in separate terminals; the flight test does not launch them:

```bash
# Terminal 1: DDS transport.
MicroXRCEAgent udp4 -p 8888
```

```bash
# Terminal 2: PX4 and headless Gazebo.
source ~/px4-env.bash
cd ~/PX4-Autopilot
HEADLESS=1 make -j4 px4_sitl gz_x500
```

```bash
# Terminal 3: communication node.
source ~/ros2-px4-env.bash
source ~/uav_control_ws/install/local_setup.bash
ros2 run uav_ommpc_ros px4_bridge --ros-args -p hover_thrust:=0.728744
```

A separate test-only GCS heartbeat is needed for PX4's GCS-present safety check:

```bash
cd ~/uav_control_ws/src/uav-ommpc-controller
~/.venvs/px4-v1.17/bin/python tools/sitl_gcs_heartbeat.py
```

This helper sends an actual GCS heartbeat on UDP port 18570, not vehicle commands or parameter changes. In another terminal, source the ROS underlays and new workspace overlay. The flight utility requires NumPy in that ROS Python environment (`sudo apt install python3-numpy` if missing):

```bash
cd ~/uav_control_ws/src/uav-ommpc-controller
python3 tools/sitl_flight_test.py
# Optional: --output-dir /absolute/path/to/results
```

The default result directory is `~/uav_control_ws/flight-test-results`. Consult `--help` for options and generated output for measured results. The utility is a **test controller**, not the user's OMMPC algorithm. It uses telemetry and explicit vehicle commands to exercise the bridge, preserves measured initial yaw, and later requests a yaw offset of +0.25 rad. Never run either test utility against real hardware; no real-hardware verification has been performed.

### Observed SITL verification

On the installed Jetson stack, both packages built successfully in Release mode; the installed bridge is a native ARM64 ELF executable, and the 6 `CommandState` unit tests pass.

A live, **disarmed** check against PX4 SITL (`cpp-run-02/communication.json`) observed:

| PX4 state / input | Heartbeat messages | Rate setpoints |
| --- | ---: | ---: |
| Fresh input, not Offboard (1.5 s) | 76 | 0 |
| Fresh input, Offboard (1.0 s) | 50 | 50 |
| Fresh input, after switching to Hold (1.0 s) | 50 | 0 |
| Input stopped, after timeout (0.5 s) | 0 | 0 |

In Offboard, input `[0.1, -0.2, 0.3]` rad/s at 1 g produced `[0.1, -0.2, 0.3]` and thrust `-0.728744`; input `[4, -5, 2]` rad/s at 30 m/s² was limited to `[3, -3, 1.5]` and thrust `-0.9`.

The gated C++ bridge then passed the unchanged flight acceptance with `hover_thrust=0.728744` and default limits (`cpp-run-02/result.json`, `trace.csv`; earlier evidence remains in `cpp-run-01` and `run-04`). PX4 received 1020 heartbeats but only 893 rate setpoints, consistent with rate setpoints being withheld during the roughly 2 s pre-Offboard stream.

| Observed measurement | Value |
| --- | --- |
| Maximum height above initial ground | 2.056 m |
| Initial hover stable window | 5.039 s |
| Initial hover position RMS error | 0.083 m |
| Second hover stable window | 5.036 s |
| Second hover position RMS error | 0.125 m |
| Second target displacement | North 1.0 m, East 0.5 m, yaw +0.25 rad |
| Final landing | PX4 native NAV_LAND, land detection and ground disarm observed |

All three nonzero body-rate axes and positive thrust were observed at the PX4 ROS input, and actual measured motion satisfied the hold criteria. Offboard, arm and land commands received accepted PX4 acknowledgements. No preflight/failsafe parameters were disabled or force-arming used.

The test keeps odometry freshness at 0.5 s and allows 1.5 s for VehicleStatus: PX4 Commander normally publishes unchanged status at 2 Hz (`Commander.cpp`), so a 0.5 s status timeout would reject normal operation. The 0.60 initial hover estimate did not lift this motor model; the model-derived calibration above is required. Neither the flight script, acceptance criteria nor PX4 safety parameters were changed.
