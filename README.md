# mini_handler_ros_connector

A standalone ROS 2 Humble driver and small Docker image for the mini-handler
parallel gripper, using a HighTorque motor through an **mjbots fdcanusb** adapter.
One process owns one serial port. No arm stack, MoveIt, cameras, GPU, or desktop
installation is required. Connecting only reads the motor; movement requires a
command. The serial connection stays open between commands.

## Start in two commands (Linux)

```bash
git clone https://github.com/abhinavpathak9873/mini_handler_ros_connector.git
cd mini_handler_ros_connector
docker compose up --build -d
```

Default device is `/dev/ttyACM0`. For a stable device identity:

```bash
SERIAL_PORT=/dev/serial/by-id/YOUR_FDCANUSB_ADAPTER docker compose up --build -d
```

The motor needs its external 24 V supply. Only the selected serial device is
passed through; no privileged container or Docker socket is needed. Linux host
networking makes ROS communication on this computer work without port mapping.
For the image published by this repository's CI:

```bash
docker compose pull
docker compose up -d --no-build
```

Or run the image directly:

```bash
docker run --rm --init --network host --device /dev/ttyACM0:/dev/mini_handler \
  ghcr.io/abhinavpathak9873/mini_handler_ros_connector:latest
```

The supported release platform is Linux amd64. USB passthrough on Docker
Desktop/WSL and ARM builds are not covered by this release.

## Commands

Each CLI call prints acceptance and then the final result. All values are
explicit: opening `1` is the configured open endpoint, `0` is closed.

```bash
docker compose exec gripper /entrypoint.sh mini-handler status
docker compose exec gripper /entrypoint.sh mini-handler open
docker compose exec gripper /entrypoint.sh mini-handler close
docker compose exec gripper /entrypoint.sh mini-handler opening 0.5
docker compose exec gripper /entrypoint.sh mini-handler relative -0.1
docker compose exec gripper /entrypoint.sh mini-handler relative 0.1
docker compose exec gripper /entrypoint.sh mini-handler close --speed 0.5 --accel 0.25
docker compose exec gripper /entrypoint.sh mini-handler grip_torque 2.5
docker compose exec gripper /entrypoint.sh mini-handler stop
```

`relative -0.1` reduces opening by **10 percentage points of total travel**;
it does not mean 10% of the current opening. `--speed` and `--accel` are fractions
of independently configured absolute limits. `--torque` sets a per-command
ceiling for ordinary position/open/close commands. `--timeout` sets seconds.
`--no-wait` returns after acceptance; use the returned command ID to query the
result. Launching a CLI has Python/ROS discovery overhead. Use a persistent ROS
service client for the shortest command-to-execution path.

After measuring/calibrating jaw widths and force, these also become available:

```bash
docker compose exec gripper /entrypoint.sh mini-handler width 80
docker compose exec gripper /entrypoint.sh mini-handler relative_mm -5
docker compose exec gripper /entrypoint.sh mini-handler grip_force 100
```

`width` specifies total jaw separation in mm; `relative_mm -5` reduces the total
gap by 5 mm. `grip_force 100` asks for a **calibrated torque-derived estimate** of
100 N. There is no installed fingertip load cell. It cannot promise an exact
physical jaw force, and torque includes internal friction and mechanical loads.

## ROS integration

Default namespace is `/mini_handler`. Override with standard ROS remapping
`--ros-args -r __ns:=/my_gripper`. The node is named `mini_handler`.

| Endpoint | Type | Meaning |
| --- | --- | --- |
| `command` | `mini_handler_ros_connector/srv/Command` | Immediate accept/reject + command ID |
| `open`, `close`, `stop` | `std_srvs/srv/Trigger` | Convenience acceptance; `message` is command ID |
| `state` | `mini_handler_ros_connector/msg/State` | Freshness, position, velocity, torque, opening, nominal/calibrated width, force estimate, voltage, temperature, fault, busy |
| `results` | `mini_handler_ros_connector/msg/Result` | Terminal result for each accepted command |
| `get_result` | `mini_handler_ros_connector/srv/GetResult` | Retrieve active/completed status by ID; last 128 results retained |
| `joint_states` | `sensor_msgs/msg/JointState` | Motor-shaft radians, rad/s, Nm; not a fictitious linear finger joint |

State defaults to 50 Hz, best-effort, keep-last 1. Results are reliable,
keep-last 32, volatile. Services are reliable. Subscribe with sensor-data QoS
for state. `get_result` avoids missed-event races for clients that attach after
acceptance. Completion history is in memory and is lost on restart.

```bash
# Inside a sourced ROS environment with this package installed:
ros2 service call /mini_handler/command mini_handler_ros_connector/srv/Command \
  "{operation: opening, value: 0.5, speed_scale: 0.5, acceleration_scale: 0.5}"
ros2 service call /mini_handler/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /mini_handler/state --qos-reliability best_effort
```

See [examples/client.py](examples/client.py) for a persistent Python ROS client.
There is intentionally no command topic that can silently replay a retained
movement, and no FIFO of stale movement commands. A second command is rejected
while busy. Stop interrupts the current command and confirms a stationary hold.

Terminal outcomes:

- `reached`: target reached, velocity settled; full close only if endpoint reached.
- `contact`: close/grip stopped at sustained torque before the endpoint. This is
  resistance evidence, not proof of an object or full closure.
- `blocked`: a distance/opening command could not reach its target, or grip reached
  the closed endpoint without its requested effort.
- `cancelled`, `timeout`, `fault`, `communication_error`: explicit failures.

Services acknowledge receipt quickly; acceptance is **not movement success**.
`accepted_to_send_ms` measures worker dispatch (including its fresh pre-command
read), not mechanical response. `last_transport_rtt_ms` measures a serial query.
These are ordinary Linux/USB timings, not hard real-time guarantees.

## Configuration and calibration

Edit [config/gripper.yaml](config/gripper.yaml), then
`docker compose up -d --force-recreate`. Parameters are read-only while running;
per-command scaling, effort, and timeouts are adjustable on every request.

| Setting | Default | Meaning |
| --- | ---: | --- |
| `port`, `motor_id` | `/dev/mini_handler`, `1` | Exclusive serial ownership and motor address |
| `poll_hz` | 50 | Feedback rate; 1..200 configurable |
| `request_timeout_s` | 0.10 | Bounded serial transaction |
| `open_position_rev`, `close_position_rev` | -0.1597, +0.35 | Existing operator-supplied encoder travel |
| `max_speed_rps` | 1.0 | Output-shaft rev/s ceiling |
| `max_acceleration_rps2` | 1.0 | Output-shaft rev/s² ceiling |
| `open_torque_nm`, `close_torque_nm` | 3.07, 3.50 | Default direction limits |
| `max_torque_nm` | 5.0 | Independent configurable command ceiling |
| `default_speed_scale`, `default_acceleration_scale` | 1.0, 1.0 | Defaults for omitted/zero request values |
| `command_timeout_s`, `settle_time_s` | 12.0, 0.15 | Deadline and stable completion duration |
| `position_tolerance_rev` | 0.002 | Arrival tolerance |
| `stationary_velocity_rps` | 0.01 | Stationary threshold |
| `contact_ratio` | 0.95 | Sustained stationary torque / requested torque ratio |
| `extended_telemetry` | true | Include float bus voltage and temperature |

The existing open endpoint has operator confirmation. The +0.35 closed endpoint
has previously been obstructed before arrival; full mechanical closure and
absolute millimetre calibration are provisional. Configure your own unit's
measured endpoints. No automatic homing or pushing beyond travel is performed.

The old 55–105 mm labels are published with `width_calibrated=false`. Millimetre
commands are rejected until measured endpoint widths are saved and
`width_calibrated=true`. `opening_fraction` always expresses the configured
encoder span. Measurements outside that span are not clamped or hidden.

`force_n_per_nm: 0.0` disables newton commands and reports `estimated_force_n=NaN`.
Measure opposing jaw compression with a load cell over the intended gap and
torque range before setting a conversion. The simple linear conversion remains
an estimate. A CAD-only ideal conversion is deliberately not enabled.

On idle startup and after a fault, the connector never resets the motor. A
communication failure latches the session and requires an explicit container
restart after recovery; it never reconnects and replays a movement automatically.
Only one driver may own the port, including the original Picker driver.

Stop and graceful shutdown use fresh position feedback to establish a hold.
An already stationary loaded gripper retains its clamp. Serial disconnect,
process kill, power loss, or USB failure cannot guarantee a physical stop;
there is no commissioned firmware watchdog in this release. Fault results say
`HOLD UNCONFIRMED` when feedback/stop cannot be established. Software Stop is
not a physical E-stop. Do not raise limits based on this software alone.

## Networking and multiple grippers

Commands executed inside the container require no LAN setup. Host clients need
the same `ROS_DOMAIN_ID` (default 0) and generated interfaces. For Picker domain
63, use `ROS_DOMAIN_ID=63 docker compose up -d`. ROS on another computer also
requires working DDS discovery/routing through the LAN/firewall; host networking
does not remove those external requirements. The connector itself has no robot
IP or fixed network-interface configuration.

For multiple grippers, run one container per serial device, with distinct ROS
namespaces. USB hotplug may change the kernel device: recreate the container
after reconnecting, preferably using a `/dev/serial/by-id` path.

## Simulation and tests

```bash
docker compose -f compose.sim.yaml up --build -d
docker compose -f compose.sim.yaml exec gripper /entrypoint.sh mini-handler close
docker compose -f compose.sim.yaml exec gripper /entrypoint.sh mini-handler open
docker compose -f compose.sim.yaml down
```

Simulator contact defaults to 35% opening; `simulated_contact_fraction: -1.0`
enables empty full travel. The ROS state explicitly reports `simulated=true`.
Every image build runs unit/failure tests, a real pseudo-terminal serial test,
and ROS service/message integration tests. Runtime contains no compiler or test
suite. See [VALIDATION.md](VALIDATION.md) for measured results and limits.

Native Humble workspace build:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mini_handler_ros_connector
source install/setup.bash
ros2 run mini_handler_ros_connector connector --ros-args \
  --params-file src/mini_handler_ros_connector/config/gripper.yaml
```

Build instructions assume the repository is `src/mini_handler_ros_connector`.
Protocol provenance is documented in [NOTICE](NOTICE). Licensed under MIT.
