# Validation record — 2026-09-19

## Automated checks

The multistage Docker build runs 47 unit and failure tests, then starts a real
ROS 2 node and exercises its services, topics, result history and SIGTERM cleanup.
The final build passed all checks.

Covered behaviours include full simulated travel, relative position, calibrated
mm/force conversion, uncalibrated-request rejection, effort/speed/acceleration
limits, busy rejection, stop/cancellation (including before velocity rises),
timeout hold, cable loss without command replay, fault latching, no fault reset,
startup/shutdown without energizing an idle motor, and opening through residual
clamp torque. Protocol tests use real OS pseudo-terminals and pyserial, including
exclusive port ownership, wrong motor IDs, fragmented replies, invalid values,
adapter errors, missing feedback, and the exact command register layout.

ROS simulator benchmark (20 warm service calls, desktop Linux amd64):

- Median service acknowledgement: 0.597 ms.
- P95 service acknowledgement: 0.716 ms.
- P95 acceptance-to-dispatch: 0.342 ms.

These simulated timings are not USB/motor response measurements. A fresh CLI
invocation also has process startup and DDS discovery overhead.

## Actual gripper

The standalone runtime image was transferred to the Picker NUC and launched
with only its fdcanusb serial device mapped. The existing robot/camera container
was not used to execute these commands. No arm/base motion was issued.
Operator explicitly confirmed empty jaws and clearance for one close/open cycle.

Read-only telemetry test:

- 201 messages at 49.99 Hz.
- Serial round-trip median 1.56 ms, maximum 3.43 ms in the four-second sample.
- Position -0.159683 rev, motor fault 0, bus approximately 23.94 V,
  temperature approximately 39.74 °C.
- Raw motor torque available; no measured jaw-force calibration installed.

Close at 1.0 rev/s, 1.0 rev/s², 3.50 Nm:

- Service acknowledgement 0.601 ms; acceptance-to-dispatch 1.796 ms.
- Completed in 1.006 s with `contact`.
- Measured position +0.132523 rev, opening fraction 0.426677, torque 3.380 Nm.
- The configured +0.35 rev endpoint was NOT reached. This repeats existing
  mechanical-resistance behaviour; it is not a full-close validation.

The initial opening attempt returned `blocked` after 0.361 s while residual
closing torque remained present. The connector performed a measured-position
hold. The condition incorrectly applied closing-contact detection to opening.
The implementation was corrected to match the commissioned driver's semantics:
opening completes only on position arrival or its bounded deadline. A regression
test reproduces delayed clamp unloading. Limits were not increased.

After deploying that corrected image, the remaining opening completed:

- Outcome `reached`, stationary, in 2.978 s.
- Acceptance-to-dispatch 2.130 ms.
- Position -0.157730 rev, within the configured 0.002 rev arrival tolerance;
  opening fraction 0.996135 and torque -0.968 Nm at completion.
- The motor remains targeted at the saved -0.1597 rev open endpoint.

The following idle read confirmed -0.159683 rev (99.9967% opening), fault 0,
and stationary feedback. Docker measured 28.08 MiB RAM and 6.48% of one CPU
core in a single idle sample on the NUC at 50 Hz.

This validates real telemetry, one closing motion and the corrected opening.
It is not a long-duration endurance qualification or physical load-cell
calibration. The full suite covers partial moves, cancellation and faults in
simulation; those were not additionally exercised on hardware in this one cycle.

## Image

Local final tested runtime config digest before registry publication:
`sha256:db01373bca0fc2af9e00a3539325d5bc32e8863ff85f262f6911175a2fa19b4e`.
The local Docker containerd store reports approximately 152 MB of compressed
image content and `docker images` reports 706 MB of combined local storage;
these are different size measurements, not a 152 MB unpacked filesystem.
The installed connector payload is approximately 2.2 MB. No compiler is present
in the runtime.
Registry rebuild digests can differ; use the published immutable SHA tag/digest
when pinning deployments. CI runs the same tests before publishing.

## Published release and clean deployment

[GitHub Actions run 35458188194](https://github.com/abhinavpathak9873/mini_handler_ros_connector/actions/runs/35458188194)
passed all 47 tests and ROS integration, then published the public image:

`ghcr.io/abhinavpathak9873/mini_handler_ros_connector:sha-c8a4534`

The same image is tagged `latest`. Published OCI index digest:
`sha256:7233e591ef492f6b91885fe35d34f45c2b63ece28353774fd22ad0731dca0ca0`.

An anonymous pull with an empty Docker authentication configuration succeeded.
A fresh simulator container from the published image accepted an 80% opening
command with 50% speed/acceleration, reached it in 0.986 s, and stopped cleanly.

On the Picker NUC, the local test container was gracefully stopped and replaced
by the published image using the repository's production Compose file and
the adapter's stable `/dev/serial/by-id` path. Its digest matched the registry
digest above. Read-only feedback confirmed connected, not simulated, not busy,
fault 0, position -0.159698 rev (99.9997% configured opening), approximately
23.94 V, and a 1.68 ms serial round trip. No additional motion was requested.
The existing mobile-manipulation container remained healthy and running.

This standalone deployment currently owns the serial port, on ROS domain 0.
Stop it before using the original gripper driver; this release does not silently
rewire the tray stack to a new gripper interface.

## Remaining limits

- The 55–105 mm scale is nominal; mm commands are disabled by default.
- There is motor torque feedback, not an installed fingertip force sensor.
  Newton commands require a measured conversion and remain estimates.
- The provisional closed endpoint remains unvalidated because of resistance.
- Loss of communication cannot guarantee a physical stop. No firmware watchdog
  was commissioned. Stop is a software measured-position hold.
- Linux amd64 tested; ARM, Docker Desktop USB, other adapter types, endurance,
  EMC and network-failure qualification are not claimed.
