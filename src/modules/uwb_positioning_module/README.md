# UWB Indoor Positioning for PX4

Single-anchor UWB positioning that feeds EKF2 through the external vision interface, allowing GPS to be disabled for indoor flight.

## Overview

A UWB anchor sits on the ground with its antenna facing up. The vehicle carries the second antenna facing down. The anchor defines the origin of the NED frame.

The module converts each UWB range + angle-of-arrival measurement into a vehicle position, rotates it from the body frame into NED using the current vehicle attitude, and publishes it as `vehicle_visual_odometry`. EKF2 fuses it as an external vision position.

**Target setup:** ~2 m altitude in a 6 × 3 × 3 m volume, single anchor.

## Why the attitude transform matters

The angles reported by the SR150 are in the *sensor* frame, which is bolted to the airframe. If the vehicle pitches 20° while hovering in place, the reported elevation changes even though the vehicle has not moved.

Multiplying by the body→NED rotation cancels this: the measured angle changes by 20°, the rotation matrix changes by 20° in the opposite sense, and the resulting NED position is unchanged. This is what separates tilt from translation.

## Data flow

```
SR150 shield
    │ UART @ 115200
    ▼
uwb_sr150 driver ──────► sensor_uwb (uORB)
                              │ distance, aoa_azimuth_dev, aoa_elevation_dev
                              ▼
                     uwb_positioning_module ◄──── vehicle_attitude (q)
                              │
                              ▼
                   vehicle_visual_odometry (uORB)
                              │
                              ▼
                      EKF2 external vision
```

No MAVLink involved — EKF2 subscribes to `vehicle_visual_odometry` directly (`src/modules/ekf2/EKF2.hpp:339`).

## Files added

```
src/modules/uwb_positioning_module/
├── UwbPositioningModule.hpp     class declaration, subscriptions, params
├── UwbPositioningModule.cpp     implementation
├── params.yaml                  UWB_POS_* parameter definitions
├── CMakeLists.txt               px4_add_module
└── Kconfig                      MODULES_UWB_POSITIONING_MODULE
```

## Files modified

| File | Change |
|---|---|
| `boards/px4/fmu-v6xrt/default.px4board:89` | `CONFIG_MODULES_UWB_POSITIONING_MODULE=y` |
| `boards/px4/fmu-v6xrt/default.px4board:122` | `CONFIG_DRIVERS_UWB_UWB_SR150=y` |
| `ROMFS/px4fmu_common/init.d/rcS:577-582` | auto-start block |

Auto-start, placed after the `mag_bias_estimator` block and following the same idiom:

```sh
# UWB single-anchor positioning
if param compare -s UWB_POS_EN 1
then
	uwb_positioning_module start
fi
```

`-s` silences the check on boards where the parameter doesn't exist.

## Position computation

`UwbPositioningModule.cpp:93-125`. Inputs: `distance` (m), `aoa_azimuth_dev` (deg), `aoa_elevation_dev` (deg) from `sensor_uwb`; `q` from `vehicle_attitude`.

**Step 1 — spherical to Cartesian, sensor frame:**

```
N = -d · cos(azimuth) · sin(elevation)
E =  d · sin(azimuth) · sin(elevation)
Z = -d · cos(elevation)
```

```cpp
const Vector3f v_sensor(-d * cosf(az) * sinf(el),
                         d * sinf(az),
                        -d * cosf(el));
```

**Step 2 — lever arm.** Adds `offset_x/y/z` from the topic (sourced from the driver's `UWB_INIT_OFF_*` params), the vehicle CoG → antenna offset in body axes.

**Step 3 — body FRD → NED:**

```cpp
const Dcmf R_nb(q_nb);
const Vector3f v_ned = R_nb * v_body;
```

**Step 4 — assign.** `pos_ned = v_ned` directly; the minus signs in the formulas already give the anchor→vehicle direction. If the sign comes out inverted on the bench, negate here (`.cpp:122`).

## Published message

`vehicle_visual_odometry` (`vehicle_odometry_s`):

| Field | Value |
|---|---|
| `pose_frame` | `POSE_FRAME_NED` |
| `position[3]` | computed NED position |
| `position_variance[3]` | `UWB_POS_NOISE²` on all three axes |
| `q`, `orientation_variance` | `NAN` — not measured |
| `velocity`, `velocity_variance` | `NAN` — not measured |
| `angular_velocity` | `NAN` |
| `velocity_frame` | `VELOCITY_FRAME_UNKNOWN` |
| `timestamp_sample` | `sensor_uwb.timestamp` |

EKF2 gates each field on finiteness independently, so NaN fields are simply ignored.

## Sample gating

Rejected, with a counter per reason visible in `status`:

| Reason | Condition |
|---|---|
| `disabled` | `UWB_POS_EN == 0` |
| `nonfinite` | distance or either angle not finite |
| `status` | `sensor_uwb.status != 0` |
| `nlos` | `sensor_uwb.nlos != 0` (likely a reflection) |
| `range` | outside `UWB_POS_R_MIN` … `UWB_POS_R_MAX` |
| `attitude` | no attitude, older than 100 ms, or non-finite quaternion |

## Parameters

| Parameter | Default | Unit | Purpose |
|---|---|---|---|
| `UWB_POS_EN` | 0 | — | Enable module (also gates auto-start) |
| `UWB_POS_NOISE` | 0.2 | m | 1σ position noise → reported variance |
| `UWB_POS_R_MIN` | 0.2 | m | Reject near-field samples |
| `UWB_POS_R_MAX` | 10.0 | m | Reject implausible ranges |

## Setup

**1. UWB driver** — assign the serial port, then reboot:

```
param set UWB_PORT_CFG <port>
param save
reboot
```

Verify: `listener sensor_uwb`

if it didn't work then
```
uwb_sr150 stop
uwb_sr150 start -d /dev/ttyS5 (this switches it to TELEM2)
````

**2. Positioning module:**

```
param set UWB_POS_EN 1
param save
```

Auto-starts on subsequent boots. Verify:

```
uwb_positioning_module status
listener vehicle_visual_odometry
```

**3. EKF2** — only after confirming the published position looks correct:

```
param set EKF2_EV_CTRL 3      # bit0 horizontal + bit1 vertical position
param set EKF2_HGT_REF 3      # External Vision as height reference
param set EKF2_GPS_CTRL 0     # disable GNSS
param save
reboot
```

`EKF2_EV_CTRL` bit 3 (yaw) is deliberately **not** set — the module does not produce an attitude estimate.

## Implementation notes

- Runs on a `sensor_uwb` uORB callback (`px4::wq_configurations::lp_default`), so it executes once per measurement with no polling.
- Attitude is read from `vehicle_attitude`, not raw IMU. EKF2 already fuses gyro/accel/mag to produce that quaternion; re-integrating it here would create a second, worse estimate competing with EKF2's.
- Position variance is currently isotropic (`UWB_POS_NOISE²` on all axes). The real error is anisotropic — range is accurate to centimetres while angular error produces lateral error of roughly `range × σ_angle` — so this is a simplification that may need revisiting.

## Diagnostics

```
uwb_positioning_module status      # counts, rejection reasons, last NED position
listener sensor_uwb                # raw sensor input
listener vehicle_visual_odometry   # module output
listener estimator_status          # EKF2 fusion state
```

