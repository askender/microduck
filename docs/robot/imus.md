# The two IMUs

A duck carries **two IMUs on two different buses, read by two different
daemons, for two different reasons**. The control loop cannot run without the
first; nothing in the tree reads the second.

| | Trunk | Head |
|---|---|---|
| Chip | LSM6DSV16X on the `imu_to_dxl` v2 board | BMI088 on the head HAT |
| Bus | Dynamixel UART, **id 200** (`/dev/ttyS2`, 1 Mbps, protocol v2) | I²C, a bus `tofd` owns |
| Read by | `robotd`'s control loop — in the same 50 Hz `sync_read` as the 15 servos | `tofd`, on its own thread |
| Fusion | **on-chip** — the SFLP block (game-rotation quaternion + gyro-bias estimate) | **host-side** — a Madgwick AHRS in `tofd` |
| Rate | 50 Hz, one per control tick | 100 Hz default |
| Default | always on | **off** — `[head_imu] enabled = true` in `robotd.toml`, then restart `tofd` |
| Consumer | control-loop observations, fall detection (gravity), odometry heading | none today — `head_imu.stream` has no subscriber |

## The trunk IMU is a servo-bus node — not a sensor wired to the main board

The v2 board is a genuine Dynamixel-protocol slave: it sits on the servo bus
as id 200, answers the same register protocol the XL330s do, and its data
comes back in the same transaction as the joints. Nothing on the main board is
involved, and the host does not "disguise" IMU data as servo data — the board
itself *is* a servo-protocol device, by design.

- One combined `sync_read` per tick covers the IMU block and all 15 servo
  blocks. The IMU is listed first in the id vector so it answers before the
  servo burst (`duck-control/src/bus.rs`).
- The control loop consumes the first **12 bytes of the register block at
  address 124**: 6 bytes of gyro (x, y, z as i16 LE, ±500 dps,
  17.5 mdps/LSB), then 6 bytes of SFLP quaternion (x, y, z as IEEE
  half-precision; `w = √(1−x²−y²−z²)` is reconstructed on the host). The
  board serves a 20-byte diagnostic block; the loop reads 12 so the whole
  sensor round-trip stays in one transaction.
- The chip does the fusion. The SFLP block outputs a game-rotation quaternion
  and estimates its own gyro bias; the host only decodes —
  `duck-control/src/imu.rs` keeps a last-good quaternion and rejects spikes,
  but runs no filter of its own.
- A board that answers without refreshing is detected, not trusted:
  `StaleImuTracker` counts consecutive identical blocks ("orientation is
  frozen"), and `imu_stale()` / `imu_ready()` ride the `RobotIo` trait into
  `robot.health`.

Why a bus node: one transaction per tick means the IMU sample and the fifteen
joint samples share a timestamp — no cross-sensor clock sync in a 50 Hz loop —
and there is no second port to own. [`robotd-design.md`](../design/robotd-design.md)
§1.1 owns that reasoning.

What the loop makes of it: `ImuData { gyro, gravity, quat }` — gyro in trunk
rad/s, gravity (upright `[0, 0, −1]`) as the policy observable and
fall-detection threshold, and the scalar-first trunk→world quaternion.
Odometry integrates the IMU yaw for heading, so the world frame is wherever
the duck looked at boot.

## The head IMU is read by `tofd`, and left off

The BMI088 rides the head HAT, rigid to the camera, on the I²C bus `tofd`
owns — so `tofd` reads it, on its own thread at 100 Hz (~4.5% of a core),
fusing host-side into head-frame orientation, and serves it as
`head_imu.stream` (gyro, acceleration, orientation). It is **off by
default**, and nothing subscribes to the stream;
[`tof-on-demand.md`](../project/tof-on-demand.md) owns that story. If you
remember "the IMU has no consumer", this is the one.

## "Can I wire the IMU straight to the main board on a real robot?"

Electrically yes — the chip only needs I²C/SPI. Architecturally it is a
different robot:

1. **A new `RobotIo` implementation.** `duck_control::io::RobotIo` is the only
   hardware seam (six methods: `read`, `write`, `set_gain`, `set_torque`,
   `slow_sensors`, plus the defaulted `imu_stale`/`imu_ready`), and every
   implementation today — `DynamixelIo`, `FakeIo`, `RemoteIo` — gets the IMU
   out of the servo `sync_read`. A separately wired IMU is a fourth
   implementation, and `read()` can no longer return joints and IMU as one
   transaction.
2. **The timestamp alignment goes away.** Today the IMU sample and the joint
   samples are the same tick, one transaction, nothing to reconcile. On a
   separate bus the 50 Hz loop either eats the phase skew between two reads or
   runs the clock sync it never needed.
3. **The host takes back the fusion.** SFLP currently produces the quaternion
   and the gyro-bias estimate on the chip; a directly wired raw IMU puts a
   filter and a bias tracker back in the loop.

The `imu_to_dxl` board exists precisely to make the separate wiring
unnecessary. On a real duck: IMU board on the servo bus as id 200, and the
main board carries no IMU wiring at all.
