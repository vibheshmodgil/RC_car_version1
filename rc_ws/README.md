# RC Car — ROS 2 base (IMU feedback + ESP32 control)

A ROS 2 (Jazzy) workspace that runs on the Raspberry Pi. The Pi reads its own
IMU, closes a heading loop, and drives the car by commanding the ESP32 over
UART. Built so the next pieces — wheel encoders, PID, and a laptop on the same
ROS graph (camera Pi→laptop, commands laptop→Pi) — slot in without rework.

```
                        ROS 2 graph (Pi now, + laptop later)
  ┌───────────┐  /imu/data, /car/heading_deg   ┌──────────────────┐
  │ imu_node  ├───────────────┬───────────────►│ controller_node  │
  │ (MPU/I2C) │  /imu/distance_est              │ (Maneuver action)│
  └───────────┘                                 └────────┬─────────┘
                                                          │ /cmd_vel
                                                          ▼
  ESP32 ◄──UART JSON──  ┌───────────────┐  ◄──────────────┘
  (drive/turn/move)     │ esp32_bridge  │  /car/battery, /car/busy
                        └───────────────┘
```

## 1. Prerequisites (on the Pi)

- Ubuntu 24.04 (arm64) + **ROS 2 Jazzy**, or your Jazzy Docker image.
- Enable I2C **and** the UART, and free the UART from the login console:
  ```bash
  sudo raspi-config
  #  Interface Options -> I2C            -> ENABLE
  #  Interface Options -> Serial Port    -> login shell? NO, hardware? YES
  sudo reboot
  ```
- Python deps:
  ```bash
  pip3 install pyserial smbus2
  ```

## 2. Wiring

**ESP32 ↔ Pi (UART)** — both 3.3 V, no level shifter:

| Pi                    | →  | ESP32        |
|-----------------------|----|--------------|
| GPIO14 TXD (pin 8)    | →  | GPIO16 (RX2) |
| GPIO15 RXD (pin 10)   | ←  | GPIO17 (TX2) |
| GND                   | —  | GND          |

**MPU IMU ↔ Pi (I2C-1):**

| MPU  | →  | Pi                  |
|------|----|---------------------|
| VCC  | →  | 3.3 V (pin 1)       |
| GND  | →  | GND (pin 9)         |
| SDA  | →  | GPIO2 / SDA (pin 3) |
| SCL  | →  | GPIO3 / SCL (pin 5) |

Check it's seen: `i2cdetect -y 1` should show `68`.

## 3. Build

files\bringup_launch.py
ssh shiv@192.168.1.10
scp "C:\Users\vibhe\OneDrive\Desktop\Pi_ROS_setup\files\bringup_launch.py" shiv@192.168.1.10:/home/shiv/Desktop/robot/rc_ws/src/rc_car/launch
192.168.4.1


```bash
mkdir -p ~/rc_ws/src
cp -r rc_car rc_car_interfaces ~/rc_ws/src/   # the two packages from here
cd ~/rc_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

(Interfaces build first automatically — `rc_car` depends on `rc_car_interfaces`.)

## 4. Run

```bash
ros2 launch rc_car bringup.launch.py
```

On startup the IMU node calibrates gyro bias — **keep the car still** for ~1 s.

## 5. First-run sign calibration (do this on blocks, wheels off the ground)

Two signs depend on physical mounting. Check them once:

1. **Heading sign.** Watch the heading while you rotate the car by hand:
   ```bash
   ros2 topic echo /car/heading_deg
   ```
   Turn the car **left (CCW)** → heading should **increase**. If it decreases,
   set `gyro_z_sign: -1.0` in `config/params.yaml`.

2. **Steer sign.** Send a small turn and watch which way it spins:
   ```bash
   ros2 run rc_car maneuver_client turn 30
   ```
   It should rotate left and settle near +30°. If it spins the wrong way and
   never converges (heading runs away), flip `angular_sign` in the bridge params
   (`-1.0` ↔ `1.0`).

Rebuild/relaunch after editing params (params are loaded at launch).

## 6. Maneuvers + validation workflow

```bash
ros2 run rc_car maneuver_client turn 90      # spin to +90° (IMU heading loop)
ros2 run rc_car maneuver_client move 1.0     # drive ~1 m, holding heading
ros2 run rc_car maneuver_client drive_heading 1.0 --hold 0   # 1 m holding 0°
```

The action **result** reports `final_heading_deg` and `measured_distance_m`
(IMU-measured) so you can compare commanded vs measured — that's your IMU
validation. For a clean distance read, reset the integrator before a run:
```bash
ros2 service call /imu/reset std_srvs/srv/Trigger
```

> **Honest limits.** Gyro-integrated **heading** is reliable for turns and
> heading-hold. Accel-integrated **speed/distance** (`/imu/speed_est`,
> `/imu/distance_est`) **drift** — they're for validation/sanity only. That's
> exactly why control of `move` defaults to `distance_source: time`, and why
> wheel encoders are the real fix (next step).

## 7. Laptop on the same ROS graph (for later)

ROS 2 auto-discovers peers that share a `ROS_DOMAIN_ID` on the same LAN. On
**both** Pi and laptop:
```bash
export ROS_DOMAIN_ID=7          # any matching number
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # same RMW on both
```
Then `ros2 topic list` on the laptop shows the Pi's topics, and vice-versa.

- **Camera Pi→laptop:** publish `sensor_msgs/CompressedImage` on the Pi; the
  laptop just subscribes. (Use compressed transport over WiFi.)
- **Laptop→Pi:** publish whatever (goals, `/cmd_vel`, nav commands); the Pi
  subscribes. No code change here — same graph.
- **Mac Docker caveat:** Docker Desktop on macOS runs Linux in a VM, so DDS
  multicast discovery to the LAN usually fails. When you get to it, run a
  **FastDDS Discovery Server** (or set CycloneDDS unicast `Peers`) and point
  both ends at the Pi's IP. Flag me then and I'll give you that config.

## 8. Expansion hooks (already wired for these)

- **Wheel encoders → distance:** publish measured distance on `/car/distance_m`
  from a new `encoder_node`, subscribe to it in `controller_node` (marked
  `ENCODER HOOK`), and set `distance_source: encoder`. The action result already
  carries `measured_distance_m`.
- **Full PID with encoders:** the heading PID is in `pid.py`; add a distance PID
  in the controller once encoder feedback exists (the structure mirrors heading).
- **Sensor fusion / odometry:** `imu_node` publishes a standard
  `sensor_msgs/Imu`, so you can drop in `robot_localization` (EKF) later to fuse
  IMU + encoders into `/odom` + TF for RViz/Nav2.

## Interface reference

| Topic / action        | Type                              | Dir            |
|-----------------------|-----------------------------------|----------------|
| `/cmd_vel`            | geometry_msgs/Twist               | controller→bridge |
| `/imu/data`           | sensor_msgs/Imu                   | imu→           |
| `/car/heading_deg`    | std_msgs/Float32 (deg)            | imu→controller |
| `/imu/speed_est`      | std_msgs/Float32 (m/s, rough)     | imu→           |
| `/imu/distance_est`   | std_msgs/Float32 (m, rough)       | imu→controller |
| `/imu/reset`          | std_srvs/Trigger                  | →imu           |
| `/car/battery`        | std_msgs/Float32 (V)              | bridge→        |
| `/car/busy`           | std_msgs/Bool                     | bridge→        |
| `/maneuver`           | rc_car_interfaces/action/Maneuver | →controller    |
