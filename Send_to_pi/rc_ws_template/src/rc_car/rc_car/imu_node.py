"""
imu_node.py — reads the MPU over I2C and publishes feedback for the controller.

Publishes:
    /imu/data        (sensor_msgs/Imu)   orientation, angular_velocity, linear_accel
    /car/heading_deg (std_msgs/Float32)  yaw in degrees (gyro-Z integrated)
    /imu/speed_est   (std_msgs/Float32)  forward speed estimate  [m/s]  (rough)
    /imu/distance_est(std_msgs/Float32)  forward distance estimate [m]  (rough)
Service:
    /imu/reset       (std_srvs/Trigger)  zero the speed/distance integrators

HEADING (yaw) from gyro-Z integration is reliable short-term and is the primary
feedback for turns. ROLL/PITCH use a complementary filter (gyro + accel).

SPEED/DISTANCE from accelerometer double-integration DRIFT — they are published
for VALIDATION only. Zero-velocity update (ZUPT) limits the drift when the car
is still, but do not trust them for control. Wheel encoders are the real
distance source (you'll add them next).
"""

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from .mpu6050 import MPU
from .pid import euler_to_quaternion, angle_wrap_deg

G = 9.80665
DEG2RAD = math.pi / 180.0


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', 104)        # 0x68
        self.declare_parameter('sample_rate', 100.0)  # Hz
        self.declare_parameter('gyro_z_sign', 1.0)    # flip if heading goes the wrong way
        self.declare_parameter('comp_alpha', 0.98)    # complementary filter weight
        self.declare_parameter('publish_velocity', True)
        self.declare_parameter('accel_forward_sign', 1.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calib_samples', 400)

        bus = int(self.get_parameter('i2c_bus').value)
        addr = int(self.get_parameter('address').value)
        self.rate = float(self.get_parameter('sample_rate').value)
        self.gyro_z_sign = float(self.get_parameter('gyro_z_sign').value)
        self.alpha = float(self.get_parameter('comp_alpha').value)
        self.publish_velocity = bool(self.get_parameter('publish_velocity').value)
        self.accel_fwd_sign = float(self.get_parameter('accel_forward_sign').value)
        self.frame_id = self.get_parameter('frame_id').value
        calib_n = int(self.get_parameter('calib_samples').value)

        self.mpu = MPU(bus=bus, addr=addr)
        self.mpu.begin()
        try:
            who = self.mpu.who_am_i()
            self.get_logger().info(f'MPU WHO_AM_I = 0x{who:02X}')
        except Exception as e:
            self.get_logger().warn(f'WHO_AM_I read failed: {e}')

        self.get_logger().info('Calibrating gyro bias — keep the car STILL...')
        self.gz_bias = self._calibrate(calib_n)
        self.get_logger().info(f'gyro-Z bias = {self.gz_bias:.3f} deg/s')

        # state
        self.yaw = 0.0          # deg (continuous, integrated)
        self.roll = 0.0         # deg
        self.pitch = 0.0        # deg
        self.vel = 0.0          # m/s forward (estimate)
        self.dist = 0.0         # m forward (estimate)
        self.a_fwd_bias = 0.0   # accel bias placeholder
        self._still_count = 0
        self.last_t = self.get_clock().now()

        self.pub_imu = self.create_publisher(Imu, 'imu/data', 20)
        self.pub_head = self.create_publisher(Float32, 'car/heading_deg', 20)
        self.pub_spd = self.create_publisher(Float32, 'imu/speed_est', 20)
        self.pub_dst = self.create_publisher(Float32, 'imu/distance_est', 20)
        self.create_service(Trigger, 'imu/reset', self._on_reset)

        self.create_timer(1.0 / self.rate, self._tick)

    def _calibrate(self, n):
        s = 0.0
        for _ in range(n):
            _, _, _, _, _, gz = self.mpu.read()
            s += gz
            time.sleep(0.002)
        return s / max(1, n)

    def _on_reset(self, request, response):
        self.vel = 0.0
        self.dist = 0.0
        response.success = True
        response.message = 'speed/distance integrators zeroed'
        return response

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0 or dt > 0.5:
            return

        ax, ay, az, gx, gy, gz = self.mpu.read()
        gz_corr = self.gyro_z_sign * (gz - self.gz_bias)

        # yaw: integrate corrected gyro-Z (drift-aware; primary feedback)
        self.yaw = angle_wrap_deg(self.yaw + gz_corr * dt)

        # roll/pitch: complementary filter (accel reference + gyro integration)
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * pitch_acc

        # --- Imu message ---
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        qx, qy, qz, qw = euler_to_quaternion(self.roll * DEG2RAD,
                                             self.pitch * DEG2RAD,
                                             self.yaw * DEG2RAD)
        msg.orientation.x, msg.orientation.y = qx, qy
        msg.orientation.z, msg.orientation.w = qz, qw
        # yaw drifts -> large yaw covariance; roll/pitch better
        msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.25]
        msg.angular_velocity.x = gx * DEG2RAD
        msg.angular_velocity.y = gy * DEG2RAD
        msg.angular_velocity.z = gz_corr * DEG2RAD
        msg.angular_velocity_covariance = [0.0009, 0, 0, 0, 0.0009, 0, 0, 0, 0.0009]
        msg.linear_acceleration.x = ax * G
        msg.linear_acceleration.y = ay * G
        msg.linear_acceleration.z = az * G
        msg.linear_acceleration_covariance = [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04]
        self.pub_imu.publish(msg)

        self.pub_head.publish(Float32(data=float(self.yaw)))

        # --- rough forward speed/distance (validation only) ---
        if self.publish_velocity:
            a_fwd = self.accel_fwd_sign * ax * G   # body-x forward accel (gravity ignored ~flat)
            # ZUPT: if barely accelerating and barely rotating, assume stopped
            still = abs(a_fwd) < 0.15 and abs(gz_corr) < 1.0
            if still:
                self._still_count += 1
            else:
                self._still_count = 0
            if self._still_count > int(0.3 * self.rate):
                self.vel = 0.0   # zero-velocity update
            else:
                self.vel += a_fwd * dt
                self.dist += self.vel * dt
            self.pub_spd.publish(Float32(data=float(self.vel)))
            self.pub_dst.publish(Float32(data=float(self.dist)))

    def destroy_node(self):
        try:
            self.mpu.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
