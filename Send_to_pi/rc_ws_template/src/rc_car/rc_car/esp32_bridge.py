"""
esp32_bridge.py — ROS 2 <-> ESP32 UART bridge.

Subscribes:
    /cmd_vel  (geometry_msgs/Twist)   linear.x [m/s], angular.z [rad/s]
Publishes:
    /car/battery (std_msgs/Float32)   volts
    /car/busy    (std_msgs/Bool)      true while an ESP32 timed maneuver runs

It re-sends the latest command at a fixed rate so the ESP32's 600 ms link
failsafe never trips, and zeroes the command if /cmd_vel goes stale.
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node

import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_linear', 1.0)    # m/s mapped to full throttle (255)
        self.declare_parameter('max_angular', 3.0)   # rad/s mapped to full steer (255)
        self.declare_parameter('angular_sign', -1.0)  # +z (CCW/left) -> -steer (ESP32 +steer = right)
        self.declare_parameter('cmd_timeout', 0.4)   # s; zero command if older than this
        self.declare_parameter('send_rate', 20.0)    # Hz

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.angular_sign = float(self.get_parameter('angular_sign').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        send_rate = float(self.get_parameter('send_rate').value)

        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        self._wlock = threading.Lock()

        self._last_twist = Twist()
        self._last_twist_t = 0.0

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.pub_batt = self.create_publisher(Float32, 'car/battery', 10)
        self.pub_busy = self.create_publisher(Bool, 'car/busy', 10)

        self.create_timer(1.0 / send_rate, self._tick)

        self._stop = False
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self._send({'cmd': 'stop'})   # safety on startup
        self.get_logger().info(f'esp32_bridge up on {self.port} @ {self.baud}')

    # ---- command path -------------------------------------------------------
    def _on_cmd_vel(self, msg):
        self._last_twist = msg
        self._last_twist_t = time.monotonic()

    def _tick(self):
        twist = self._last_twist
        if (time.monotonic() - self._last_twist_t) > self.cmd_timeout:
            twist = Twist()   # stale -> stop
        throttle = clamp(twist.linear.x / self.max_linear * 255.0, -255, 255)
        steer = clamp(self.angular_sign * twist.angular.z / self.max_angular * 255.0,
                      -255, 255)
        self._send({'cmd': 'drive', 't': int(throttle), 's': int(steer)})

    def _send(self, obj):
        line = (json.dumps(obj, separators=(',', ':')) + '\n').encode()
        with self._wlock:
            self._ser.write(line)
            self._ser.flush()

    # ---- status path --------------------------------------------------------
    def _read_loop(self):
        buf = b''
        while not self._stop:
            try:
                data = self._ser.read(256)
                if not data:
                    continue
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    self._handle_status(line)
            except Exception:
                time.sleep(0.1)

    def _handle_status(self, raw):
        try:
            msg = json.loads(raw.decode('utf-8', 'ignore').strip())
        except Exception:
            return
        if not isinstance(msg, dict) or 'cmd' in msg:
            return
        if 'batt' in msg:
            self.pub_batt.publish(Float32(data=float(msg['batt'])))
        if 'auto' in msg:
            self.pub_busy.publish(Bool(data=bool(msg['auto'])))

    def destroy_node(self):
        self._stop = True
        try:
            self._send({'cmd': 'stop'})
            time.sleep(0.05)
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
