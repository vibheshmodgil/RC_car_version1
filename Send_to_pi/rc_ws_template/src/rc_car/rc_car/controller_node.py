"""
controller_node.py — closed-loop maneuver controller (Maneuver action server).

Closes the loop on IMU heading (reliable) and emits /cmd_vel. Distance is
pluggable via the 'distance_source' param:
    time     -> elapsed * nominal_speed_mps  (robust default, no sensors needed)
    imu      -> /imu/distance_est            (validation; drifts)
    encoder  -> (add a subscriber later; hook is marked below)

Maneuvers:
    turn          spin in place to (start_heading + value_deg)
    move          drive 'value' meters forward, holding the start heading
    drive_heading drive 'value' meters while holding 'hold_heading'
    stop          publish zero velocity

Send a goal:
    ros2 action send_goal /maneuver rc_car_interfaces/action/Maneuver \
        "{maneuver: 'turn', value: 90.0, speed: 0.0}"
"""

import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from rc_car_interfaces.action import Maneuver
from .pid import PID, clamp, angle_wrap_deg


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('kp_heading', 0.04)     # (rad/s) per deg of error
        self.declare_parameter('ki_heading', 0.0)
        self.declare_parameter('kd_heading', 0.005)
        self.declare_parameter('max_turn_rate', 1.5)   # rad/s
        self.declare_parameter('cruise_speed', 0.4)    # m/s for moves
        self.declare_parameter('heading_tol_deg', 2.0)
        self.declare_parameter('distance_tol_m', 0.05)
        self.declare_parameter('distance_source', 'time')   # time | imu | encoder
        self.declare_parameter('nominal_speed_mps', 0.4)    # used when source == time
        self.declare_parameter('settle_ticks', 5)
        self.declare_parameter('move_timeout_s', 30.0)

        self.rate = float(self.get_parameter('control_rate').value)
        self.max_turn_rate = float(self.get_parameter('max_turn_rate').value)
        self.cruise = float(self.get_parameter('cruise_speed').value)
        self.head_tol = float(self.get_parameter('heading_tol_deg').value)
        self.dist_tol = float(self.get_parameter('distance_tol_m').value)
        self.dist_source = self.get_parameter('distance_source').value
        self.nominal_speed = float(self.get_parameter('nominal_speed_mps').value)
        self.settle_ticks = int(self.get_parameter('settle_ticks').value)
        self.move_timeout = float(self.get_parameter('move_timeout_s').value)

        self.heading_pid = PID(
            float(self.get_parameter('kp_heading').value),
            float(self.get_parameter('ki_heading').value),
            float(self.get_parameter('kd_heading').value),
            out_min=-self.max_turn_rate, out_max=self.max_turn_rate)

        self.current_heading = 0.0     # deg
        self.imu_distance = 0.0        # m (from /imu/distance_est)
        self._have_heading = False

        cb = ReentrantCallbackGroup()
        self.create_subscription(Float32, 'car/heading_deg', self._on_heading, 20,
                                 callback_group=cb)
        self.create_subscription(Float32, 'imu/distance_est', self._on_imu_dist, 20,
                                 callback_group=cb)
        # ENCODER HOOK: when you add encoders, publish distance on /car/distance_m
        # and subscribe to it here, then set distance_source: encoder.

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._server = ActionServer(
            self, Maneuver, 'maneuver',
            execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=cb)

        self.get_logger().info(
            f"controller ready (distance_source={self.dist_source})")

    # ---- feedback subscriptions --------------------------------------------
    def _on_heading(self, msg):
        self.current_heading = msg.data
        self._have_heading = True

    def _on_imu_dist(self, msg):
        self.imu_distance = msg.data

    # ---- helpers ------------------------------------------------------------
    def _publish(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def _stop(self):
        self._publish(0.0, 0.0)

    def _measured_distance(self, start_imu_dist, elapsed):
        if self.dist_source == 'imu':
            return self.imu_distance - start_imu_dist
        # 'time' (and fallback): rough estimate from elapsed * nominal speed
        return elapsed * self.nominal_speed

    # ---- action execution ---------------------------------------------------
    def _execute(self, goal_handle):
        g = goal_handle.request
        kind = g.maneuver.lower()
        dt = 1.0 / self.rate

        # wait briefly for first heading sample
        t0 = time.monotonic()
        while not self._have_heading and (time.monotonic() - t0) < 2.0:
            time.sleep(0.05)

        self.heading_pid.reset()
        start_heading = self.current_heading
        start_imu = self.imu_distance
        result = Maneuver.Result()

        if kind == 'stop':
            self._stop()
            result.success = True
            result.final_heading_deg = self.current_heading
            result.message = 'stopped'
            goal_handle.succeed()
            return result

        if kind == 'turn':
            target = start_heading + g.value
        elif kind in ('move', 'drive_heading'):
            target = g.hold_heading if kind == 'drive_heading' else start_heading
        else:
            self._stop()
            result.success = False
            result.message = f'unknown maneuver "{kind}"'
            goal_handle.abort()
            return result

        cruise = self.cruise if kind in ('move', 'drive_heading') else 0.0
        settled = 0
        start_t = time.monotonic()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                result.success = False
                result.message = 'canceled'
                goal_handle.canceled()
                return result

            now = time.monotonic()
            elapsed = now - start_t
            herr = angle_wrap_deg(target - self.current_heading)
            ang = self.heading_pid.update(herr, self.current_heading, dt)

            if kind == 'turn':
                self._publish(0.0, ang)
                done = abs(herr) <= self.head_tol
                progress = clamp(1.0 - abs(herr) / max(1.0, abs(g.value)), 0.0, 1.0)
                measured = 0.0
            else:  # move / drive_heading
                measured = self._measured_distance(start_imu, elapsed)
                remaining = g.value - measured
                self._publish(cruise if remaining > 0 else 0.0, ang)
                done = abs(remaining) <= self.dist_tol
                progress = clamp(measured / max(1e-3, g.value), 0.0, 1.0)
                if elapsed > self.move_timeout:
                    done = True

            # feedback
            fb = Maneuver.Feedback()
            fb.progress = float(progress)
            fb.current_heading_deg = float(self.current_heading)
            fb.heading_error_deg = float(herr)
            fb.measured_distance_m = float(measured)
            goal_handle.publish_feedback(fb)

            if done:
                settled += 1
                if settled >= self.settle_ticks:
                    break
            else:
                settled = 0
            time.sleep(dt)

        self._stop()
        result.success = True
        result.final_heading_deg = float(self.current_heading)
        result.heading_error_deg = float(angle_wrap_deg(target - self.current_heading))
        result.measured_distance_m = float(self.imu_distance - start_imu)
        result.message = 'done'
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
