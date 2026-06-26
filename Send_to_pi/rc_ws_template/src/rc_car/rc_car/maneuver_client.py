"""
maneuver_client.py — send one maneuver and print feedback/result.

Usage:
    ros2 run rc_car maneuver_client turn 90
    ros2 run rc_car maneuver_client move 1.0
    ros2 run rc_car maneuver_client drive_heading 1.0 --hold 0
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rc_car_interfaces.action import Maneuver


class Client(Node):
    def __init__(self):
        super().__init__('maneuver_client')
        self._ac = ActionClient(self, Maneuver, 'maneuver')

    def send(self, kind, value, speed=0.0, hold=0.0):
        self._ac.wait_for_server()
        goal = Maneuver.Goal()
        goal.maneuver = kind
        goal.value = float(value)
        goal.speed = float(speed)
        goal.hold_heading = float(hold)
        fut = self._ac.send_goal_async(goal, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('goal rejected')
            return
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        r = rfut.result().result
        self.get_logger().info(
            f'result: success={r.success} heading={r.final_heading_deg:.1f} '
            f'dist={r.measured_distance_m:.2f} err={r.heading_error_deg:.1f} '
            f'msg="{r.message}"')

    def _fb(self, fb):
        f = fb.feedback
        self.get_logger().info(
            f'progress={f.progress:.2f} hdg={f.current_heading_deg:.1f} '
            f'herr={f.heading_error_deg:.1f} dist={f.measured_distance_m:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = Client()
    argv = sys.argv[1:]
    kind = argv[0] if len(argv) > 0 else 'stop'
    value = float(argv[1]) if len(argv) > 1 else 0.0
    hold = 0.0
    if '--hold' in argv:
        hold = float(argv[argv.index('--hold') + 1])
    try:
        node.send(kind, value, hold=hold)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
