"""/mux/cmd_vel (Twist, from agt_cmd_vel_guard) -> /yhs/ctrl_cmd at a fixed rate.

High-level packages only ever publish Twist; this is the only node that knows
the YHS CtrlCmd message. It publishes continuously (driver/manual require
>= 30 Hz) and sends a zero command whenever the input is stale.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from yhs_can_interfaces.msg import CtrlCmd

from .conversion import ConversionError, stop_command, twist_to_ctrl, validate_gear


class YhsCmdVelBridge(Node):
    def __init__(self):
        super().__init__('agt_yhs_cmd_vel_bridge')
        self.declare_parameter('input_topic', '/mux/cmd_vel')
        self.declare_parameter('output_topic', '/yhs/ctrl_cmd')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('drive_gear', -1)      # no default: must be confirmed
        self.declare_parameter('allow_reverse', False)
        self.gear = validate_gear(int(self.get_parameter('drive_gear').value))
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate < 30.0:
            raise ConversionError('publish_rate_hz must be >= 30 (YHS manual)')
        self.timeout = float(self.get_parameter('command_timeout_sec').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.last = None
        self.last_rx = 0.0
        self.pub = self.create_publisher(CtrlCmd, str(self.get_parameter('output_topic').value), 10)
        self.create_subscription(Twist, str(self.get_parameter('input_topic').value), self._on_cmd, 20)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'YHS bridge gear={self.gear} rate={rate:.0f}Hz '
                               f'allow_reverse={self.allow_reverse}')

    def _on_cmd(self, msg):
        self.last = msg
        self.last_rx = time.monotonic()

    def _tick(self):
        if self.last is None or time.monotonic() - self.last_rx > self.timeout:
            gear, lin, ang = stop_command(self.gear)
        else:
            gear, lin, ang = twist_to_ctrl(self.last.linear.x, self.last.angular.z,
                                           self.gear, self.allow_reverse)
        out = CtrlCmd()
        out.ctrl_cmd_gear = gear
        out.ctrl_cmd_linear = float(lin)
        out.ctrl_cmd_angular = float(ang)
        self.pub.publish(out)

    def publish_stop(self):
        gear, lin, ang = stop_command(self.gear)
        out = CtrlCmd(ctrl_cmd_gear=gear, ctrl_cmd_linear=lin, ctrl_cmd_angular=ang)
        for _ in range(3):
            self.pub.publish(out)


def main():
    rclpy.init()
    node = YhsCmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.try_shutdown()
