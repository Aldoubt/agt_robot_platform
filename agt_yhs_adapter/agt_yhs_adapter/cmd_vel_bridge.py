"""Verified SI Twist endpoint to YHS CtrlCmd with independent permission gates."""
import math
import time

import rclpy
from agt_robot_interfaces.msg import BaseState
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from yhs_can_interfaces.msg import CtrlCmd

from .conversion import ConversionError, stop_command, twist_to_ctrl, validate_gear


class YhsCmdVelBridge(Node):
    def __init__(self):
        super().__init__('agt_yhs_cmd_vel_bridge')
        defaults = {'field_verified':False, 'robot_profile':'',
                    'input_topic':'/agt/yhs/cmd_vel', 'output_topic':'/yhs/ctrl_cmd',
                    'base_state_topic':'/agt/base/state', 'publish_rate_hz':50.0,
                    'command_timeout_sec':0.15, 'state_timeout_sec':0.30,
                    'drive_gear':-1, 'angular_semantics':'', 'angular_unit':'', 'linear_unit':'',
                    'allow_reverse':False, 'reverse_semantics':'',
                    'manual_priority_verified':False, 'hardware_watchdog_verified':False,
                    'max_forward_mps':0.0, 'max_reverse_mps':0.0, 'max_angular_rps':0.0}
        for name,value in defaults.items(): self.declare_parameter(name,value)
        get = lambda name:self.get_parameter(name).value
        self.verified = bool(get('field_verified'))
        self.robot_profile = str(get('robot_profile'))
        self.gear = int(get('drive_gear'))
        self.angular_unit = str(get('angular_unit'))
        self.allow_reverse = bool(get('allow_reverse'))
        self.forward = float(get('max_forward_mps'))
        self.reverse = float(get('max_reverse_mps'))
        self.angular = float(get('max_angular_rps'))
        self.timeout = float(get('command_timeout_sec'))
        self.state_timeout = float(get('state_timeout_sec'))
        rate = float(get('publish_rate_hz'))
        if rate != 50.0 or not all(math.isfinite(v) and v > 0 for v in (self.timeout,self.state_timeout)):
            raise ConversionError('verified command chain requires 50 Hz and positive finite watchdogs')
        if self.verified:
            validate_gear(self.gear)
            if (not self.robot_profile or get('angular_semantics') != 'yaw_rate'
                    or self.angular_unit not in ('deg_per_sec','rad_per_sec')
                    or get('linear_unit') != 'm_per_sec'
                    or not get('manual_priority_verified') or not get('hardware_watchdog_verified')):
                raise ConversionError('verified YHS yaw-rate protocol, units, manual priority and watchdog required')
            if not all(math.isfinite(v) for v in (self.forward,self.reverse,self.angular)) or self.forward <= 0 or self.angular <= 0 or self.reverse < 0:
                raise ConversionError('measured motion caps are required')
            if self.allow_reverse and (get('reverse_semantics') != 'signed_speed_same_gear' or self.reverse <= 0):
                raise ConversionError('reverse requires measured signed_speed_same_gear protocol and speed cap')
        self.last = self.state = None
        self.last_rx = self.state_rx = 0.0
        self.pub = self.create_publisher(CtrlCmd,str(get('output_topic')),10) if self.verified else None
        self.create_subscription(Twist,str(get('input_topic')),self._on_cmd,20)
        self.create_subscription(BaseState,str(get('base_state_topic')),self._on_state,10)
        self.create_timer(1.0/rate,self._tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'YHS protocol bridge verified={self.verified}; profile={self.robot_profile}')

    def _on_state(self,msg):
        self.state,self.state_rx = msg,time.monotonic()
        if not self._permitted(): self.last = None

    def _permitted(self):
        state = self.state
        now = self.get_clock().now().nanoseconds
        source = int(state.stamp.sec)*1_000_000_000 + int(state.stamp.nanosec) if state else 0
        return bool(state is not None and state.robot_profile == self.robot_profile
                    and state.source_valid and state.drive_permitted
                    and not state.emergency_stop and not state.manual_override and not state.fault_active
                    and 0 <= (now-source)/1e9 <= self.state_timeout
                    and 0 <= time.monotonic()-self.state_rx <= self.state_timeout)

    def _on_cmd(self,msg):
        values = (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if (not self._permitted() or not all(math.isfinite(v) for v in values)
                or any(abs(v)>1e-9 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y))):
            self.last = None
            return
        self.last = (max(-self.reverse if self.allow_reverse else 0.0,min(self.forward,msg.linear.x)),
                     max(-self.angular,min(self.angular,msg.angular.z)))
        self.last_rx = time.monotonic()

    def _tick(self):
        if self.pub is None: return
        if not self._permitted() or self.last is None or not 0 <= time.monotonic()-self.last_rx <= self.timeout:
            gear,linear,angular = stop_command(self.gear)
        else:
            gear,linear,angular = twist_to_ctrl(*self.last,self.gear,self.allow_reverse,self.angular_unit)
        self.pub.publish(CtrlCmd(ctrl_cmd_gear=gear,ctrl_cmd_linear=float(linear),ctrl_cmd_angular=float(angular)))

    def publish_stop(self):
        if self.pub is None: return
        gear,linear,angular = stop_command(self.gear)
        for _ in range(3):
            self.pub.publish(CtrlCmd(ctrl_cmd_gear=gear,ctrl_cmd_linear=float(linear),ctrl_cmd_angular=float(angular)))


def main():
    rclpy.init()
    node = None
    try:
        node = YhsCmdVelBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()
        rclpy.try_shutdown()
