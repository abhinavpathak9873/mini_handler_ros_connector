"""ROS interface; callbacks never wait for a physical movement."""
import math
import signal
from dataclasses import fields

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from mini_handler_ros_connector.msg import State, Result
from mini_handler_ros_connector.srv import Command, GetResult

from .controller import Config, Controller
from .protocol import SerialMotor
from .simulator import SimulatedMotor


class Connector(Node):
    def __init__(self):
        super().__init__('mini_handler', namespace='mini_handler')
        defaults = Config()
        for f in fields(defaults):
            self.declare_parameter(f.name, getattr(defaults, f.name),
                                   ParameterDescriptor(read_only=True))
        self.c = Config(**{f.name: self.get_parameter(f.name).value for f in fields(defaults)})
        self.state_pub = self.create_publisher(State, 'state', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.result_pub = self.create_publisher(Result, 'results', 32)
        transport = (SimulatedMotor(self.c) if self.c.simulate else SerialMotor(
            self.c.port, self.c.motor_id, self.c.request_timeout_s, self.c.extended_telemetry))
        self.controller = Controller(self.c, transport, self.publish_result)
        self.create_service(Command, 'command', self.command)
        self.create_service(GetResult, 'get_result', self.get_result)
        for op in ('open', 'close', 'stop'):
            self.create_service(Trigger, op, lambda req, res, operation=op: self.trigger(operation, res))
        self.create_timer(1/self.c.poll_hz, self.publish_state)
        self.get_logger().info('SIMULATOR' if self.c.simulate else f'Serial owner: {self.c.port}')

    def command(self, req, res):
        try:
            res.command_id = self.controller.submit(req.operation, req.value, req.speed_scale,
                req.acceleration_scale, req.torque_limit_nm, req.timeout_s)
            res.accepted, res.message = True, 'accepted; poll get_result or subscribe to results'
        except (ValueError, RuntimeError) as exc:
            res.accepted, res.message = False, str(exc)
        return res

    def trigger(self, operation, res):
        try:
            identifier = self.controller.stop() if operation == 'stop' else self.controller.submit(operation)
            res.success, res.message = True, identifier
        except (ValueError, RuntimeError) as exc:
            res.success, res.message = False, str(exc)
        return res

    def result_message(self, value):
        result = Result()
        result.header.stamp = self.get_clock().now().to_msg()
        for key, val in value.items():
            setattr(result, key, val)
        return result

    def publish_result(self, value):
        if rclpy.ok():
            self.result_pub.publish(self.result_message(value))

    def get_result(self, req, res):
        res.found, res.done, value = self.controller.get_result(req.command_id)
        if value:
            res.result = self.result_message(value)
        return res

    def publish_state(self):
        import time
        s, connected, phase, error, plan = self.controller.snapshot()
        msg = State()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.simulated = self.c.simulate
        msg.connected, msg.busy = connected, plan is not None or self.controller.stop_pending is not None
        if s and abs(s.velocity) >= self.c.stationary_velocity_rps:
            msg.busy = True
        msg.phase, msg.detail = phase, error
        msg.command_id = plan.command_id if plan else ''
        msg.sample_age_s = time.monotonic()-s.received_at if s else math.inf
        msg.width_calibrated = self.c.width_calibrated
        msg.force_calibrated = self.c.force_n_per_nm > 0
        msg.last_transport_rtt_ms = self.controller.transport.rtt_ms
        for name in ('position_rev', 'velocity_rps', 'torque_nm', 'opening_fraction',
                     'width_mm', 'estimated_force_n', 'voltage_v', 'temperature_c'):
            setattr(msg, name, math.nan)
        if s:
            msg.position_rev, msg.velocity_rps, msg.torque_nm = s.position, s.velocity, s.torque
            msg.mode, msg.fault = s.mode, s.fault
            msg.opening_fraction, msg.width_mm = self.c.opening(s.position), self.c.width(s.position)
            msg.estimated_force_n = abs(s.torque)*self.c.force_n_per_nm if msg.force_calibrated else math.nan
            msg.voltage_v, msg.temperature_c = s.voltage, s.temperature
            if connected:
                joints = JointState()
                joints.header = msg.header
                joints.name = ['mini_handler_motor']
                joints.position, joints.velocity, joints.effort = [s.position*math.tau], [s.velocity*math.tau], [s.torque]
                self.joint_pub.publish(joints)
        self.state_pub.publish(msg)


def main():
    # Finish the measured hold before shutting down the ROS context. Run this
    # executable directly as container PID 1 (under Docker --init).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node = None
    try:
        node = Connector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.controller.close()
            if node.controller.error:
                print(node.controller.error, flush=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
