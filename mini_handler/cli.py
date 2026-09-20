"""Convenience CLI. Persistent ROS service clients have lower discovery latency."""
import argparse
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from std_srvs.srv import Trigger
from mini_handler_ros_connector.msg import State
from mini_handler_ros_connector.srv import Command, GetResult


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['status', 'recover', 'open', 'close', 'stop', 'opening',
        'relative', 'width', 'relative_mm', 'grip_torque', 'grip_force',
        'calibrate_open', 'calibrate_close'])
    parser.add_argument('value', type=float, nargs='?', default=0.)
    parser.add_argument('--speed', type=float, default=0., help='scale 0..1; zero uses default')
    parser.add_argument('--accel', type=float, default=0.)
    parser.add_argument('--torque', type=float, default=0.)
    parser.add_argument('--timeout', type=float, default=12., help='motion timeout in seconds')
    parser.add_argument('--namespace', default='/mini_handler')
    parser.add_argument('--no-wait', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('mini_handler_cli')
    prefix = args.namespace.rstrip('/')

    def call(kind, name, request):
        client = node.create_client(kind, prefix+'/'+name)
        try:
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError(f'{name} unavailable; check container/domain/namespace')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            if not future.done():
                raise RuntimeError('service timeout: command outcome may be unknown')
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        if args.operation == 'status':
            received = []
            sub = node.create_subscription(State, prefix+'/state', received.append, qos_profile_sensor_data)
            deadline = time.monotonic()+4
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if not received:
                raise RuntimeError('no state received')
            print(json.dumps(message_to_ordereddict(received[-1]), indent=2))
            return 0
        if args.operation == 'recover':
            response = call(Trigger, 'recover', Trigger.Request())
            print(json.dumps({'success': response.success, 'message': response.message}))
            return 0 if response.success else 2
        if args.operation == 'stop':
            response = call(Trigger, 'stop', Trigger.Request())
            if not response.success:
                raise RuntimeError(response.message)
            identifier = response.message
        else:
            response = call(Command, 'command', Command.Request(operation=args.operation, value=args.value,
                speed_scale=args.speed, acceleration_scale=args.accel, torque_limit_nm=args.torque,
                timeout_s=args.timeout))
            if not response.accepted:
                raise RuntimeError(response.message)
            identifier = response.command_id
        print(json.dumps({'accepted': True, 'command_id': identifier}), flush=True)
        if args.no_wait:
            return 0
        deadline = time.monotonic()+args.timeout+5
        client = node.create_client(GetResult, prefix+'/get_result')
        client.wait_for_service(timeout_sec=3.)
        while time.monotonic() < deadline:
            future = client.call_async(GetResult.Request(command_id=identifier))
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
            if future.done() and future.result().done:
                result = future.result().result
                print(json.dumps(message_to_ordereddict(result), indent=2))
                return 0 if result.success else 2
            time.sleep(.02)
        raise RuntimeError('completion unknown; inspect state/get_result before retrying')
    except RuntimeError as exc:
        print(json.dumps({'error': str(exc)}))
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
