#!/usr/bin/env python3
"""Persistent ROS service client. Example commands move the gripper."""
import time
import rclpy
from mini_handler_ros_connector.srv import Command, GetResult


class GripperClient:
    def __init__(self, node, namespace='/mini_handler'):
        self.node = node
        self.command = node.create_client(Command, namespace+'/command')
        self.result = node.create_client(GetResult, namespace+'/get_result')
        for client in (self.command, self.result):
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('gripper service unavailable')

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.)
        if not future.done():
            raise RuntimeError('response missing; inspect state before retrying movement')
        return future.result()

    def move(self, operation, value=0., speed=.5, acceleration=.5):
        response = self.call(self.command, Command.Request(operation=operation, value=value,
            speed_scale=speed, acceleration_scale=acceleration, timeout_s=12.))
        if not response.accepted:
            raise RuntimeError(response.message)
        return response.command_id

    def wait(self, command_id):
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            response = self.call(self.result, GetResult.Request(command_id=command_id))
            if not response.found:
                raise RuntimeError('command absent: connector restarted or history expired')
            if response.done:
                return response.result
            time.sleep(.02)
        raise RuntimeError('result timeout; inspect status before issuing another command')


if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('my_gripper_client')
    try:
        gripper = GripperClient(node)
        # Connect once and reuse gripper.move()/wait() in your application.
        result = gripper.wait(gripper.move('opening', .8))
        print(result)
    finally:
        node.destroy_node()
        rclpy.shutdown()
