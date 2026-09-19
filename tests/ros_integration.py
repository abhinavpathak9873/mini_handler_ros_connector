"""End-to-end real ROS services, messages, cancellation and telemetry in simulation."""
import os
import signal
import subprocess
import time
import statistics

os.environ['ROS_DOMAIN_ID'] = '178'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from rclpy.qos import qos_profile_sensor_data
from mini_handler_ros_connector.msg import State, Result
from mini_handler_ros_connector.srv import Command, GetResult
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_prefix


def main():
    executable = get_package_prefix('mini_handler_ros_connector')+'/lib/mini_handler_ros_connector/connector'
    proc = subprocess.Popen([executable,
        '--ros-args', '-p', 'simulate:=true', '-p', 'settle_time_s:=0.04'])
    rclpy.init()
    node = rclpy.create_node('mini_handler_integration_test')
    states, events = [], []
    node.create_subscription(State, '/mini_handler/state', states.append, qos_profile_sensor_data)
    node.create_subscription(Result, '/mini_handler/results', events.append, 32)
    command = node.create_client(Command, '/mini_handler/command')
    stop = node.create_client(Trigger, '/mini_handler/stop')
    results = node.create_client(GetResult, '/mini_handler/get_result')

    def call(client, request):
        f = client.call_async(request)
        rclpy.spin_until_future_complete(node, f, timeout_sec=3.)
        assert f.done(), 'service timeout'
        return f.result()

    def finish(identifier):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            r = call(results, GetResult.Request(command_id=identifier))
            assert r.found
            if r.done:
                return r.result
            time.sleep(.01)
        raise AssertionError('movement did not complete')

    try:
        assert command.wait_for_service(timeout_sec=15.)
        deadline = time.monotonic()+5
        while (not states or not states[-1].connected) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert states[-1].simulated and states[-1].connected
        assert not states[-1].width_calibrated and not states[-1].force_calibrated
        assert not call(command, Command.Request(operation='grip_force', value=100.)).accepted
        r = call(command, Command.Request(operation='opening', value=.8))
        assert r.accepted
        assert not call(command, Command.Request(operation='close')).accepted
        assert finish(r.command_id).outcome == 'reached'
        r = call(command, Command.Request(operation='grip_torque', value=2.))
        assert finish(r.command_id).outcome == 'contact'
        r = call(command, Command.Request(operation='open', speed_scale=.1))
        s = call(stop, Trigger.Request())
        assert s.success
        assert finish(r.command_id).outcome == 'cancelled'
        assert finish(s.message).success
        assert finish(call(command, Command.Request(operation='open')).command_id).success
        # Warm persistent-client latency, not per-invocation CLI startup.
        timings, send_times = [], []
        for _ in range(20):
            start = time.monotonic()
            r = call(command, Command.Request(operation='open'))
            timings.append((time.monotonic()-start)*1000)
            send_times.append(finish(r.command_id).accepted_to_send_ms)
        print('ROS_SIM_BENCHMARK', {'samples': len(timings),
            'service_median_ms': statistics.median(timings),
            'service_p95_ms': sorted(timings)[18],
            'accepted_to_send_p95_ms': sorted(send_times)[18]}, flush=True)
        assert len(events) >= 20
        assert len(states) >= 20
        print('ROS_INTEGRATION_PASS', flush=True)
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=8)
        assert proc.returncode == 0, proc.returncode
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
