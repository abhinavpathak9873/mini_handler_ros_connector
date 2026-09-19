"""Read live telemetry; --cycle explicitly commands one close then open cycle."""
import argparse
import json
import math
import statistics
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from mini_handler_ros_connector.msg import State
from mini_handler_ros_connector.srv import Command, GetResult


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cycle', action='store_true')
    args = p.parse_args()
    rclpy.init()
    node = rclpy.create_node('mini_handler_hardware_check')
    samples, times = [], []
    def received(s):
        samples.append(s)
        times.append(time.monotonic())
    node.create_subscription(State, '/mini_handler/state', received, qos_profile_sensor_data)
    command = node.create_client(Command, '/mini_handler/command')
    results = node.create_client(GetResult, '/mini_handler/get_result')
    def call(client, request):
        f = client.call_async(request)
        rclpy.spin_until_future_complete(node, f, timeout_sec=2.)
        if not f.done():
            raise RuntimeError('service response missing: inspect state; do not retry movement')
        return f.result()
    try:
        assert command.wait_for_service(timeout_sec=5.)
        until = time.monotonic()+4
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        assert len(samples) > 20, 'insufficient telemetry'
        s = samples[-1]
        assert s.connected and not s.simulated and not s.busy and s.fault == 0
        assert math.isfinite(s.position_rev) and math.isfinite(s.torque_nm)
        print('TELEMETRY', json.dumps(message_to_ordereddict(s)), flush=True)
        print('TELEMETRY_BENCHMARK', json.dumps({
            'frames': len(samples), 'hz': (len(times)-1)/(times[-1]-times[0]),
            'serial_rtt_median_ms': statistics.median(x.last_transport_rtt_ms for x in samples),
            'serial_rtt_max_ms': max(x.last_transport_rtt_ms for x in samples)}), flush=True)
        if args.cycle:
            for operation in ('close', 'open'):
                begin = time.monotonic()
                accepted = call(command, Command.Request(operation=operation, timeout_s=12.))
                latency = (time.monotonic()-begin)*1000
                assert accepted.accepted, accepted.message
                print('ACCEPTED', operation, accepted.command_id, 'rtt_ms', latency, flush=True)
                deadline = time.monotonic()+16
                outcome = None
                while time.monotonic() < deadline:
                    response = call(results, GetResult.Request(command_id=accepted.command_id))
                    if response.done:
                        outcome = response.result
                        break
                    time.sleep(.02)
                assert outcome is not None, 'completion missing; no automatic recovery movement'
                print('RESULT', json.dumps(message_to_ordereddict(outcome)), flush=True)
                assert outcome.success, outcome.detail
                if operation == 'open':
                    assert outcome.outcome == 'reached'
            print('HARDWARE_CYCLE_PASS', flush=True)
        print('HARDWARE_TELEMETRY_PASS', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
