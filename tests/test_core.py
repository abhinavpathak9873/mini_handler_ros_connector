import math
import time
from dataclasses import replace
import pytest
from mini_handler.controller import Config, Controller
from mini_handler.simulator import SimulatedMotor


def wait_for(predicate, timeout=3):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.005)
    raise AssertionError('timed out')


@pytest.fixture
def rig():
    c = Config(poll_hz=100., settle_time_s=.025, simulated_contact_fraction=-1.)
    motor = SimulatedMotor(c)
    driver = Controller(c, motor)
    wait_for(lambda: driver.connected)
    yield driver, motor
    driver.close()


def result(driver, identifier):
    return wait_for(lambda: driver.get_result(identifier)[2], 4)


def test_full_cycle_and_relative_opening(rig):
    d, m = rig
    close = result(d, d.submit('close'))
    assert close['outcome'] == 'closed'
    assert close['at_closed_reference']
    assert close['opening_fraction'] == pytest.approx(0, abs=.004)
    partial = result(d, d.submit('relative', .5))
    assert partial['opening_fraction'] == pytest.approx(.5, abs=.005)
    opened = result(d, d.submit('open'))
    assert opened['opening_fraction'] == pytest.approx(1, abs=.004)
    assert opened['accepted_to_send_ms'] < 200


@pytest.mark.parametrize('operation,value', [('width', 80), ('relative_mm', -5),
    ('grip_force', 100), ('opening', 1.1), ('relative', .5), ('grip_torque', 6),
    ('grip_torque', 0), ('opening', math.nan), ('bogus', 0)])
def test_invalid_commands_never_write(rig, operation, value):
    d, m = rig
    with pytest.raises(ValueError):
        d.submit(operation, value)
    assert not m.commands


@pytest.mark.parametrize('kwargs', [dict(speed_scale=2), dict(acceleration_scale=-1),
    dict(torque_limit_nm=8), dict(timeout_s=-1), dict(speed_scale=math.inf)])
def test_invalid_limits(rig, kwargs):
    d, m = rig
    with pytest.raises(ValueError):
        d.submit('close', **kwargs)
    assert not m.commands


def test_busy_rejected_stop_interrupts_and_restores_ownership(rig):
    d, m = rig
    identifier = d.submit('close', speed_scale=.1)
    with pytest.raises(ValueError, match='busy'):
        d.submit('open')
    wait_for(lambda: len(m.commands) > 0)
    time.sleep(.04)
    stop = d.stop()
    assert result(d, identifier)['outcome'] == 'cancelled'
    assert result(d, stop)['success']
    assert abs(d.sample.velocity) < d.config.stationary_velocity_rps
    assert result(d, d.submit('open'))['success']


def test_contact_and_distance_obstruction_are_distinct(rig):
    d, m = rig
    m.c = replace(m.c, simulated_contact_fraction=.5)
    grip = result(d, d.submit('grip_torque', 2.))
    assert grip['outcome'] == 'contact'
    assert grip['opening_fraction'] == pytest.approx(.5)
    assert grip['torque_nm'] == pytest.approx(2)
    assert result(d, d.submit('open'))['success']
    blocked = result(d, d.submit('opening', .1))
    assert blocked['outcome'] == 'blocked'
    assert not blocked['success']


def test_empty_force_target_does_not_claim_contact(rig):
    d, m = rig
    assert result(d, d.submit('grip_torque', 2.))['outcome'] == 'blocked'


def test_measured_calibration_enables_mm_and_newtons(rig):
    d, m = rig
    d.config = replace(d.config, width_calibrated=True, force_n_per_nm=50.)
    assert result(d, d.submit('width', 90.))['success']
    assert d.config.width(d.sample.position) == pytest.approx(90, abs=.2)
    m.c = replace(m.c, simulated_contact_fraction=.35)
    value = result(d, d.submit('grip_force', 100.))
    assert value['outcome'] == 'contact'
    assert value['torque_nm'] == pytest.approx(2.)


def test_timeout_holds_and_reports_failure(rig):
    d, m = rig
    value = result(d, d.submit('close', speed_scale=.05, timeout_s=.05))
    assert value['outcome'] == 'timeout'
    assert abs(d.sample.velocity) < .01


def test_cable_loss_latches_and_never_replays(rig):
    d, m = rig
    identifier = d.submit('close', speed_scale=.1)
    wait_for(lambda: len(m.commands) > 0)
    m.disconnected = True
    value = result(d, identifier)
    assert value['outcome'] == 'communication_error'
    assert 'HOLD UNCONFIRMED' in value['detail']
    count = len(m.commands)
    m.disconnected = False
    time.sleep(.05)
    with pytest.raises(ValueError):
        d.submit('open')
    assert len(m.commands) == count


def test_motor_fault_never_resets_or_moves(rig):
    d, m = rig
    m.fault = 38
    wait_for(lambda: d.error)
    with pytest.raises(ValueError):
        d.submit('close')
    assert not m.commands
    assert d.sample.fault == 38


def test_shutdown_cancels_and_holds(rig):
    d, m = rig
    identifier = d.submit('close', speed_scale=.1)
    wait_for(lambda: len(m.commands) > 0)
    d.close()
    assert result(d, identifier)['outcome'] == 'cancelled'
    assert not d.thread.is_alive()


@pytest.mark.parametrize('kwargs', [dict(open_position_rev=.328063965), dict(max_torque_nm=1),
    dict(poll_hz=0), dict(force_n_per_nm=-1), dict(default_speed_scale=2),
    dict(max_speed_rps=math.nan), dict(position_tolerance_rev=.4)])
def test_bad_config_fails(kwargs):
    with pytest.raises(ValueError):
        Config(**kwargs)


def test_reverse_encoder_mapping():
    c = Config(open_position_rev=1., close_position_rev=-1.)
    assert c.opening(1.) == 1
    assert c.position(.25) == -.5
    assert c.width(-1.) == 55.


@pytest.mark.parametrize('start,operation', [(-.3, 'open'), (.5, 'close')])
def test_feedback_outside_saved_span_stays_ready_and_reenters(start, operation):
    c = Config(poll_hz=100., settle_time_s=.025, simulated_contact_fraction=-1.)
    m = SimulatedMotor(c)
    m.position = m.target = start
    d = Controller(c, m)
    try:
        wait_for(lambda: d.connected)
        assert not d.error
        value = result(d, d.submit(operation))
        assert value['success']
        low, high = sorted((c.open_position_rev, c.close_position_rev))
        assert low-c.position_tolerance_rev <= d.sample.position <= high+c.position_tolerance_rev
    finally:
        d.close()


def test_read_only_session_shutdown_does_not_enable_idle_motor():
    from mini_handler.protocol import Sample
    class Idle:
        rtt_ms = 0.
        commands = []
        def exchange(self, command=b''):
            if command:
                self.commands.append(command)
            return Sample(-.1597, 0., 0., 0, 0, 24., 30., time.monotonic())
        def close(self):
            pass
    motor = Idle()
    d = Controller(Config(), motor)
    wait_for(lambda: d.connected)
    d.close()
    assert motor.commands == []


def test_cancel_replaces_sent_target_even_when_velocity_is_still_zero(rig):
    d, m = rig
    identifier = d.submit('close', speed_scale=.001)
    wait_for(lambda: len(m.commands) > 0)
    stop = d.stop()
    assert result(d, identifier)['outcome'] == 'cancelled'
    assert result(d, stop)['success']
    assert len(m.commands) >= 2
    assert m.target != d.config.close_position_rev
    assert abs(m.target-m.position) <= d.config.position_tolerance_rev


def test_opening_unloads_existing_clamp_without_false_blocked_result(rig):
    d, m = rig
    m.c = replace(m.c, simulated_contact_fraction=.5)
    assert result(d, d.submit('close'))['outcome'] == 'contact'
    original = m.exchange
    release_at = time.monotonic()+.4
    def delayed_release(command=b''):
        sample = original(command)
        if time.monotonic() < release_at:
            m.position = m.c.position(.5)
            m.velocity = 0.
            sample.position, sample.velocity, sample.torque = m.position, 0., 3.0
        return sample
    m.exchange = delayed_release
    assert result(d, d.submit('open'))['outcome'] == 'reached'


def test_object_contact_does_not_change_empty_reference(rig):
    d, m = rig
    reference = d.config.close_position_rev
    m.c = replace(m.c, simulated_contact_fraction=.6)
    r = result(d, d.submit('close'))
    assert r['success'] and r['outcome'] == 'contact'
    assert not r['at_closed_reference']
    assert r['opening_fraction'] == pytest.approx(.6)
    assert d.config.close_position_rev == reference


def test_guarded_calibration_probes_both_empty_stops(rig):
    d, m = rig
    with pytest.raises(ValueError, match='explicit --torque'):
        d.submit('calibrate_open')
    closed = result(d, d.submit('calibrate_close', torque_limit_nm=2.,
                                speed_scale=.25, acceleration_scale=.25))
    assert closed['outcome'] == 'contact'
    assert closed['position_rev'] == pytest.approx(d.config.close_position_rev, abs=.002)
    assert m.target == pytest.approx(m.position, abs=d.config.position_tolerance_rev)
    opened = result(d, d.submit('calibrate_open', torque_limit_nm=2.,
                                speed_scale=.25, acceleration_scale=.25))
    assert opened['outcome'] == 'contact'
    assert opened['position_rev'] == pytest.approx(d.config.open_position_rev, abs=.002)
    assert m.target == pytest.approx(m.position, abs=d.config.position_tolerance_rev)
    with pytest.raises(ValueError, match='calibration session active'):
        d.submit('close')


def test_calibration_speed_is_capped(rig):
    d, _ = rig
    with pytest.raises(ValueError, match='cannot exceed'):
        d.submit('calibrate_close', torque_limit_nm=2., speed_scale=.5)


def test_absent_start_and_idle_hotplug_reconnect_are_read_only_and_automatic():
    c = Config(reconnect_interval_s=.02, poll_hz=100.)
    m = SimulatedMotor(c)
    available = False
    def factory():
        if not available:
            raise OSError('adapter missing')
        return m
    d = Controller(c, transport_factory=factory)
    try:
        wait_for(lambda: d.error)
        assert not d.connected and not d.ever_connected
        with pytest.raises(ValueError):
            d.submit('close')
        available = True
        wait_for(lambda: d.connected and not d.error)
        assert not m.commands
        m.disconnected = True
        wait_for(lambda: not d.connected)
        assert not d.fault_latched
        assert not d.connected
        m.disconnected = False
        wait_for(lambda: d.connected and not d.error)
        assert not d.fault_latched
        assert not m.commands
        assert result(d, d.submit('open'))['success']
    finally:
        d.close()


def test_reconnect_preserves_failed_result_and_never_replays():
    c = Config(reconnect_interval_s=.02, poll_hz=100.)
    m = SimulatedMotor(c)
    d = Controller(c, transport_factory=lambda: m)
    try:
        wait_for(lambda: d.connected)
        ident = d.submit('close', speed_scale=.01)
        wait_for(lambda: m.commands)
        m.disconnected = True
        assert result(d, ident)['outcome'] == 'communication_error'
        count = len(m.commands)
        m.disconnected = False
        wait_for(lambda: d.connected)
        assert d.fault_latched and len(m.commands) == count
        assert d.get_result(ident)[2]['outcome'] == 'communication_error'
    finally:
        d.close()


def test_recover_rejects_fault_and_moving_motor(rig):
    d, m = rig
    with d.lock:
        d.sample.fault = 38
        with pytest.raises(RuntimeError):
            d.recover()
        d.sample.fault = 0
        d.sample.velocity = 1.
        with pytest.raises(ValueError):
            d.recover()
        d.sample.velocity = 0.
