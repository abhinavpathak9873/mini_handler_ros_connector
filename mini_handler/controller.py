"""Single serial worker, immediate command acceptance, bounded completion."""
import math
import termios
import threading
import time
import uuid
from collections import OrderedDict
from dataclasses import dataclass, fields

from .protocol import target


@dataclass(frozen=True)
class Config:
    port: str = 'auto'
    motor_id: int = 1
    simulate: bool = False
    poll_hz: float = 50.0
    request_timeout_s: float = .1
    open_position_rev: float = -.295181274
    close_position_rev: float = .328063965
    # Maximum distance beyond either saved endpoint used only by the guarded
    # empty-jaw calibration operations.
    calibration_probe_margin_rev: float = .25
    reconnect_interval_s: float = 1.0
    min_width_mm: float = 55.0
    max_width_mm: float = 105.0
    width_calibrated: bool = False
    force_n_per_nm: float = 0.0
    max_speed_rps: float = 1.0
    max_acceleration_rps2: float = 1.0
    max_torque_nm: float = 4.0
    open_torque_nm: float = 3.07
    close_torque_nm: float = 4.0
    default_speed_scale: float = 1.0
    default_acceleration_scale: float = 1.0
    command_timeout_s: float = 12.0
    settle_time_s: float = .15
    position_tolerance_rev: float = .002
    stationary_velocity_rps: float = .01
    contact_ratio: float = .95
    extended_telemetry: bool = True
    simulated_contact_fraction: float = .35

    def __post_init__(self):
        for f in fields(self):
            v = getattr(self, f.name)
            if isinstance(v, (float, int)) and not math.isfinite(v):
                raise ValueError(f'{f.name} must be finite')
        if not self.port or not 1 <= self.motor_id <= 127:
            raise ValueError('invalid serial port or motor id')
        if not 1 <= self.poll_hz <= 200 or not .01 <= self.request_timeout_s <= 1:
            raise ValueError('poll_hz must be 1..200; request timeout .01..1 s')
        if (self.open_position_rev == self.close_position_rev
                or max(abs(self.open_position_rev), abs(self.close_position_rev)) > 1000
                or not 0 < self.calibration_probe_margin_rev <= 1
                or not 0 <= self.min_width_mm < self.max_width_mm):
            raise ValueError('invalid endpoint calibration')
        for name in ('max_speed_rps', 'max_acceleration_rps2', 'max_torque_nm',
                     'open_torque_nm', 'close_torque_nm', 'command_timeout_s',
                     'settle_time_s', 'position_tolerance_rev', 'stationary_velocity_rps',
                     'reconnect_interval_s'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if (max(self.open_torque_nm, self.close_torque_nm) > self.max_torque_nm
                or self.max_torque_nm > 327.67 or self.force_n_per_nm < 0
                or self.position_tolerance_rev >= abs(self.close_position_rev-self.open_position_rev)/2):
            raise ValueError('invalid effort or position tolerance')
        for name in ('default_speed_scale', 'default_acceleration_scale', 'contact_ratio'):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f'{name} must be in (0,1]')
        if not -1 <= self.simulated_contact_fraction <= 1:
            raise ValueError('invalid simulator contact')

    def opening(self, position):
        return (position - self.close_position_rev) / (self.open_position_rev - self.close_position_rev)

    def position(self, opening):
        return self.close_position_rev + opening * (self.open_position_rev-self.close_position_rev)

    def width(self, position):
        return self.min_width_mm + self.opening(position)*(self.max_width_mm-self.min_width_mm)


@dataclass
class Plan:
    command_id: str
    operation: str
    position: float
    speed: float
    acceleration: float
    torque: float
    timeout: float
    accepted_at: float
    sent_at: float = 0.0
    settled_at: float = 0.0
    condition: str = ''
    closing: bool = False
    movement_sign: float = 0.0
    motion_observed: bool = False


class Controller:
    def __init__(self, config, transport=None, on_result=lambda result: None, transport_factory=None):
        self.config, self.transport, self.on_result = config, transport, on_result
        self.transport_factory = transport_factory
        self.ever_connected = False
        self.fault_latched = False
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.shutting_down = False
        self.sample = None
        self.connected = False
        self.error = ''
        self.phase = 'connecting'
        self.active = None
        self.stop_pending = None
        self.calibration_session = False
        self.results = OrderedDict()
        self.thread = threading.Thread(target=self._run, name='mini-handler-serial', daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.sample, self.connected, self.phase, self.error, self.active

    def _fresh(self):
        if self.error or not self.connected or self.sample is None:
            raise ValueError(self.error or 'waiting for fresh hardware feedback')
        if time.monotonic()-self.sample.received_at > max(.2, 3/self.config.poll_hz):
            raise ValueError('feedback stale')
        self._check(self.sample)

    def recover(self):
        """Acknowledge a recovered link/fault without sending a motor command."""
        with self.lock:
            if not self.connected or self.sample is None:
                raise ValueError('not connected')
            if time.monotonic()-self.sample.received_at > max(.2, 3/self.config.poll_hz):
                raise ValueError('feedback stale')
            self._check(self.sample)
            if self.active or self.stop_pending or abs(self.sample.velocity) >= self.config.stationary_velocity_rps:
                raise ValueError('motor must be stationary with no pending command')
            self.fault_latched = False
            self.error, self.phase = '', 'idle'

    def at_closed_reference(self, sample):
        return bool(sample and abs(sample.position-self.config.close_position_rev) <= self.config.position_tolerance_rev
                    and abs(sample.velocity) < self.config.stationary_velocity_rps)

    def submit(self, operation, value=0., speed_scale=0., acceleration_scale=0.,
               torque_limit_nm=0., timeout_s=0.):
        with self.lock:
            calibrating = operation in ('calibrate_open', 'calibrate_close')
            self._fresh()
            if self.calibration_session and not calibrating:
                raise ValueError('calibration session active; save endpoints and restart before normal motion')
            if self.shutting_down:
                raise ValueError('shutting down')
            if self.active or self.stop_pending:
                raise ValueError('busy; wait for completion or call stop')
            if abs(self.sample.velocity) >= self.config.stationary_velocity_rps:
                raise ValueError('motor already moving; call stop before taking ownership')
            values = (value, speed_scale, acceleration_scale, torque_limit_nm, timeout_s)
            if not all(math.isfinite(v) for v in values) or min(values[1:]) < 0:
                raise ValueError('parameters must be finite; limits cannot be negative')
            c = self.config
            current = c.opening(self.sample.position)
            torque = torque_limit_nm
            direction = math.copysign(1., c.close_position_rev-c.open_position_rev)
            if operation == 'calibrate_open':
                position = c.open_position_rev-direction*c.calibration_probe_margin_rev
                opening = c.opening(position)
            elif operation == 'calibrate_close':
                position = c.close_position_rev+direction*c.calibration_probe_margin_rev
                opening = c.opening(position)
            elif operation == 'open':
                opening = 1.
            elif operation in ('close', 'grip_torque', 'grip_force'):
                opening = 0.
            elif operation == 'opening':
                opening = value
            elif operation == 'relative':
                opening = current + value
            elif operation in ('width', 'relative_mm'):
                if not c.width_calibrated:
                    raise ValueError('millimetre commands require width_calibrated=true and measured endpoints')
                opening = ((value-c.min_width_mm)/(c.max_width_mm-c.min_width_mm)
                           if operation == 'width' else current+value/(c.max_width_mm-c.min_width_mm))
            else:
                raise ValueError('unknown operation')
            if not calibrating and not 0 <= opening <= 1:
                raise ValueError('requested opening outside configured travel [0,1]')
            if calibrating and torque_limit_nm <= 0:
                raise ValueError('calibration requires an explicit --torque limit')
            if operation in ('grip_torque', 'grip_force'):
                if torque_limit_nm:
                    raise ValueError('grip command uses value as its effort target; omit torque_limit_nm')
                if operation == 'grip_force' and c.force_n_per_nm <= 0:
                    raise ValueError('newton commands require measured force_n_per_nm calibration')
                torque = value if operation == 'grip_torque' else value/c.force_n_per_nm
                if torque <= 0:
                    raise ValueError('grip effort must be positive')
            closing = opening < current
            torque = torque or (c.close_torque_nm if closing else c.open_torque_nm)
            speed_scale = speed_scale or (min(c.default_speed_scale, .25) if calibrating else c.default_speed_scale)
            acceleration_scale = acceleration_scale or (min(c.default_acceleration_scale, .25) if calibrating else c.default_acceleration_scale)
            if not 0 < speed_scale <= 1 or not 0 < acceleration_scale <= 1:
                raise ValueError('speed/acceleration scaling must be in (0,1]')
            if calibrating and (speed_scale > .25 or acceleration_scale > .25):
                raise ValueError('calibration speed/acceleration cannot exceed 0.25')
            if not .01 <= torque <= c.max_torque_nm:
                raise ValueError('torque exceeds configured range')
            # The device torque register has 0.01 Nm resolution; round down.
            torque = math.floor(torque*100+1e-9)/100
            position = position if calibrating else c.position(opening)
            movement_sign = math.copysign(1., position-self.sample.position)
            plan = Plan(uuid.uuid4().hex, operation, position,
                        c.max_speed_rps*speed_scale, c.max_acceleration_rps2*acceleration_scale,
                        torque, timeout_s or c.command_timeout_s, time.monotonic(), closing=closing,
                        movement_sign=movement_sign)
            if calibrating:
                self.calibration_session = True
            self.active = plan
            self.phase = 'accepted'
            self.wake.set()
            return plan.command_id

    def stop(self):
        with self.lock:
            self._fresh()
            if self.stop_pending:
                return self.stop_pending.command_id
            c = self.config
            torque = self.active.torque if self.active else c.close_torque_nm
            self.stop_pending = Plan(uuid.uuid4().hex, 'stop', self.sample.position,
                                     c.max_speed_rps, c.max_acceleration_rps2,
                                     torque, min(3., c.command_timeout_s), time.monotonic())
            self.wake.set()
            return self.stop_pending.command_id

    def get_result(self, identifier):
        with self.lock:
            if identifier in self.results:
                return True, True, self.results[identifier]
            pending = [p.command_id for p in (self.active, self.stop_pending) if p]
            return identifier in pending, False, None

    def _check(self, sample):
        if sample.fault or sample.mode == 1:
            raise RuntimeError(f'motor fault={sample.fault}, mode={sample.mode}; no automatic reset')
        # A saved endpoint is a motion target, not a validity boundary for
        # feedback. Fingertip changes, manual movement and stall unloading can
        # legitimately place the encoder just outside the old span. Normal
        # commands always target a point inside the saved span and therefore
        # bring the mechanism back in-bounds instead of making it unusable.
        # Guarded calibration is the one exception: its deliberate overtravel
        # remains bounded by calibration_probe_margin_rev.
        plan = self.active
        if plan and plan.operation in ('calibrate_open', 'calibrate_close'):
            c = self.config
            low, high = sorted((c.open_position_rev, c.close_position_rev))
            low -= c.calibration_probe_margin_rev+c.position_tolerance_rev
            high += c.calibration_probe_margin_rev+c.position_tolerance_rev
            if not low <= sample.position <= high:
                raise RuntimeError('calibration probe exceeded guarded travel envelope')

    def _record(self, sample):
        with self.lock:
            self.sample = sample
            self.connected = True
            self.ever_connected = True
        self._check(sample)

    def _finish(self, plan, outcome, detail):
        sample = self.sample
        result = dict(command_id=plan.command_id,
                      success=outcome in ('reached', 'closed', 'contact'), outcome=outcome, detail=detail,
                      at_closed_reference=self.at_closed_reference(sample) and self.connected,
                      position_rev=sample.position if sample else math.nan,
                      opening_fraction=self.config.opening(sample.position) if sample else math.nan,
                      torque_nm=sample.torque if sample else math.nan,
                      accepted_to_send_ms=(plan.sent_at-plan.accepted_at)*1000 if plan.sent_at else math.nan,
                      duration_s=time.monotonic()-plan.accepted_at)
        with self.lock:
            self.results[plan.command_id] = result
            while len(self.results) > 128:
                self.results.popitem(last=False)
            if self.active is plan:
                self.active = None
            self.phase = outcome
        self.on_result(result)

    def _hold(self):
        # Always obtain a fresh position. A lost link cannot guarantee a stop.
        sample = self.transport.exchange()
        self._record(sample)
        must_cancel_target = self.active is not None and self.active.sent_at > 0
        if abs(sample.velocity) < self.config.stationary_velocity_rps and not must_cancel_target:
            # Preserve an established clamp, and do not energize an idle motor
            # merely because a read-only session is shutting down.
            return sample
        c = self.config
        torque = self.active.torque if self.active else c.close_torque_nm
        sample = self.transport.exchange(target(sample.position, c.max_speed_rps,
                                                c.max_acceleration_rps2, torque))
        self._record(sample)
        deadline = time.monotonic()+min(3., c.command_timeout_s)
        stable = None
        while time.monotonic() < deadline:
            if sample.mode == 10 and abs(sample.velocity) < c.stationary_velocity_rps:
                stable = stable or time.monotonic()
                if time.monotonic()-stable >= c.settle_time_s:
                    return sample
            else:
                stable = None
            time.sleep(1/c.poll_hz)
            sample = self.transport.exchange()
            self._record(sample)
        raise RuntimeError('stationary hold could not be confirmed')

    def _tick(self):
        with self.lock:
            stop = self.stop_pending
            plan = self.active
        if stop:
            stop.sent_at = time.monotonic()
            self._hold()
            if plan:
                self._finish(plan, 'cancelled', 'stop requested; stationary hold confirmed')
            self._finish(stop, 'reached', 'stationary hold confirmed')
            with self.lock:
                self.stop_pending = None
            return
        if plan and not plan.sent_at:
            # Fresh preflight before any actuation, including after queue delay.
            self._record(self.transport.exchange())
            with self.lock:
                if self.stop_pending or self.shutting_down:
                    return
                plan.sent_at = time.monotonic()
                self.phase = 'moving'
            self._record(self.transport.exchange(target(plan.position, plan.speed, plan.acceleration, plan.torque)))
        else:
            self._record(self.transport.exchange())
        if not plan:
            if self.phase == 'connecting':
                self.phase = 'idle'
            return
        s, c = self.sample, self.config
        stationary = abs(s.velocity) < c.stationary_velocity_rps and s.mode == 10
        # A loaded mechanism can initially unload opposite the requested
        # direction. That does not arm stall detection for the new move.
        if s.velocity*plan.movement_sign >= c.stationary_velocity_rps:
            plan.motion_observed = True
        arrived = abs(s.position-plan.position) <= c.position_tolerance_rev
        closing_sign = math.copysign(1., c.close_position_rev-c.open_position_rev)
        probe = plan.operation in ('calibrate_open', 'calibrate_close')
        loaded = s.torque*closing_sign >= c.contact_ratio*plan.torque
        # Opening initially unloads the old clamp: its stationary positive
        # torque is not an opening obstruction. Match the commissioned driver:
        # only closing uses torque-contact completion; opening waits for its
        # position or the bounded deadline.
        probe_stalled = probe and plan.motion_observed and stationary and not arrived
        condition = ('reached' if arrived else 'contact'
                     if probe_stalled or (loaded and plan.closing and not probe) else '') if stationary else ''
        if condition != plan.condition:
            plan.settled_at = time.monotonic() if condition else 0.
            plan.condition = condition
        if time.monotonic()-plan.accepted_at >= plan.timeout:
            self._hold()
            self._finish(plan, 'timeout', 'deadline elapsed; stationary hold confirmed')
        elif condition and time.monotonic()-plan.settled_at >= c.settle_time_s:
            if condition == 'contact':
                if probe:
                    # Cancel the deliberately out-of-range probe target before
                    # publishing/saving the measured endpoint. Otherwise a
                    # transient stall could unload and continue moving after
                    # calibration reported completion.
                    self._hold()
                    self._finish(plan, 'contact', 'calibration stall detected after observed motion; save this endpoint only with empty, clear jaws')
                elif plan.operation in ('close', 'grip_torque', 'grip_force') and plan.closing:
                    self._finish(plan, 'contact', 'sustained stationary torque before empty-jaw reference; object or resistance, not full closure')
                else:
                    self._hold()
                    self._finish(plan, 'blocked', 'target obstructed before arrival')
            elif probe:
                self._finish(plan, 'blocked', 'calibration probe limit reached without detecting resistance; endpoint not updated')
            elif plan.operation in ('grip_torque', 'grip_force') and not loaded:
                self._finish(plan, 'blocked', 'closed endpoint reached without requested effort')
            else:
                closed = plan.operation in ('close', 'grip_torque', 'grip_force') and self.at_closed_reference(s)
                self._finish(plan, 'closed' if closed else 'reached',
                             'empty-jaw reference reached; not proof that jaws are empty' if closed else 'position reached and stationary')

    def _run(self):
        try:
            while not self.shutting_down:
                self.wake.clear()
                start = time.monotonic()
                try:
                    if self.transport is None:
                        if self.transport_factory is None:
                            break
                        self.transport = self.transport_factory()
                    self._tick()
                    if not self.fault_latched:
                        self.error = ''
                        if self.phase in ('communication_error', 'connecting'):
                            self.phase = 'idle'
                except Exception as exc:
                    # pyserial can expose a USB removal either as OSError /
                    # SerialException or directly as termios.error. All are
                    # transport loss, not a motor fault.
                    outcome = ('communication_error'
                               if isinstance(exc, (OSError, ValueError, termios.error)) else 'fault')
                    detail = str(exc)
                    # One bounded fresh hold attempt; never replay a failed command.
                    if self.active and self.active.sent_at:
                        try:
                            self._hold()
                            detail += '; stationary hold confirmed'
                        except Exception as hold_error:
                            detail += f'; HOLD UNCONFIRMED: {hold_error}'
                    with self.lock:
                        active, stop = self.active, self.stop_pending
                        interrupted_motion = active is not None or stop is not None
                        self.error, self.phase = detail, outcome
                        # Idle USB hotplug is safe to recover automatically: no
                        # movement was in flight and reconnect performs reads only.
                        # Preserve explicit acknowledgement for motor faults or
                        # communication loss during a command, whose physical
                        # outcome cannot be inferred after feedback disappears.
                        self.fault_latched = (self.fault_latched or outcome == 'fault'
                                              or (self.ever_connected and interrupted_motion))
                        self.connected = outcome == 'fault' and self.sample is not None
                        self.stop_pending = None
                    for plan in (active, stop):
                        if plan:
                            self._finish(plan, outcome, detail)
                    if self.transport_factory is None:
                        break
                    if outcome == 'communication_error' and self.transport is not None:
                        self.transport.close()
                        self.transport = None
                    # Retry reads only. Results and the motion-inhibit latch survive.
                    self.wake.wait(self.config.reconnect_interval_s)
                    continue
                self.wake.wait(max(0., 1/self.config.poll_hz-(time.monotonic()-start)))
        finally:
            if self.shutting_down and not self.error and self.sample:
                try:
                    self._hold()
                    if self.active:
                        self._finish(self.active, 'cancelled', 'shutdown; stationary hold confirmed')
                except Exception as exc:
                    self.error = f'shutdown HOLD UNCONFIRMED: {exc}'
            if self.transport is not None:
                self.transport.close()

    def close(self):
        self.shutting_down = True
        self.wake.set()
        self.thread.join(5.)
        if self.thread.is_alive():
            raise RuntimeError('serial worker did not terminate')
