import math
import os
import pty
import select
import struct
import threading
import time
import pytest
from mini_handler.protocol import SerialMotor, parse, query, target


def feedback(position=.12345):
    return (b'\x21\x00\x0a\x21\x0f\x00\x2f\x01'
            + struct.pack('<fff', position, .02, 2.5)
            + b'\x2e\x0d' + struct.pack('<ff', 24., 35.))


def test_wire_command_uses_stationary_target_and_independent_limits():
    wire = target(.175, .6, 1., 3.5)
    assert wire[:5] == b'\x01\x00\x0a\x0a\x20'
    assert struct.unpack_from('<ii', wire, 5) == (17500, 0)
    assert wire[13:15] == b'\x0e\x28'
    assert struct.unpack_from('<ff', wire, 15) == pytest.approx((.6, 1.))
    assert wire[23:25] == b'\x05\x25'
    assert struct.unpack_from('<h', wire, 25) == (350,)


def test_parse_complete_float_and_fragmented_reply():
    wire = feedback()
    st = parse(wire[:6])
    st = parse(wire[6:]+b'\x50'*3, st)
    assert st.position == pytest.approx(.12345)
    assert st.torque == 2.5
    assert st.mode == 10 and st.fault == 0
    assert st.voltage == 24. and st.temperature == 35.


@pytest.mark.parametrize('wire', [b'\x27', b'\x20', b'\x27\x01\x00',
    b'\x27\x01\x00\x80\x00\x00\x00\x00', b'\x2d\x01'+struct.pack('<f', math.nan),
    b'\x11\x00', b'\x2d\x00'+struct.pack('<f', 10)])
def test_reject_malformed_telemetry(wire):
    with pytest.raises(ValueError):
        parse(wire)


class Adapter:
    """Real OS pseudo-terminal exercising pyserial, ASCII CAN framing and timeouts."""
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.running, self.drop, self.fragment, self.error = True, False, False, False
        self.lines = []
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        buf = b''
        while self.running:
            if not select.select([self.master], [], [], .02)[0]:
                continue
            try:
                buf += os.read(self.master, 4096)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.decode().strip()
                    self.lines.append(line)
                    if line.startswith('can ext'):
                        if self.drop:
                            continue
                        if self.error:
                            os.write(self.master, b'ERR BusOff\n')
                            continue
                        data = feedback()
                        # Wrong motor must not be accepted.
                        os.write(self.master, b'rcv 200 '+feedback(.8).hex().encode()+b'\n')
                        if self.fragment:
                            os.write(self.master, b'rcv 100 '+data[:6].hex().encode()+b'\n')
                            os.write(self.master, b'rcv 100 '+data[6:].hex().encode()+b'\n')
                        else:
                            os.write(self.master, b'rcv 100 '+data.hex().encode()+b'\n')
                    elif line == 'can off':
                        os.write(self.master, b'ERR already in BusOff\n')
                    else:
                        os.write(self.master, b'OK\n')
            except OSError:
                return

    def close(self):
        self.running = False
        self.thread.join(.2)
        os.close(self.master)
        os.close(self.slave)


@pytest.fixture
def serial_rig():
    adapter = Adapter()
    motor = SerialMotor(adapter.path, timeout=.03)
    yield motor, adapter
    motor.close()
    adapter.close()


def test_persistent_serial_framing_and_exclusive_owner(serial_rig):
    motor, adapter = serial_rig
    for _ in range(3):
        st = motor.exchange(target(.1, .2, .3, 2.))
        assert st.position == pytest.approx(.12345)
    assert len([s for s in adapter.lines if s == 'can on']) == 1
    assert 'conf set can.bitrate_switch 0' in adapter.lines
    with pytest.raises(OSError):
        SerialMotor(adapter.path)
    frame = next(s for s in adapter.lines if s.startswith('can ext'))
    assert frame.split()[2] == '8001'
    assert len(bytes.fromhex(frame.split()[3])) == 48


def test_serial_fragmented_replies(serial_rig):
    motor, adapter = serial_rig
    adapter.fragment = True
    assert motor.exchange().torque == 2.5


def test_timeout_does_not_reuse_previous_sample(serial_rig):
    motor, adapter = serial_rig
    assert motor.exchange().fault == 0
    adapter.drop = True
    start = time.monotonic()
    with pytest.raises(TimeoutError):
        motor.exchange()
    assert time.monotonic()-start < .2


def test_adapter_error_is_not_silently_ignored(serial_rig):
    motor, adapter = serial_rig
    adapter.error = True
    with pytest.raises(OSError, match='BusOff'):
        motor.exchange()
