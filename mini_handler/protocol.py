"""HighTorque CAN-FD register protocol over the fdcanusb serial adapter.

Protocol provenance: operator-supplied ht_motor.py, integrated in
dff_mobile_manipulation_docker on 2026-09-18. Reimplemented here without
robot-stack dependencies. Position command velocity is ZERO; 0x28/0x29
are the independent speed and acceleration limits. Never resets motor faults.
"""
import math
import struct
import time
from dataclasses import dataclass


@dataclass
class Sample:
    position: float = math.nan
    velocity: float = math.nan
    torque: float = math.nan
    mode: int = -1
    fault: int = -1
    voltage: float = math.nan
    temperature: float = math.nan
    received_at: float = 0.0


def query(extended=False):
    # Float feedback avoids the coarse/int16 position span of the legacy driver.
    return b'\x11\x00\x11\x0f\x1f\x01' + (b'\x1e\x0d' if extended else b'')


def target(position, speed, acceleration, torque):
    if not all(math.isfinite(v) for v in (position, speed, acceleration, torque)):
        raise ValueError('nonfinite command')
    if min(speed, acceleration, torque) <= 0 or torque > 327.67:
        raise ValueError('invalid motion limits')
    return (b'\x01\x00\x0a\x0a\x20'
            + struct.pack('<ii', round(position * 1e5), 0)
            + b'\x0e\x28' + struct.pack('<ff', speed, acceleration)
            + b'\x05\x25' + struct.pack('<h', math.floor(torque * 100 + 1e-9)))


def parse(data, sample=None):
    sample = sample or Sample()
    names = {0: 'mode', 1: 'position', 2: 'velocity', 3: 'torque',
             13: 'voltage', 14: 'temperature', 15: 'fault'}
    i = 0
    while i < len(data):
        h = data[i]; i += 1
        if h == 0x50:
            continue
        dtype, count = (h >> 2) & 3, h & 3
        if count == 0:
            if i >= len(data):
                raise ValueError('truncated count')
            count = data[i]; i += 1
        if i >= len(data):
            raise ValueError('truncated register')
        reg = data[i]; i += 1
        size = (1, 2, 4, 4)[dtype]
        if h >> 4 != 2 or count == 0 or i + size * count > len(data):
            raise ValueError('malformed reply')
        for offset in range(count):
            value = struct.unpack_from(('<b', '<h', '<i', '<f')[dtype], data, i)[0]
            i += size
            r = reg + offset
            if r not in names:
                continue
            if r in (0, 15):
                if dtype != 0:
                    raise ValueError('invalid mode/fault encoding')
                value &= 255
            else:
                if dtype == 0 or (dtype == 1 and value == -32768) or (dtype == 2 and value == -2147483648):
                    raise ValueError('invalid telemetry')
                if dtype != 3:
                    scales = {1: {1: 1e-4, 2: 2.5e-4, 3: .01},
                              2: {1: 1e-5, 2: 1e-5, 3: .001}}
                    if r not in scales[dtype]:
                        raise ValueError('diagnostics require float encoding')
                    value *= scales[dtype][r]
                if not math.isfinite(value):
                    raise ValueError('nonfinite telemetry')
            setattr(sample, names[r], value)
    return sample


class SerialMotor:
    def __init__(self, port, motor_id=1, timeout=.1, extended=True):
        import serial
        self.motor_id, self.timeout, self.extended = motor_id, timeout, extended
        self.serial = serial.Serial(port, 3_000_000, timeout=.02,
                                    write_timeout=timeout, exclusive=True)
        self.buffer = bytearray()
        self.rtt_ms = 0.0
        try:
            self.serial.reset_input_buffer()
            for cmd in ('can off', 'conf set can.bitrate 1000000',
                        'conf set can.fd_bitrate 5000000', 'conf set can.fdcan_frame 1',
                        'conf set can.bitrate_switch 0',
                        'conf set can.automatic_retransmission 0', 'can on'):
                self._configure(cmd)
        except Exception:
            self.serial.close()
            raise

    def _line(self, deadline):
        while time.monotonic() < deadline:
            if b'\n' in self.buffer:
                line, _, self.buffer = self.buffer.partition(b'\n')
                return line.decode('ascii', errors='strict').strip()
            self.serial.timeout = max(.001, deadline - time.monotonic())
            self.buffer.extend(self.serial.read(max(1, self.serial.in_waiting)))
        raise TimeoutError('serial reply timeout')

    def _configure(self, cmd):
        self.serial.write((cmd + '\n').encode())
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            line = self._line(deadline)
            if line == 'OK' or (cmd == 'can off' and line == 'ERR already in BusOff'):
                return
            if line.startswith('ERR'):
                raise OSError(f'adapter rejected {cmd}: {line}')
        raise TimeoutError('adapter configuration timeout')

    def exchange(self, command=b''):
        # One worker owns this transport. No queued requests or concurrent reads.
        self.buffer.clear()
        self.serial.reset_input_buffer()
        payload = command + query(self.extended)
        length = next(n for n in (8, 12, 16, 20, 24, 32, 48, 64) if n >= len(payload))
        payload += b'\x50' * (length - len(payload))
        start = time.monotonic()
        self.serial.write(f'can ext {0x8000 | self.motor_id:X} {payload.hex()} F\n'.encode())
        deadline = start + self.timeout
        sample = Sample()
        while time.monotonic() < deadline:
            line = self._line(deadline)
            if line.startswith('ERR'):
                raise OSError(f'adapter error: {line}')
            parts = line.split()
            if len(parts) < 3 or parts[0] != 'rcv':
                continue
            aid = int(parts[1], 16)
            if aid not in (self.motor_id << 8, 0x8000 | (self.motor_id << 8), 0x8000 | self.motor_id):
                continue
            sample = parse(bytes.fromhex(parts[2]), sample)
            if (sample.mode >= 0 and sample.fault >= 0
                    and all(math.isfinite(v) for v in (sample.position, sample.velocity, sample.torque))
                    and (not self.extended or all(math.isfinite(v) for v in (sample.voltage, sample.temperature)))):
                sample.received_at = time.monotonic()
                self.rtt_ms = (sample.received_at - start) * 1000
                return sample
        raise TimeoutError('incomplete fresh motor feedback')

    def close(self):
        # Closing serial is not a motor stop. The controller owns explicit hold.
        self.serial.close()
