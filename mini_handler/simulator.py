"""Deterministic acceleration-limited motor model. No hardware access."""
import math
import struct
import time
from .protocol import Sample


class SimulatedMotor:
    def __init__(self, config):
        self.c = config
        self.position = self.target = config.open_position_rev
        self.velocity = 0.
        self.speed = self.acceleration = 1.
        self.limit = 3.5
        self.last = time.monotonic()
        self.rtt_ms = 0.
        self.fault = 0
        self.disconnected = False
        self.commands = []

    def exchange(self, command=b''):
        if self.disconnected:
            raise TimeoutError('simulated cable loss')
        if command:
            self.commands.append(command)
            self.target = struct.unpack_from('<i', command, 5)[0]*1e-5
            self.speed, self.acceleration = struct.unpack_from('<ff', command, 15)
            self.limit = struct.unpack_from('<h', command, 25)[0]*.01
        now = time.monotonic()
        dt = min(now-self.last, .1)
        self.last = now
        delta = self.target-self.position
        desired = math.copysign(min(self.speed, math.sqrt(2*self.acceleration*abs(delta))), delta)
        self.velocity += max(-self.acceleration*dt, min(self.acceleration*dt, desired-self.velocity))
        step = self.velocity*dt
        if abs(step) >= abs(delta) and step*delta >= 0:
            self.position, self.velocity = self.target, 0.
        else:
            self.position += step
        torque = .08 if abs(self.velocity) > .001 else 0.
        contact = self.c.simulated_contact_fraction
        if contact >= 0 and self.c.opening(self.target) < contact and self.c.opening(self.position) <= contact:
            self.position, self.velocity, torque = self.c.position(contact), 0., self.limit
        return Sample(self.position, self.velocity, torque, 10, self.fault, 24., 35., now)

    def close(self):
        pass
