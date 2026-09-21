"""Absolute monotonic deadlines; physics time never depends on sleep accuracy."""

import math
import time


class SimulationClock:
    def __init__(self, frequency=100.0, fast=True, *, now=time.monotonic, sleep=time.sleep):
        if not math.isfinite(frequency) or frequency <= 0:
            raise ValueError("frequency must be finite and positive")
        self.dt = 1.0 / frequency
        if not math.isfinite(self.dt) or self.dt <= 0:
            raise ValueError("frequency must produce a finite positive timestep")
        self.fast = fast
        self.now = now
        self.sleep = sleep
        self.started = now()
        self.overruns = 0
        self.max_lateness_s = 0.0

    def wait(self, completed_steps: int) -> None:
        if self.fast:
            return
        remaining = self.started + completed_steps * self.dt - self.now()
        if remaining > 0:
            self.sleep(remaining)
        else:
            self.overruns += 1
            self.max_lateness_s = max(self.max_lateness_s, -remaining)

    @property
    def elapsed_s(self) -> float:
        return self.now() - self.started
