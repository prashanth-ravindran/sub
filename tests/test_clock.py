import pytest

from simulator.simulation.clock import SimulationClock


def test_absolute_deadlines_do_not_accumulate_work_or_sleep_drift():
    wall = [10.0]
    sleeps = []

    def sleep(seconds):
        sleeps.append(seconds)
        wall[0] += seconds + 0.001  # Deliberate oversleep.

    clock = SimulationClock(100, now=lambda: wall[0], sleep=sleep)
    clock.wait(1)
    assert sleeps == []  # Accelerated by default.
    clock.fast = False
    wall[0] += 0.003
    clock.wait(1)
    wall[0] += 0.002
    clock.wait(2)
    assert sleeps == pytest.approx([0.007, 0.007])
    wall[0] = 10.035
    clock.wait(3)
    assert clock.overruns == 1
    assert clock.max_lateness_s == pytest.approx(0.005)
    clock.wait(4)
    assert sleeps[-1] == pytest.approx(0.005)
    clock.fast = True
    clock.wait(5)
    assert len(sleeps) == 3


@pytest.mark.parametrize("frequency", [0, -1, float("nan"), float("inf")])
def test_invalid_frequency_is_rejected(frequency):
    with pytest.raises(ValueError):
        SimulationClock(frequency)
