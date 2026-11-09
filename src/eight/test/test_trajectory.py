import math

import pytest

from eight.trajectory import FigureEight


def test_endpoint_and_derivatives_are_zero():
    trajectory = FigureEight((3.0, -2.0, -1.5))
    for time_sec in (0.0, trajectory.duration):
        sample = trajectory.sample(time_sec)
        assert sample.position == pytest.approx(trajectory.origin, abs=1e-12)
        assert sample.velocity == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
        assert sample.acceleration == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_analytic_derivatives_and_peak_speed():
    trajectory = FigureEight((0.0, 0.0, -2.0))
    dt = 1e-4
    peak_speed = 0.0
    for index in range(1, int(trajectory.duration / 0.1)):
        time_sec = index * 0.1
        before = trajectory.sample(time_sec - dt)
        sample = trajectory.sample(time_sec)
        after = trajectory.sample(time_sec + dt)
        for axis in range(3):
            numerical_velocity = (after.position[axis] - before.position[axis]) / (2 * dt)
            numerical_acceleration = (after.velocity[axis] - before.velocity[axis]) / (2 * dt)
            assert sample.velocity[axis] == pytest.approx(numerical_velocity, abs=1e-6)
            assert sample.acceleration[axis] == pytest.approx(numerical_acceleration, abs=1e-5)
        peak_speed = max(peak_speed, math.hypot(*sample.velocity[:2]))
    assert 1.95 <= peak_speed <= 2.01


def test_ramp_boundaries_are_continuous():
    trajectory = FigureEight((1.0, 2.0, -1.0))
    for boundary in (trajectory.ramp, trajectory.duration - trajectory.ramp):
        before = trajectory.sample(boundary - 1e-6)
        after = trajectory.sample(boundary + 1e-6)
        assert after.position == pytest.approx(before.position, abs=2e-6)
        assert after.velocity == pytest.approx(before.velocity, abs=5e-6)
        assert after.acceleration == pytest.approx(before.acceleration, abs=5e-6)


def test_invalid_parameters_are_rejected():
    with pytest.raises(ValueError):
        FigureEight((0.0, 0.0, 0.0), period=0.0)
    with pytest.raises(ValueError):
        FigureEight((0.0, 0.0, 0.0), ramp=math.nan)
