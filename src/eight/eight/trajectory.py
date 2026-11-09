"""Analytic, smoothly time-scaled figure-eight reference in PX4 local NED."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Sample:
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]


class FigureEight:
    def __init__(
        self,
        origin: tuple[float, float, float],
        amplitude_x: float = 2.0,
        amplitude_y: float = 1.0,
        period: float = 8.9,
        cycles: int = 2,
        ramp: float = 3.0,
    ) -> None:
        values = (*origin, amplitude_x, amplitude_y, period, ramp)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("trajectory parameters must be finite")
        if amplitude_x <= 0 or amplitude_y <= 0 or period <= 0 or cycles < 1:
            raise ValueError("amplitudes, period and cycles must be positive")
        if ramp <= 0 or 2 * ramp >= cycles * period + ramp:
            raise ValueError("ramp must fit inside the trajectory")
        self.origin = origin
        self.amplitude_x = amplitude_x
        self.amplitude_y = amplitude_y
        self.period = period
        self.cycles = cycles
        self.ramp = ramp
        self.omega = 2.0 * math.pi / period
        # Both ramps advance by half their duration at nominal angular speed.
        self.duration = cycles * period + ramp

    @staticmethod
    def _smoothstep(u: float) -> tuple[float, float, float]:
        speed_fraction = 10 * u**3 - 15 * u**4 + 6 * u**5
        derivative = 30 * u**2 - 60 * u**3 + 30 * u**4
        integral = 2.5 * u**4 - 3 * u**5 + u**6
        return speed_fraction, derivative, integral

    def _phase(self, time_sec: float) -> tuple[float, float, float]:
        if time_sec <= 0:
            return 0.0, 0.0, 0.0
        if time_sec >= self.duration:
            return 2.0 * math.pi * self.cycles, 0.0, 0.0
        if time_sec < self.ramp:
            u = time_sec / self.ramp
            fraction, derivative, integral = self._smoothstep(u)
            return (
                self.omega * self.ramp * integral,
                self.omega * fraction,
                self.omega * derivative / self.ramp,
            )
        if time_sec <= self.duration - self.ramp:
            return self.omega * (time_sec - self.ramp / 2.0), self.omega, 0.0
        u = (time_sec - (self.duration - self.ramp)) / self.ramp
        fraction, derivative, integral = self._smoothstep(u)
        return (
            self.omega * (self.duration - 1.5 * self.ramp)
            + self.omega * self.ramp * (u - integral),
            self.omega * (1.0 - fraction),
            -self.omega * derivative / self.ramp,
        )

    def sample(self, time_sec: float) -> Sample:
        if not math.isfinite(time_sec):
            raise ValueError("sample time must be finite")
        phase, phase_rate, phase_acceleration = self._phase(time_sec)
        sin_p, cos_p = math.sin(phase), math.cos(phase)
        sin_2p, cos_2p = math.sin(2.0 * phase), math.cos(2.0 * phase)
        ax, ay = self.amplitude_x, self.amplitude_y
        x0, y0, z0 = self.origin
        return Sample(
            position=(x0 + ax * sin_p, y0 + ay * sin_2p, z0),
            velocity=(ax * cos_p * phase_rate, 2.0 * ay * cos_2p * phase_rate, 0.0),
            acceleration=(
                ax * (-sin_p * phase_rate**2 + cos_p * phase_acceleration),
                2.0 * ay * (-2.0 * sin_2p * phase_rate**2 + cos_2p * phase_acceleration),
                0.0,
            ),
        )
