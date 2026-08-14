"""The angle sensor model — quantisation, delay, optional noise.

Feedback controllers see the world through this, so their robustness claims
are claims about the real system, not about a perfect observer. Quantisation
models an encoder/resolver step; the delay models the sampling pipeline
(one control period by default); noise is **off by default** and seeded, so
every run with the same seed and sequence is bit-reproducible.

The legacy open-loop reference does not read the sensor at all — the
behaviour it reproduces (``ByWirePlant``'s angle channel) predates it, and
its bit-exactness contract forbids adding a measurement in the loop.
"""

from __future__ import annotations

from collections import deque

import numpy as np


class AngleSensor:
    def __init__(
        self,
        *,
        quant_rad: float = 5.0e-4,   # ~0.03 deg — a resolver step
        delay_steps: int = 1,
        noise_std_rad: float = 0.0,
        seed: int = 1234,
    ) -> None:
        self.quant = float(quant_rad)
        self.noise_std = float(noise_std_rad)
        #: One measurement per call; `delay_steps`+1 holds the pipeline. A
        #: discrete sensor is always one sample behind even with
        #: delay_steps = 0, so the reading is delayed by delay_steps+1
        #: periods in total.
        self._buf: deque[float] = deque([0.0] * (max(int(delay_steps), 0) + 1))
        #: Seeded at construction — identical sequences for identical configs.
        self._rng = np.random.default_rng(int(seed))

    def measure(self, truth: float) -> float:
        """Return the delayed, quantised reading of `truth`."""
        v = float(truth)
        if self.noise_std > 0.0:
            v += float(self._rng.normal(0.0, self.noise_std))
        if self.quant > 0.0:
            v = self.quant * round(v / self.quant)
        self._buf.append(v)
        return self._buf.popleft()
