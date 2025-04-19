import threading
import time


class FixedFrequencyLoopManager:
    def __init__(self, freq_Hz: float):
        super().__init__()

        self._period_ns = abs(round(1e9 / freq_Hz))

        if self._period_ns == 0:
            raise ValueError(
                "Argument `period_ns` must be an integer strictly greater than 0"
            )

        self._prev_time = time.time_ns()

    @property
    def period_s(self):
        return self._period_ns / 1e9

    def reset(self):
        self._prev_time = time.time_ns()

    def sleep(self):
        curr_time = time.time_ns()

        # Calculate expected `next_time` for this loop iteration
        next_time = self._prev_time + self._period_ns

        # If the `prev_time` is after the `curr_time`, then set `next_time` to be
        # `period` after `current_time`
        if curr_time < self._prev_time:
            next_time = curr_time + self._period_ns

        # Update `prev_time` by user specified period
        self._prev_time += self._period_ns

        # If `curr_time` is after the calculated `next_time`, then loop frequency
        # is below desired.
        if curr_time >= next_time:
            # If off by a whole loop period or more, then reset the `prev_time`
            if curr_time > next_time + self._period_ns:
                self._prev_time = curr_time + self._period_ns

            return

        time.sleep((next_time - curr_time) / 1e9)
