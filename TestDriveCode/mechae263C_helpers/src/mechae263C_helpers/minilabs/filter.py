import numpy as np


class ExponentialFilter:
    def __init__(self, smoothing_coeff: float, num_warmup_time_steps: int = 0):
        self.smoothing_coeff = max(min(float(smoothing_coeff), 1.0), 0.0)
        self.prev_output = 0.0
        self.is_first_run = True
        self.num_warmup_time_steps = max(round(num_warmup_time_steps), 0)
        self.warmup_time_step_counter = 0

    def __call__(self, x: float) -> float:
        if self.is_first_run:
            self.is_first_run = False
            self.prev_output = x
        else:
            self.prev_output = (
                1 - self.smoothing_coeff
            ) * x + self.smoothing_coeff * self.prev_output

        if self.warmup_time_step_counter <= self.num_warmup_time_steps:
            self.warmup_time_step_counter += 1
            return 0.0
        else:
            return self.prev_output


class KalmanDerivativeEstimator:
    def __init__(self, x0, dx0, ddx0, dt, process_var=1e-2, meas_var=1e-1):
        self.dt = dt
        dt2 = dt * dt
        dt3 = dt2 * dt / 2
        dt4 = dt2 * dt2 / 4

        # State vector: [position, velocity, acceleration]
        self.x = np.array([[x0],
                           [dx0],
                           [ddx0]])

        # State transition matrix (constant acceleration model)
        self.A = np.array([
            [1, dt, 0.5 * dt2],
            [0,  1,       dt],
            [0,  0,        1]
        ])

        # Measurement matrix: only observe position
        self.H = np.array([[1, 0, 0]])

        # Process noise covariance
        self.Q = process_var * np.array([
            [dt4, dt3, dt2],
            [dt3, dt2, dt],
            [dt2, dt,  1]
        ])

        # Measurement noise covariance
        self.R = np.array([[meas_var]])

        # Initial uncertainty
        self.P = np.eye(3) * 1e-3

    def __call__(self, z_meas):
        z = np.array([[z_meas]])

        # Predict
        x_pred = self.A @ self.x
        P_pred = self.A @ self.P @ self.A.T + self.Q

        # Innovation
        y = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K
