import numpy as np

class KalmanFilterWithRandomNoise:
    def __init__(self):
        self.x = np.zeros(2)
        self.P = np.eye(2)
        self.F = np.eye(2)
        self.H = np.eye(2)
        self.I = np.eye(2)

    def reset(self, measurement):
        self.x = measurement[:2]
        self.P = np.eye(2)
        return self.x

    def update(self, dt, measurement):
        z = measurement[:2]
        Rt = measurement[2:].reshape(2, 2)
        self.P = self.F @ self.P @ self.F.T
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + Rt
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P
        return self.x
