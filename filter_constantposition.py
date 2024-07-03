import numpy as np

class ComplexKalmanFilter:
    def __init__(self):
        self.x = np.zeros(2)  # Zustandsvektor [x, y]
        self.P = np.eye(2)  # Kovarianzmatrix
        self.F = np.eye(2)  # Zustandsübergangsmatrix
        self.H = np.eye(2)  # Messmatrix
        self.R = np.eye(2) * 0.04  # Messrauschkovarianz
        self.I = np.eye(2)  # Einheitsmatrix

    def reset(self, measurement):
        self.x = measurement[:2]
        self.P = np.eye(2)
        return self.x

    def update(self, dt, measurement):
        z = measurement[:2]
        y = z - self.x
        S = self.P + self.R
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K) @ self.P
        return self.x
