import numpy as np

class ConstantTurnFilter:
    def __init__(self, q_noise_std=0.01):
        self.state = np.zeros(4)  # [x, y, vx, vy]
        self.P = np.eye(4) * 100  # Initial Covariance Matrix P
        self.Q = np.diag([q_noise_std**2, q_noise_std**2, (q_noise_std*10)**2, (q_noise_std*10)**2])  # Adjusted Process noise Q
        self.turn_rate = 0  # Estimated turn rate

    def reset(self, initial_state):
        self.state[:2] = initial_state[:2] 
        self.state[2:] = 0
        self.P = np.diag([10, 10, 1, 1])  # Adjusted initial covariance
        self.turn_rate = 0
        return self.state[:2]  # Return only position

    def update(self, dt, measurements, turn_rate=None):
        if turn_rate is not None:
            self.turn_rate = turn_rate
        else:
            # Estimate turn rate from current state
            v = np.linalg.norm(self.state[2:])
            if v > 0.1:
                self.turn_rate = (self.state[2] * -self.state[3] + self.state[3] * self.state[2]) / (v ** 2)

        # Calculate state transition matrix
        c = np.cos(self.turn_rate * dt)
        s = np.sin(self.turn_rate * dt)
        F = np.array([
            [1, 0, dt*c, -dt*s],
            [0, 1, dt*s,  dt*c],
            [0, 0,    c,    -s],
            [0, 0,    s,     c]
        ])

        # Predict step
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q * dt  # Scale Q by dt

        # Update step
        H = np.zeros((10, 4))
        H[0::2, 0] = 1  # x measurements
        H[1::2, 1] = 1  # y measurements

        z = measurements[:10]  # First 10 elements are measurements
        R = np.diag(measurements[10:]**2)  # Last 10 elements are std devs

        innovation = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.state = self.state + K @ innovation
        self.P = (np.eye(4) - K @ H) @ self.P

        # Ensure P remains symmetric and positive definite
        self.P = (self.P + self.P.T) / 2
        eigvals, eigvecs = np.linalg.eigh(self.P)
        eigvals = np.maximum(eigvals, 1e-6)
        self.P = eigvecs @ np.diag(eigvals) @ eigvecs.T
        return self.state[:2]  # Return only position