import numpy as np
import pickle
import matplotlib.pyplot as plt

class AngularFilter:
    def __init__(self, r_noise_std=0.01, phi_noise_std=0.05):
        self.est_position = np.zeros(2)
        self.P = np.eye(2) * 100
        self.R = np.array([[r_noise_std**2, 0.0], 
                           [0.0, phi_noise_std**2]])
        self.Q = np.eye(2) * r_noise_std

    def reset(self,measurement):
        r, phi = measurement
        self.est_position = np.array([[r * np.cos(phi)], [r * np.sin(phi)]])
        self.P = np.eye(2) * 10
        #print(f"Reset called with measurement: {measurement}")
        #print(f"Initial position set to: {self.est_position.flatten()}")
        return self.est_position.flatten() #its a (2, ) vector

    def update(self, dt, measurement, r_noise_std=0.01):
        self.P += self.Q
        r_meas, phi_meas = measurement
        x, y = self.est_position[0, 0], self.est_position[1, 0]
        r_est = np.sqrt(x**2 + y**2)
        phi_est = np.arctan2(y, x)

        epsilon = 1e-6
        r_est = max(r_est, epsilon)

        z_est = np.array([r_est, phi_est]) #Column Vectors!!!!!
        z_meas = np.array([r_meas, phi_meas])

        H = np.array([
            [x / r_est, y / r_est],
            [-y / (r_est**2), x / (r_est**2)]
        ])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        innovation = z_meas - z_est
        innovation[1] = np.arctan2(np.sin(innovation[1]), np.cos(innovation[1]))

        self.est_position += np.reshape(K @ innovation, (2, 1))  # Reshape to (2, 1) vector
        self.P = (np.eye(2) - K @ H) @ self.P
        return self.est_position.flatten()
