import numpy as np
import matplotlib.pyplot as plt

#For Problem3 we will implement an extended Kalman Filter because the relationship between the Cartesian coordinates 
# and the polar coordinates is nonlinear.
class AngularFilter:
    def __init__(self, r_noise_std=0.01, phi_noise_std=0.05):
        self.est_position = np.zeros(2) # here we rename x = est_position and y = innovation 
#to avoid confusion with the true coordinates. Although the true coordinates are r and phi, we use x,y notation 
# to calculate Jacobi Matrix for our measurement matrix H
        self.P = np.eye(2) * 100
        self.R = np.array([[r_noise_std**2, 0.0], 
                           [0.0, phi_noise_std**2]])
        self.Q = np.eye(2) * r_noise_std

    def reset(self,measurement):
        r, phi = measurement
        self.est_position = np.array([[r * np.cos(phi)], [r * np.sin(phi)]])
        self.P = np.eye(2) * 10 #lower uncertainty
        return self.est_position.flatten() #its a (2, ) vector

    def update(self, dt, measurement, r_noise_std=0.01):
        self.P += self.Q
        r_meas, phi_meas = measurement
        x, y = self.est_position[0, 0], self.est_position[1, 0]
        r_est = np.sqrt(x**2 + y**2)
        phi_est = np.arctan2(y, x)

# To avoid division by 0 while calculating H, let us add a small epsilon to the r_est
        epsilon = 1e-6
        r_est = max(r_est, epsilon)

        z_est = np.array([r_est, phi_est]) #column vectors!!
        z_meas = np.array([r_meas, phi_meas])

# The Jacobian measurement matrix H linearizes the elements of Covar matrix 𝑃 during each measurement step,
# transforming it from state space to measurement space
        H = np.array([
            [x / r_est, y / r_est],
            [-y / (r_est**2), x / (r_est**2)]
        ])

        S = H @ self.P @ H.T + self.R # Innovation Covar Matrix of the measurements
        K = self.P @ H.T @ np.linalg.inv(S)

        innovation = z_meas - z_est ## innovation or prediction error

 # Without this normalization of the angle innovation, it will not lie within -pi to pi
        innovation[1] = np.arctan2(np.sin(innovation[1]), np.cos(innovation[1]))

        self.est_position += np.reshape(K @ innovation, (2, 1))  # we update and reshape our state estimate to (2, 1) vector
        self.P = (np.eye(2) - K @ H) @ self.P # update Covar estimate matrix P
        return self.est_position.flatten()
