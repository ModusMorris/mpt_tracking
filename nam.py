import numpy as np


# Problem 1
class ConstantpositionFilter:
    def __init__(self):
        self.state = np.zeros(2) # The initial state is set to a zero vector
        self.uncertainty = np.eye(2) * 500  # Initial uncertainty is set to a matrix with 500 on the diagonal
        self.measurement_noise = 0.2 
        self.process_noise = 1e-5  # Process noise is set to a very small value

    def reset(self, measurement):
        self.state = np.array(measurement[:2]) # State is set to the first two measurements
        self.uncertainty = np.eye(2) * 500     # State is set to the first two measurements
        return self.state

    def update(self, dt, measurement):
        measurement = np.array(measurement[:2])
        measurement_uncertainty = np.eye(2) * self.measurement_noise**2

# The Kalman gain is calculated to determine how much the prediction should be adjusted based on the new measurement.
# Mathematically: K = P*(P+R)^-1
# P is the uncertainty matrix and R is the measurement uncertainty
        kalman_gain = self.uncertainty @ np.linalg.inv(
            self.uncertainty + measurement_uncertainty
        )

# State is updated
# The new state is updated based on the Kalman gain and the difference between the measurement 
# and the predicted state. Mathematically: x = x + K * (z - x)
# x = predicted state, K = Kalman gain, z = measurement
        self.state = self.state + kalman_gain @ (measurement - self.state)

#Uncertainty is updated
# The uncertainty matrix is adjusted to reflect the improved estimate: P = (I - K * H) * P, 
# where I = identity matrix
        self.uncertainty = (np.eye(2) - kalman_gain) @ self.uncertainty
        return self.state


# Problem 2
class RandomNoiseFilter:
    def __init__(self, process_noise_scale=0.04, initial_uncertainty=600):
        self.state = np.zeros((2, 1))  # Initial state vector (x, y)
        self.uncertainty = (
            np.eye(2) * initial_uncertainty
        )  # Initial uncertainty Covar Matrix P.We can print P after every update to see
        # if the diagonal high numbers (uncertainty) goes down
        self.process_noise_scale = process_noise_scale
        self.process_noise = (
            np.eye(2) * self.process_noise_scale
        )  # Process noise matrix Q, here stays const, unlike meas noise matrix R

    def reset(self, measurement):
        # Extract initial state from the measurement
        measurement = np.array(measurement[:2]).reshape((2, 1))
        self.state = measurement
        self.uncertainty = np.eye(2) * 500
        return self.state.flatten()

    def update(self, dt, measurement):
        # Extract the state measurements xy from z and reshape it to the shape of self.state, 
        # save the Covar meas Matrix R as a 2x2 matrix
        # for the further S calculation.
        z = np.array(measurement[:2]).reshape((2, 1))
        R = np.array(measurement[2:6]).reshape((2, 2))

        # Compute the Kalman gain. K optimizes the distribution of weights for our prediction and the new measurement
        # These weights determine the relative influence of the new measurement and the prediction on the updated state estimate
        # A higher Kalman gain means more trust is placed in the new measurement, 
        # whereas a lower Kalman gain means more trust for our prediction
        S = self.uncertainty + R
        K = self.uncertainty @ np.linalg.inv(S)

        # Innovation y shows us the difference between the actual measurement z from our state estimate 
        # (basically out prediction errror, also called measurement residual)
        y = z - self.state  # Innovation y
        self.state = self.state + K @ y  # Update of our state estimate
        I = np.eye(self.uncertainty.shape[0])  # Identity Matrix I
        # Update the uncertainty Covar P
        self.uncertainty = (I - K) @ self.uncertainty
        return (
            self.state.flatten()
        )  # returns the current state estimate as a 1d array bs otherwise main.py will throw an error

    # Problem3


class AngularFilter:
    def __init__(self):
        self.state = np.zeros(2)  # Initial state (x, y) is set to a zero vector
        self.uncertainty = np.eye(2) * 500  # Initial uncertainty is set to a matrix with 500 on the diagonal
# Measurement noise is a Matrix with normalised variances of distance measurement (r) and angle measurement (phi) on diagonals 
        self.measurement_noise = np.array([[0.01, 0], [0, 0.0025]])  # Measurement noise
        self.process_noise = np.eye(2) * 1e-5  # Process noise set as an identity matrix 2x2 with a tiny value

    def reset(self, measurement):
        r, phi = measurement  # Initialisation  of the polar coordinates 
   
# Coversion of the polar coordinates into the cartesian ones 
        x = r * np.cos(phi)
        y = r * np.sin(phi)
      
        self.state = np.array([x, y]) # Initial state is set to the now converted Cartesian coordinates
        self.uncertainty = np.eye(2) * 500 #We reset the uncertainty 
        return self.state

    def update(self, dt, measurement, *args, **kwargs):
        r, phi = measurement
        self.uncertainty += self.process_noise #Increase uncertainty to take notice of the process noise 
        measured_x = r * np.cos(phi) #same conversion of the coordinates
        measured_y = r * np.sin(phi)
        measurement_cartesian = np.array([measured_x, measured_y])

# Insert the cartesian coordinates into out measurements Covar Matrix H
        H = np.array([[np.cos(phi), -r * np.sin(phi)], [np.sin(phi), r * np.cos(phi)]])
        R_cartesian = H @ self.measurement_noise @ H.T #calculate the measurement noise Covar Matrix R 

# Calculate the Innovation Covar matrix S that is needed to calculate the Kalman gain
        S = self.uncertainty + R_cartesian
        K = self.uncertainty @ np.linalg.inv(S)

   # Innovation calculation and the update of the estimated state
        y = measurement_cartesian - self.state  # Innovation (prediction error) 
        self.state = self.state + K @ y

  # Update the uncertainty  matrix P
        I = np.eye(2)
        self.uncertainty = (I - K) @ self.uncertainty
        return self.state


# Problem 4
class KalmanFilterConstantVelocity:
    def __init__(self):
# The initial state is set to a zero vector (position and speed)
        self.x = np.zeros(4)   # State vector [x,y,vx,vy]
        self.P = np.eye(4)  # Initial uncertainty is set to an ide matrix
        # Messrauschen wird als Matrix mit 0.04 auf der Diagonale gesetzt
        self.R = np.eye(2) * 0.04 # Measurement noise is set as a matrix with 0.04 (from the task description) on the diagonal. 
        self.I = np.eye(4)

    def reset(self, measurement):
    # The initial state is set to a zero vector
        self.x = np.zeros(4)
    # The first two values of the state are set to the measurements
        self.x[:2] = measurement[:2]  # Set position to the measurement
    # Uncertainty is reset
        self.P = np.eye(4)  # Reset uncertainty covariance
        return self.x[:2]

    def update(self, dt, measurement):
    # State transition matrix for constant velocity
        self.F = np.eye(4)  # State transition matrix
        self.F[0, 2] = dt  # Position x changes with velocity vx
        self.F[1, 3] = dt  # Position y changes with velocity vy

    # Prediction: Update the state and uncertainty based on the model
        self.x = self.F @ self.x  # Predicted state estimate
        self.P = self.F @ self.P @ self.F.T  # Predicted uncertainty covariance

    # Measurement matrix that maps the state to the measurement
        H = np.eye(2, 4)

    # Correction step: Compute the difference between measurement and prediction
    # Actual measurement
        z = measurement[:2]
    # Measurement deviation (residual)
        y = z - H @ self.x

    # Compute the Kalman gain
        S = H @ self.P @ H.T + self.R  # Innovation covariance
    # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

    # Update the state based on the measurement deviation and Kalman gain
        self.x = self.x + K @ y

    # Update the uncertainty based on the Kalman gain
        self.P = (self.I - K @ H) @ self.P
    # Return the estimated position (x, y)
        return self.x[:2]


# Problem 5
class KalmanFilterConstantVelocityMultiple:
    def __init__(self):
        # The initial state is set to a zero vector (position and velocity)
        # State vector [x, y, vx, vy]
        self.x = np.zeros(4)
        # Initial uncertainty is set to the identity matrix
        # Covariance matrix
        self.P = np.eye(4)
        # Identity matrix
        self.I = np.eye(4)

    def reset(self, measurement):
        # The initial state is set to a zero vector
        self.x = np.zeros(4)
        # The first two values of the state are set to the average position of the first 10 measurements
        self.x[:2] = np.mean(
            measurement[:10].reshape(5, 2), axis=0
        )  # Initialize with the average position
        # Uncertainty matrix is reset
        self.P = np.eye(4)
        return self.x[:2]

    def update(self, dt, measurement):
        # Extract the five position measurements and the standard deviations
        z = measurement[:10].reshape(5, 2)  # Extract the five position measurements
        R_values = measurement[10:]  # Extract the standard deviations
        # Create the diagonal matrix of measurement noise covariances
        R = np.diag(
            R_values
        )  # Create the diagonal matrix of measurement noise covariances

        # State transition matrix for constant velocity
        self.F = np.eye(4)  # State transition matrix
        self.F[0, 2] = dt  # Position x changes with velocity vx
        self.F[1, 3] = dt  # Position y changes with velocity vy

        # Prediction: Update the state and uncertainty based on the model
        self.x = self.F @ self.x  # Predicted state estimate
        self.P = self.F @ self.P @ self.F.T  # Predicted covariance matrix

        # Calculation of the residuals and the Jacobian matrix
        H = np.zeros((10, 4))
        for i in range(5):
            H[2 * i : 2 * i + 2, :2] = np.eye(2)  # Measurement matrix

        # Correction: Compute the average position and the residuals
        z_mean = np.mean(z, axis=0)  # Average position
        y = (
            z_mean - self.x[:2]
        )  # Residual based on the average position

        # Scale the residuals according to the standard deviations
        y = np.concatenate([y for _ in range(5)])

        # Compute the Kalman gain
        S = H @ self.P @ H.T + R  # Innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain

        # Update the state based on the residuals and the Kalman gain
        self.x = self.x + K @ y  # Updated state estimate

        # Update the uncertainty based on the Kalman gain
        self.P = (self.I - K @ H) @ self.P  # Updated covariance matrix

        return self.x[:2]  # Return the estimated position (x, y)



# Problem 6
class ConstantTurnFilter:
    def __init__(self, turn_rate=0, initial_uncertainty=500, process_noise_std=0.01):
        self.turn_rate = turn_rate  # Estimated turn rate
        self.process_noise_std = process_noise_std
        self.initial_uncertainty = initial_uncertainty
        self.Q = np.diag(
            [
                process_noise_std**2,
                process_noise_std**2,
                (process_noise_std * 10) ** 2,
                (process_noise_std * 10) ** 2,
            ]
        )  # scaled Process noise Covar Matrix Q
        self.state = np.zeros(4)  # Initial state (x, y, vx, vy)
        self.uncertainty = (
            np.eye(4) * initial_uncertainty
        )  # Initial Covar Matrix P, that we later gonna update with our prediction error

    def reset(self, measurement):
        self.state[:2] = measurement[:2]
        self.state[2:] = 0  # Initial velocities are set to 0
        self.uncertainty = np.diag([10, 10, 1, 1])  # Adjusted initial covariance
        return self.state[:2]  # Return only the position (x, y)

    def update(self, dt, measurements, turn_rate=None):
        # Extract measurements and their uncertainties
        if turn_rate is not None:
            self.turn_rate = turn_rate
        else:
            # Estimate turn rate from current state
            v = np.linalg.norm(self.state[2:])
            if v > 0.1:
                self.turn_rate = (
                    self.state[2] * -self.state[3] + self.state[3] * self.state[2]
                ) / (v**2)

        # Calculating the Jocobi state transition matrix F. It calculates the new state based on the old state and turn rate.
        a_cos = np.cos(self.turn_rate * dt)
        a_sin = np.sin(self.turn_rate * dt)

        F = np.array(
            [
                [1, 0, dt * a_cos, -dt * a_sin],
                [0, 1, dt * a_sin, dt * a_cos],
                [0, 0, a_cos, -a_sin],
                [0, 0, a_sin, a_cos],
            ]
        )

        # Predict step for the estimated state and the uncertainty Covar Matrix P
        self.state = F @ self.state
        self.uncertainty = (
            F @ self.uncertainty @ F.T + self.Q * dt
        )  # we scale the process noise Matrix Q by dt

        # Extract measurements and their uncertainties
        H = np.zeros((10, 4))
        H[0::2, 0] = 1  # x measurements
        H[1::2, 1] = 1  # y measurements

        z = measurements[:10]  # First 10 elements are measurements
        R = np.diag(measurements[10:] ** 2)  # Last 10 elements are std deviations

        # Kalman gain
        innovation = z - H @ self.state
        S = H @ self.uncertainty @ H.T + R
        K = self.uncertainty @ H.T @ np.linalg.inv(S)

        # Update step
        self.state = self.state + K @ innovation
        self.uncertainty = (np.eye(4) - K @ H) @ self.uncertainty
        return self.state[:2]  # Return only the position (x, y)
