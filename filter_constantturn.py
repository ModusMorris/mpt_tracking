import numpy as np

class KalmanFilterConstantTurn:
    def __init__(self):
        self.x = np.zeros(5)  # Zustandsvektor [x, y, vx, vy, turn_rate]
        self.P = np.eye(5)  # Kovarianzmatrix
        self.I = np.eye(5)  # Einheitsmatrix

    def reset(self, measurement):
        self.x = np.zeros(5)
        self.x[:2] = np.mean(measurement[:10].reshape(5, 2), axis=0)  # Durchschnittliche Position als Initialisierung
        self.P = np.eye(5)
        return self.x[:2]

    def update(self, dt, measurement):
        z = measurement[:10].reshape(5, 2)  # Extrahiere die fünf Positionsmessungen
        R_values = measurement[10:]  # Extrahiere die Standardabweichungen
        R = np.diag(R_values)  # Erstelle die Diagonalmatrix der Messrauschkovarianzen

        turn_rate = self.x[4]

        # Zustandsübergangsmatrix
        cos_a_dt = np.cos(turn_rate * dt)
        sin_a_dt = np.sin(turn_rate * dt)
        self.F = np.array([[1, 0, dt, 0, 0],
                           [0, 1, 0, dt, 0],
                           [0, 0, cos_a_dt, -sin_a_dt, 0],
                           [0, 0, sin_a_dt, cos_a_dt, 0],
                           [0, 0, 0, 0, 1]])

        # Vorhersage
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T

        # Berechnung der Residuen und Jacobian der Messmatrix
        H = np.zeros((10, 5))
        for i in range(5):
            H[2*i:2*i+2, :2] = np.eye(2)

        # Korrektur
        z_mean = np.mean(z, axis=0)  # Durchschnittliche Position
        y = z_mean - self.x[:2]  # Residuum basierend auf der durchschnittlichen Position

        # Skalieren der Residuen entsprechend der Standardabweichungen
        y = np.concatenate([y for _ in range(5)])

        # Kalman-Gewinn
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Aktualisieren der Zustandsvektorschätzung
        self.x = self.x + K @ y
        self.P = (self.I - K @ H) @ self.P

        return self.x[:2]
