import numpy as np


class GravityModel:
    """Models for gravitational acceleration"""

    def __init__(self, model_type="constant"):
        """Initialize gravity model

        model_type : str
            'constant', 'inverse_square'
        """
        self.model_type = model_type
        self.G = 6.67430e-11  # Gravitational constant, m³/(kg·s²)
        self.M = 5.972e24  # Earth mass, kg
        self.R = 6371000  # Earth mean radius, m
        self.g0 = 9.80665  # Standard gravity at sea level, m/s²

    def acceleration(self, position):
        """Calculate gravitational acceleration at given position"""
        if self.model_type == "constant":
            return np.array([0.0, 0.0, -self.g0])

        elif self.model_type == "inverse_square":
            # Position is [x, y, z] where z is altitude
            altitude = position[2]
            # g = G*M/(R+h)^2
            g_mag = self.G * self.M / ((self.R + altitude) ** 2)
            return np.array([0.0, 0.0, -g_mag])
