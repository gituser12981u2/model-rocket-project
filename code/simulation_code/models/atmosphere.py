import numpy as np


class StandardAtmosphere:
    """US Standard Atmosphere 1976 model"""

    def __init__(self):
        # Constants
        self.R = 287.058  # Gas constant for air, J/(kg*K)
        self.gamma = 1.4  # Ratio of specific heats

    def density(self, altitude):
        """Calculate air density at given altitude."""
        if altitude < 11000:  # Troposphere
            T = 288.15 - 0.00649 * altitude
            p = 101325 * (T / 288.15) ** 5.256
        elif altitude < 20000:  # Lower stratosphere
            T = 216.65
            p = 22632 * np.exp(-0.00015769 * (altitude - 11000))
        else:  # Upper Stratosphere
            T = 216.65 + 0.001 * (altitude - 20000)
            p = 5474.9 * (T / 216.65) ** (-34.163)

        # Ideal gas law
        rho = p / (self.R * T)
        return rho

    def sound_speed(self, altitude):
        """Calculate speed of sound at given altitude"""
        if altitude < 11000:  # Troposphere
            T = 288.15 - 0.00649 * altitude
        elif altitude < 20000:  # Lower Stratosphere
            T = 216.65
        else:  # Upper Stratosphere
            T = 216.65 + 0.001 * (altitude - 20000)

        # Speed of sound formula
        c = np.sqrt(self.gamma * self.R * T)
        return c
