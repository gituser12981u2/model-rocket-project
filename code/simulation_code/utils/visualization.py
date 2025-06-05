"""
Visualization utilities for rocket simulation results
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Dict, Any, cast


class RocketPlotter:
    """
    Plotting utilities for rocket simulation visualization
    """

    def plot_comprehensive_results(
        self,
        times: np.ndarray,
        states: np.ndarray,
        envelope: Dict[str, np.ndarray],
        motor_specs: Dict[str, Any],
    ):
        """
        Plot clean basic simulation results with proper spacing

        Parameters:
        -----------
        times : np.ndarray
            Time array
        states : np.ndarray
            State array [N_points, 13]
        envelope : dict
            Performance envelope from FlightAnalyzer
        motor_specs : dict
            Motor specifications
        """
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))

        # Extract data
        altitudes = states[:, 2]
        velocities = envelope["velocities"]
        accelerations = envelope["accelerations"]

        # 1. Altitude vs Time
        ax = axes[0]
        ax.plot(times, altitudes, "b-", linewidth=2)
        apogee_idx = np.argmax(altitudes)
        ax.plot(
            times[apogee_idx],
            altitudes[apogee_idx],
            "ro",
            markersize=8,
            label=f"Apogee: {altitudes[apogee_idx]:.1f} m",
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Altitude (m)")
        ax.set_title("Altitude vs Time")
        ax.grid(True, alpha=0.3)
        ax.legend()

        # 2. Velocity vs Time
        ax = axes[1]
        ax.plot(times, velocities, "g-", linewidth=2)
        max_vel_idx = np.argmax(velocities)
        ax.plot(
            times[max_vel_idx],
            velocities[max_vel_idx],
            "ro",
            markersize=8,
            label=f"Max: {velocities[max_vel_idx]:.1f} m/s",
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.set_title("Velocity vs Time")
        ax.grid(True, alpha=0.3)
        ax.legend()

        # 3. Acceleration vs Time
        ax = axes[2]
        ax.plot(times, accelerations, "r-", linewidth=2)
        ax.axhline(y=9.81, color="gray", linestyle="--", alpha=0.7, label="1g")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Acceleration (m/s²)")
        ax.set_title("Acceleration vs Time")
        ax.grid(True, alpha=0.3)
        ax.legend()

        plt.suptitle(
            f"Flight Analysis - {motor_specs['designation']}", fontsize=16, y=0.98)
        plt.tight_layout(rect=[0, 0.03, 1, 0.93])
        plt.show()

    def plot_trajectory_3d(self, states: np.ndarray, title: str = "Rocket Trajectory"):
        """
        Plot 3D trajectory

        Parameters:
        -----------
        states : np.ndarray
            State array
        title : str
            Plot title
        """
        fig = plt.figure(figsize=(10, 8))
        ax = cast(Axes3D, fig.add_subplot(111, projection="3d"))

        # Extract positions
        x, y, z = states[:, 0], states[:, 1], states[:, 2]

        # Plot trajectory
        ax.plot(x, y, z, "b-", linewidth=2)

        # Mark key points
        ax.scatter(x[0], y[0], z[0], color="green", label="Launch")

        apogee_idx = np.argmax(z)
        ax.scatter(
            x[apogee_idx],
            y[apogee_idx],
            z[apogee_idx],
            color="red",
            label=f"Apogee: {z[apogee_idx]:.1f} m",
        )

        if len(states) > 1:
            ax.scatter(x[-1], y[-1], z[-1], color="orange", label="Landing")

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Altitude (m)")
        ax.set_title(title)
        ax.legend()

        plt.tight_layout()
        plt.show()

    def plot_time_histories(self, times: np.ndarray, states: np.ndarray):
        """
        Plot basic time histories

        Parameters:
        -----------
        times : np.ndarray
            Time array
        states : np.ndarray
            State array
        """
        fig, axs = plt.subplots(2, 3, figsize=(15, 10))

        # Position components
        axs[0, 0].plot(times, states[:, 0], label="X")
        axs[0, 0].plot(times, states[:, 1], label="Y")
        axs[0, 0].plot(times, states[:, 2], label="Z")
        axs[0, 0].set_xlabel("Time (s)")
        axs[0, 0].set_ylabel("Position (m)")
        axs[0, 0].set_title("Position vs Time")
        axs[0, 0].legend()
        axs[0, 0].grid(True, alpha=0.3)

        # Velocity components
        axs[0, 1].plot(times, states[:, 3], label="Vx")
        axs[0, 1].plot(times, states[:, 4], label="Vy")
        axs[0, 1].plot(times, states[:, 5], label="Vz")
        axs[0, 1].set_xlabel("Time (s)")
        axs[0, 1].set_ylabel("Velocity (m/s)")
        axs[0, 1].set_title("Velocity vs Time")
        axs[0, 1].legend()
        axs[0, 1].grid(True, alpha=0.3)

        # Quaternion components
        axs[0, 2].plot(times, states[:, 6], label="q0")
        axs[0, 2].plot(times, states[:, 7], label="q1")
        axs[0, 2].plot(times, states[:, 8], label="q2")
        axs[0, 2].plot(times, states[:, 9], label="q3")
        axs[0, 2].set_xlabel("Time (s)")
        axs[0, 2].set_ylabel("Quaternion")
        axs[0, 2].set_title("Attitude vs Time")
        axs[0, 2].legend()
        axs[0, 2].grid(True, alpha=0.3)

        # Angular velocity components
        axs[1, 0].plot(times, states[:, 10], label="ωx")
        axs[1, 0].plot(times, states[:, 11], label="ωy")
        axs[1, 0].plot(times, states[:, 12], label="ωz")
        axs[1, 0].set_xlabel("Time (s)")
        axs[1, 0].set_ylabel("Angular Velocity (rad/s)")
        axs[1, 0].set_title("Angular Velocity vs Time")
        axs[1, 0].legend()
        axs[1, 0].grid(True, alpha=0.3)

        # Altitude
        axs[1, 1].plot(times, states[:, 2], "b-", linewidth=2)
        apogee_idx = np.argmax(states[:, 2])
        axs[1, 1].plot(
            times[apogee_idx],
            states[apogee_idx, 2],
            "ro",
            markersize=8,
            label=f"Apogee: {states[apogee_idx, 2]:.1f} m",
        )
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].set_ylabel("Altitude (m)")
        axs[1, 1].set_title("Altitude vs Time")
        axs[1, 1].legend()
        axs[1, 1].grid(True, alpha=0.3)

        # Velocity magnitude
        vel_mag = np.sqrt(states[:, 3] ** 2 +
                          states[:, 4] ** 2 + states[:, 5] ** 2)
        axs[1, 2].plot(times, vel_mag, "g-", linewidth=2)
        max_vel_idx = np.argmax(vel_mag)
        axs[1, 2].plot(
            times[max_vel_idx],
            vel_mag[max_vel_idx],
            "ro",
            markersize=8,
            label=f"Max: {vel_mag[max_vel_idx]:.1f} m/s",
        )
        axs[1, 2].set_xlabel("Time (s)")
        axs[1, 2].set_ylabel("Velocity Magnitude (m/s)")
        axs[1, 2].set_title("Velocity Magnitude vs Time")
        axs[1, 2].legend()
        axs[1, 2].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()
