import rasp_parser
import numpy as np
import matplotlib.pyplot as plt


class ThrustCurveMotor:
    """Model rocket motors thrust profiles"""

    def __init__(self, motor_file_path: str):
        """
        Initialize motor from RASP file

        Parameters:
            motor_file_path : str
                Path to .eng file (e.g., 'C6-5.eng')
        """
        self.motor = rasp_parser.load_rasp_motor(motor_file_path)

        # Validate the motor data
        warnings = rasp_parser.validate_motor(self.motor)
        if warnings:
            print(f"Motor validation warnings for {self.motor.designation}:")
            for warning in warnings:
                print(f" {warning}")

        print(f"Loaded motor: {self.motor}")

        # Calculate exhaust velocity from specific impulse
        self._exhaust_velocity = (
            self.motor.specific_impulse * 9.81
            if self.motor.specific_impulse > 0
            else 2000.0
        )

    def thrust(self, t: float) -> float:
        """Get thrust at time t using interpolation"""
        return self.motor.get_interpolated_thrust(t)

    def mass(self, t: float) -> float:
        """Get total motor mass at time t"""
        if t <= 0:
            return self.motor.total_mass
        elif t >= self.motor.burn_time:
            return self.motor.total_mass - self.motor.propellant_mass
        else:
            # Calculate mass based on propellant consumption
            # Integrate mass flow rate from 0 to t
            propellant_consumed = 0.0
            dt = 0.01
            for time_step in np.arange(0, min(t, self.motor.burn_time), dt):
                thrust_val = self.thrust(float(time_step))
                mass_flow_rate = thrust_val / self._exhaust_velocity
                propellant_consumed += mass_flow_rate * dt

            remaining_mass = self.motor.total_mass - min(
                propellant_consumed, self.motor.propellant_mass
            )
            return max(
                remaining_mass, self.motor.total_mass - self.motor.propellant_mass
            )

    def mass_flow_rate(self, t: float) -> float:
        """Get instantaneous mass flow rate"""
        if 0 <= t <= self.motor.burn_time:
            return -self.thrust(t) / self._exhaust_velocity
        return 0.0

    def get_motor_specs(self) -> dict:
        """Get motor specifications"""
        return {
            "designation": self.motor.designation,
            "manufacturer": self.motor.manufacturer,
            "diameter": self.motor.diameter,
            "length": self.motor.length,
            "total_impulse": self.motor.total_impulse,
            "peak_thrust": self.motor.peak_thrust,
            "average_thrust": self.motor.average_thrust,
            "burn_time": self.motor.burn_time,
            "propellant_mass": self.motor.propellant_mass,
            "total_mass": self.motor.total_mass,
            "specific_impulse": self.motor.specific_impulse,
            "impulse_class": self.motor.impulse_class,
        }

    def plot_motor_data(self):
        """Plot comprehensive motor data"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

        # Create high-resolution time array
        t_plot = np.linspace(0, self.motor.burn_time * 1.1, 1000)

        # Thrust vs time
        thrust_plot = [self.thrust(t) for t in t_plot]
        ax1.plot(t_plot, thrust_plot, "b-", linewidth=2, label="Interpolated")

        # Plot original data points
        original_times = [point.time for point in self.motor.thrust_curve]
        original_thrusts = [point.thrust for point in self.motor.thrust_curve]
        ax1.scatter(
            original_times,
            original_thrusts,
            color="red",
            s=20,
            zorder=5,
            label="Data points",
        )

        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Thrust (N)")
        ax1.set_title(f"{self.motor.designation} Thrust Curve")
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Add statistics
        specs = self.get_motor_specs()
        stats_text = f"Total Impulse: {specs['total_impulse']:.1f} N⋅s\n"
        stats_text += f"Average Thrust: {specs['average_thrust']:.1f} N\n"
        stats_text += f"Peak Thrust: {specs['peak_thrust']:.1f} N\n"
        stats_text += f"Burn Time: {specs['burn_time']:.2f} s\n"
        stats_text += f"Specific Impulse: {specs['specific_impulse']:.0f} s"
        ax1.text(
            0.02,
            0.98,
            stats_text,
            transform=ax1.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
        )

        # Mass vs time
        mass_plot = [self.mass(t) * 1000 for t in t_plot]  # Convert to grams
        ax2.plot(t_plot, mass_plot, "g-", linewidth=2, label="Total mass")

        propellant_plot = [
            (self.mass(t) - (specs["total_mass"] - specs["propellant_mass"])) * 1000
            for t in t_plot
        ]
        ax2.plot(
            t_plot, propellant_plot, "orange", linewidth=2, label="Propellant mass"
        )

        ax2.axhline(
            y=(specs["total_mass"] - specs["propellant_mass"]) * 1000,
            color="gray",
            linestyle="--",
            alpha=0.7,
            label="Dry mass",
        )
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Mass (g)")
        ax2.set_title("Mass vs Time")
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # Mass flow rate vs time
        mass_flow_plot = [self.mass_flow_rate(t) * 1000 for t in t_plot]  # g/s
        ax3.plot(t_plot, mass_flow_plot, "purple", linewidth=2)
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Mass Flow Rate (g/s)")
        ax3.set_title("Mass Flow Rate vs Time")
        ax3.grid(True, alpha=0.3)

        # Impulse vs time (cumulative)
        impulse_plot = []
        cumulative_impulse = 0.0
        dt = t_plot[1] - t_plot[0]
        for thrust_val in thrust_plot:
            cumulative_impulse += thrust_val * dt
            impulse_plot.append(cumulative_impulse)

        ax4.plot(t_plot, impulse_plot, "red", linewidth=2)
        ax4.axhline(
            y=specs["total_impulse"],
            color="black",
            linestyle="--",
            alpha=0.7,
            label=f"Total: {specs['total_impulse']:.1f} N⋅s",
        )
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Cumulative Impulse (N⋅s)")
        ax4.set_title("Impulse vs Time")
        ax4.grid(True, alpha=0.3)
        ax4.legend()

        plt.tight_layout()
        plt.show()
