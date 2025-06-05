"""
Flight metrics calculation and analysis
"""

import numpy as np
from typing import Dict
from dataclasses import dataclass
from models.atmosphere import StandardAtmosphere


@dataclass
class FlightMetrics:
    """Container for flight performance metrics"""

    # Basic metrics
    apogee: float  # m
    apogee_time: float  # s
    flight_time: float  # s
    max_velocity: float  # m/s
    max_velocity_time: float  # s
    max_acceleration: float  # m/s²
    max_acceleration_time: float  # s
    max_mach: float
    max_mach_time: float  # s

    # Advanced metrics
    max_dynamic_pressure: float  # Pa
    max_dynamic_pressure_time: float  # s
    burnout_velocity: float  # m/s
    burnout_altitude: float  # m
    coast_time: float  # s (burnout to apogee)

    # Performance ratios
    velocity_at_burnout: float  # m/s
    altitude_at_burnout: float  # m
    thrust_to_weight_max: float

    def __str__(self) -> str:
        """String representation of flight metrics"""
        return f"""Flight Performance Metrics:
  Apogee: {self.apogee:.1f} m ({self.apogee/0.3048:.0f} ft) at t={
            self.apogee_time:.2f} s
  Max Velocity: {self.max_velocity:.1f} m/s at t={self.max_velocity_time:.2f} s
  Max Acceleration: {self.max_acceleration:.1f} m/s² ({
            self.max_acceleration/9.81:.1f} g) at t={self.max_acceleration_time:.2f} s
  Max Mach: {self.max_mach:.3f} at t={self.max_mach_time:.2f} s
  Max Dynamic Pressure: {self.max_dynamic_pressure:.0f} Pa at t={
            self.max_dynamic_pressure_time:.2f} s
  Burnout Conditions: {self.velocity_at_burnout:.1f} m/s at {
            self.altitude_at_burnout:.1f} m
  Coast Time: {self.coast_time:.2f} s
  Total Flight Time: {self.flight_time:.1f} s
  Max Thrust-to-Weight: {self.thrust_to_weight_max:.2f}"""


class FlightAnalyzer:
    """
    Analyzes simulation results and calculates flight metrics
    """

    def __init__(self):
        self.atmosphere = StandardAtmosphere()

    def analyze_flight(
        self, times: np.ndarray, states: np.ndarray, motor, rocket_params: Dict
    ) -> FlightMetrics:
        """
        Analyze complete flight data and calculate metrics

        Parameters:
        -----------
        times : np.ndarray
            Time array
        states : np.ndarray
            State array [N_points, 13] where columns are:
            [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
        motor : ThrustCurveMotor
            Motor object for thrust/mass data
        rocket_params : dict
            Rocket parameters

        Returns:
        --------
        FlightMetrics
            Complete flight performance metrics
        """
        # Extract basic trajectory data
        positions = states[:, 0:3]
        velocities = states[:, 3:6]
        altitudes = positions[:, 2]
        velocity_magnitudes = np.linalg.norm(velocities, axis=1)

        # Calculate accelerations
        accelerations = self._calculate_accelerations(times, velocity_magnitudes)

        # Basic metrics
        apogee_idx = np.argmax(altitudes)
        apogee = altitudes[apogee_idx]
        apogee_time = times[apogee_idx]
        flight_time = times[-1]

        max_vel_idx = np.argmax(velocity_magnitudes)
        max_velocity = velocity_magnitudes[max_vel_idx]
        max_velocity_time = times[max_vel_idx]

        max_acc_idx = np.argmax(np.abs(accelerations))
        max_acceleration = np.abs(accelerations[max_acc_idx])
        max_acceleration_time = times[max_acc_idx]

        # Calculate Mach numbers
        mach_numbers = self._calculate_mach_numbers(times, states, velocity_magnitudes)
        max_mach_idx = np.argmax(mach_numbers)
        max_mach = mach_numbers[max_mach_idx]
        max_mach_time = times[max_mach_idx]

        # Calculate dynamic pressures
        dynamic_pressures = self._calculate_dynamic_pressures(
            altitudes, velocity_magnitudes
        )
        max_q_idx = np.argmax(dynamic_pressures)
        max_dynamic_pressure = dynamic_pressures[max_q_idx]
        max_dynamic_pressure_time = times[max_q_idx]

        # Burnout conditions
        motor_specs = motor.get_motor_specs()
        burn_time = motor_specs["burn_time"]
        burnout_idx = np.argmin(np.abs(times - burn_time))
        burnout_velocity = velocity_magnitudes[burnout_idx]
        burnout_altitude = altitudes[burnout_idx]
        coast_time = apogee_time - burn_time

        # Thrust-to-weight ratio
        thrust_to_weight_max = self._calculate_max_thrust_to_weight(
            motor, rocket_params
        )

        return FlightMetrics(
            apogee=apogee,
            apogee_time=apogee_time,
            flight_time=flight_time,
            max_velocity=max_velocity,
            max_velocity_time=max_velocity_time,
            max_acceleration=max_acceleration,
            max_acceleration_time=max_acceleration_time,
            max_mach=max_mach,
            max_mach_time=max_mach_time,
            max_dynamic_pressure=max_dynamic_pressure,
            max_dynamic_pressure_time=max_dynamic_pressure_time,
            burnout_velocity=burnout_velocity,
            burnout_altitude=burnout_altitude,
            coast_time=coast_time,
            velocity_at_burnout=burnout_velocity,
            altitude_at_burnout=burnout_altitude,
            thrust_to_weight_max=thrust_to_weight_max,
        )

    def _calculate_accelerations(
        self, times: np.ndarray, velocities: np.ndarray
    ) -> np.ndarray:
        """Calculate acceleration from velocity time series"""
        if len(times) < 2:
            # Not enough points for gradient calculation
            return np.array([0.0] * len(times))
        return np.gradient(velocities, times)

    def _calculate_mach_numbers(
        self, times: np.ndarray, states: np.ndarray, velocity_magnitudes: np.ndarray
    ) -> np.ndarray:
        """Calculate Mach number time series"""
        mach_numbers = []
        for i, velocity in enumerate(velocity_magnitudes):
            altitude = states[i, 2]
            sound_speed = self.atmosphere.sound_speed(altitude)
            mach = velocity / sound_speed if sound_speed > 0 else 0
            mach_numbers.append(mach)
        return np.array(mach_numbers)

    def _calculate_dynamic_pressures(
        self, altitudes: np.ndarray, velocity_magnitudes: np.ndarray
    ) -> np.ndarray:
        """Calculate dynamic pressure time series"""
        dynamic_pressures = []
        for altitude, velocity in zip(altitudes, velocity_magnitudes):
            rho = self.atmosphere.density(altitude)
            q = 0.5 * rho * velocity**2
            dynamic_pressures.append(q)
        return np.array(dynamic_pressures)

    def _calculate_max_thrust_to_weight(self, motor, rocket_params: Dict) -> float:
        """Calculate maximum thrust-to-weight ratio"""
        motor_specs = motor.get_motor_specs()
        peak_thrust = motor_specs["peak_thrust"]
        min_mass = rocket_params["dry_mass"] + motor_specs["total_mass"]
        max_thrust_to_weight = peak_thrust / (min_mass * 9.81)
        return max_thrust_to_weight

    def calculate_performance_envelope(
        self, times: np.ndarray, states: np.ndarray, motor
    ) -> Dict[str, np.ndarray]:
        """
        Calculate time-varying performance parameters

        Returns:
        --------
        dict
            Dictionary containing time series of various performance metrics
        """
        positions = states[:, 0:3]
        velocities = states[:, 3:6]
        altitudes = positions[:, 2]
        velocity_magnitudes = np.linalg.norm(velocities, axis=1)

        # Calculate time-varying parameters
        mach_numbers = self._calculate_mach_numbers(times, states, velocity_magnitudes)
        dynamic_pressures = self._calculate_dynamic_pressures(
            altitudes, velocity_magnitudes
        )
        accelerations = self._calculate_accelerations(times, velocity_magnitudes)

        # Thrust and mass time series
        thrust_values = np.array([motor.thrust(t) for t in times])
        mass_values = np.array([motor.mass(t) for t in times])
        thrust_to_weight = thrust_values / (mass_values * 9.81)

        # Air density
        air_densities = np.array([self.atmosphere.density(alt) for alt in altitudes])

        return {
            "times": times,
            "altitudes": altitudes,
            "velocities": velocity_magnitudes,
            "accelerations": accelerations,
            "mach_numbers": mach_numbers,
            "dynamic_pressures": dynamic_pressures,
            "thrust_values": thrust_values,
            "mass_values": mass_values,
            "thrust_to_weight": thrust_to_weight,
            "air_densities": air_densities,
        }

    def generate_flight_report(
        self, metrics: FlightMetrics, motor_specs: Dict, rocket_params: Dict
    ) -> str:
        """
        Generate a comprehensive flight performance report

        Parameters:
        -----------
        metrics : FlightMetrics
            Calculated flight metrics
        motor_specs : dict
            Motor specifications
        rocket_params : dict
            Rocket parameters

        Returns:
        --------
        str
            Formatted flight report
        """
        report = f"""
{'='*60}
ROCKET FLIGHT SIMULATION REPORT
{'='*60}

MOTOR INFORMATION:
  Designation: {motor_specs['designation']} ({motor_specs['manufacturer']})
  Impulse Class: {motor_specs['impulse_class']}
  Total Impulse: {motor_specs['total_impulse']:.1f} N⋅s
  Average Thrust: {motor_specs['average_thrust']:.1f} N
  Peak Thrust: {motor_specs['peak_thrust']:.1f} N
  Burn Time: {motor_specs['burn_time']:.2f} s
  Propellant Mass: {motor_specs['propellant_mass']*1000:.1f} g
  Specific Impulse: {motor_specs['specific_impulse']:.0f} s

ROCKET CONFIGURATION:
  Dry Mass: {rocket_params['dry_mass']*1000:.0f} g
  Total Mass (loaded): {(rocket_params['dry_mass'] +
                         motor_specs['total_mass'])*1000:.0f} g
  Reference Area: {rocket_params['reference_area']*10000:.1f} cm²
  Mass Ratio: {(rocket_params['dry_mass'] + motor_specs['total_mass'])
               / rocket_params['dry_mass']:.2f}

FLIGHT PERFORMANCE:
{str(metrics)}

SAFETY ANALYSIS:
  Max G-Force: {metrics.max_acceleration/9.81:.1f} g
  Max Dynamic Pressure: {metrics.max_dynamic_pressure:.0f} Pa
  Max Mach Number: {metrics.max_mach:.3f}

EFFICIENCY METRICS:
  Velocity Efficiency: {metrics.velocity_at_burnout/motor_specs['specific_impulse']:.3f}
  Altitude Efficiency: {
            metrics.apogee/(motor_specs['total_impulse']*motor_specs['specific_impulse']/9.81):.3f}
  Coast Ratio: {metrics.coast_time/motor_specs['burn_time']:.2f}

{'='*60}
"""
        return report
