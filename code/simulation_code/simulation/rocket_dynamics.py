import numpy as np
from typing import Dict, Any
from scipy.integrate import solve_ivp

from models.atmosphere import StandardAtmosphere
from models.gravity import GravityModel
from models.thrust_curve_motor import ThrustCurveMotor
from simulation.quaternion import quaternion_derivative, quaternion_to_dcm


class RocketDynamics:
    """Handles all physics calculations for simulation"""

    def __init__(self, rocket_params: Dict[str, Any], motor: ThrustCurveMotor):
        """
        Initialize the rocket simulation with initial state and parameters.

        Parameters:
        -----------
        rocket_params : dict
            Rocket parameters (mass, aerodynamics, inertia, etc.)
        motor : ThrustCurveMotor
            Motor object with thrust curve data
        """
        self.params = rocket_params
        self.motor = motor

        # Initialize models
        self.atmosphere = StandardAtmosphere()
        self.gravity = GravityModel(model_type="inverse_square")

    def calculate_mass(self, t):
        """
        Calculate rocket mass at time t based on propellant consumption

        Parameters:
        -----------
        t : float
            Time in seconds

        Returns:
        --------
        float
            Total rocket mass in kg
        """
        motor_mass = self.motor.mass(t)
        return self.params["dry_mass"] + motor_mass

    def calculate_thrust_vector(self, t: float, quaternion: np.ndarray) -> np.ndarray:
        """
        Calculate thrust vector in world coordinates

        Parameters:
        -----------
        t : float
            Time in seconds
        quaternion : np.ndarray
            Current attitude quaternion [q0, q1, q2, q3]

        Returns:
        --------
        np.ndarray
            Thrust vector in world coordinates [Fx, Fy, Fz]
        """
        thrust_magnitude = self.motor.thrust(t)

        # Convert from body to world coordinates using current attitude
        dcm = quaternion_to_dcm(quaternion)

        # Assuming thrust is along body z-axis
        thrust_body = np.array([0.0, 0.0, thrust_magnitude])
        thrust_world = dcm @ thrust_body

        return thrust_world

    def calculate_aerodynamic_forces(self, position: np.ndarray, velocity: np.ndarray):
        """
        Calculate aerodynamic forces based on velocity, air density, and Mach number

        Parameters:
        -----------
        position : np.ndarray
            Position vector [x, y, z] in meters
        velocity : np.ndarray
            Velocity vector [vx, vy, vz] in m/s

        Returns:
        --------
        np.ndarray
            Aerodynamic force vector [Fx, Fy, Fz] in Newtons
        """
        v_mag = np.linalg.norm(velocity)
        if v_mag < 1e-6:
            return np.zeros(3)

        # Calculate atmospheric properties
        altitude = position[2]
        rho = self.atmosphere.density(altitude)
        sound_speed = self.atmosphere.sound_speed(altitude)
        mach = v_mag / sound_speed if sound_speed > 0 else 0.0

        # Calculate dynamic pressure
        q_dyn = 0.5 * rho * v_mag**2

        # Get drag coefficient based and reference area
        cd = self._calculate_drag_coefficient(mach)
        area = self.params["reference_area"]

        # Calculate drag force magnitude
        drag_mag = q_dyn * cd * area

        # Drag acts opposite to velocity direction
        drag_direction = -velocity / v_mag

        return drag_mag * drag_direction

    def _calculate_drag_coefficient(self, mach: float) -> float:
        """
        Calculate drag coefficient as a function of Mach number

        Parameters:
        -----------
        mach : float
            Mach number

        Returns:
        --------
        float
            Drag coefficient
        """
        if mach < 0.5:
            # Subsonic - relatively constant
            cd = 0.45
        elif mach < 0.8:
            # Transonic region - increasing drag
            cd = 0.45 + 0.3 * (mach - 0.5) / 0.3
        elif mach < 1.2:
            # Transonic/supersonic transition - drag peak
            cd = 0.75 + 0.5 * (mach - 0.8) / 0.4
        elif mach < 2.0:
            # Supersonic - decreasing drag
            cd = 1.25 - 0.4 * (mach - 1.2) / 0.8
        else:
            # High supersonic - more gradual decrease
            cd = 0.85 - 0.1 * min((mach - 2.0) / 2.0, 1.0)

        return cd

    def calculate_aerodynamic_moments(
        self, position: np.ndarray, velocity: np.ndarray, angular_velocity: np.ndarray
    ) -> np.ndarray:
        """
        Calculate aerodynamic moments about center of mass

        Parameters:
        -----------
        position : np.ndarray
            Position vector [x, y, z]
        velocity : np.ndarray
            Velocity vector [vx, vy, vz]
        angular_velocity : np.ndarray
            Angular velocity vector [wx, wy, wz]

        Returns:
        --------
        np.ndarray
            Moment vector [Mx, My, Mz] in N⋅m
        """
        # This would include calculations for aerodynamic moments,
        # control surface effects, thrust misalignment, etc.
        v_mag = np.linalg.norm(velocity)
        if v_mag < 1e-6:
            return np.zeros(3)

        # Simple aerodynamic damping model
        # More sophisticated models would include:
        # - Center of pressure effects
        # - Fin effectiveness
        # - Magnus effects
        # - Angle of attack dependencies
        # Current: just simple damping
        # Need to add: CP-CG offset effects, angle of attack forces, fin forces

        # Calculate normal force coefficient
        # Apply moment arm from CP-CG offset
        # Add fin restoring forces

        # TODO: Add more sophisticated moment calculations
        # - Normal force due to angle of attack
        # - Center of pressure vs center of mass offset
        # - Fin contributions

        # For a simple model, we can assume moments proportional to angular velocity (damping)
        damping = self.params.get("aero_damping", 0.01)
        return -damping * angular_velocity

    def calculate_gravitational_acceleration(self, position: np.ndarray) -> np.ndarray:
        """
        Calculate gravitational acceleration at given position

        Parameters:
        -----------
        position : np.ndarray
            Position vector [x, y, z]

        Returns:
        --------
        np.ndarray
            Gravitational acceleration vector [ax, ay, az]
        """
        return self.gravity.acceleration(position)

    def state_derivative(self, t: float, state: np.ndarray) -> np.ndarray:
        """
        Calculate state derivative for numerical integration

        Parameters:
        -----------
        t : float
            Current time
        state : np.ndarray
            State vector [x,y,z, vx,vy,vz, q0,q1,q2,q3, wx,wy,wz]

        Returns:
        --------
        np.ndarray
            State derivative vector
        """
        # Extract state variables
        position = state[0:3]
        velocity = state[3:6]
        quaternion = state[6:10]
        angular_velocity = state[10:13]

        # Normalize quaternion to prevent numerical drift
        quaternion = quaternion / np.linalg.norm(quaternion)

        # Calculate current mass and mass flow rate
        mass = self.calculate_mass(t)
        mass_flow_rate = self.motor.mass_flow_rate(t)

        # Calculate all forces
        gravitational_force = mass * \
            self.calculate_gravitational_acceleration(position)
        thrust_force = self.calculate_thrust_vector(t, quaternion)
        aerodynamic_force = self.calculate_aerodynamic_forces(
            position, velocity)

        # Total force
        total_force = thrust_force + aerodynamic_force + gravitational_force

        # Translational dynamics
        acceleration = total_force / mass - (velocity / mass) * mass_flow_rate

        # Calculate moments
        aerodynamic_moments = self.calculate_aerodynamic_moments(
            position, velocity, angular_velocity
        )

        total_moments = aerodynamic_moments

        # Rotational dynamics
        inertia = np.array(self.params["inertia"])  # [Ixx, Iyy, Izz]

        angular_acceleration = np.zeros(3)
        angular_acceleration[0] = (
            total_moments[0]
            + (inertia[1] - inertia[2]) *
            angular_velocity[1] * angular_velocity[2]
        ) / inertia[0]
        angular_acceleration[1] = (
            total_moments[1]
            + (inertia[2] - inertia[0]) *
            angular_velocity[2] * angular_velocity[0]
        ) / inertia[1]
        angular_acceleration[2] = (
            total_moments[2]
            + (inertia[0] - inertia[1]) *
            angular_velocity[0] * angular_velocity[1]
        ) / inertia[2]

        # Quaternion derivative
        quaternion_dot = quaternion_derivative(quaternion, angular_velocity)

        # Assemble state derivative
        state_derivative = np.concatenate(
            [
                velocity,  # position derivative
                acceleration,  # velocity derivative
                quaternion_dot,  # quaternion derivative
                angular_acceleration,  # angular velocity derivative
            ]
        )

        return state_derivative

    def get_current_flight_conditions(
        self, t: float, state: np.ndarray
    ) -> Dict[str, float]:
        """
        Get current flight conditions for analysis

        Parameters:
        -----------
        t : float
            Current time
        state : np.ndarray
            Current state vector

        Returns:
        --------
        dict
            Flight conditions including Mach number, dynamic pressure, etc.
        """
        position = state[0:3]
        velocity = state[3:6]

        altitude = position[2]
        velocity_magnitude = np.linalg.norm(velocity)

        # Atmospheric conditions
        air_density = self.atmosphere.density(altitude)
        sound_speed = self.atmosphere.sound_speed(altitude)

        # Flight conditions
        mach_number = velocity_magnitude / sound_speed if sound_speed > 0 else 0
        dynamic_pressure = 0.5 * air_density * velocity_magnitude**2

        # Forces
        thrust = self.motor.thrust(t)
        mass = self.calculate_mass(t)

        return {
            "time": t,
            "altitude": altitude,
            "velocity_magnitude": velocity_magnitude,
            "mach_number": mach_number,
            "dynamic_pressure": dynamic_pressure,
            "air_density": air_density,
            "sound_speed": sound_speed,
            "thrust": thrust,
            "mass": mass,
            "thrust_to_weight": thrust / (mass * 9.81) if mass > 0 else 0,
        }
