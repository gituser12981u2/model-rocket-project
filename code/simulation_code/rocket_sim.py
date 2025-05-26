import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

from models.atmosphere import StandardAtmosphere
from models.gravity import GravityModel
from simulation.quaternion import quaternion_derivative, quaternion_to_dcm


class RocketSimulation:
    def __init__(self, initial_state, rocket_params):
        """
        Initialize the rocket simulation with initial state and parameters.

        Parameters:
        -----------
        initial_state : dict
            Contains initial position, velocity, quaternion, and angular velocity
        rocket_params : dict
            Contains rocket parameters like initial mass, thrust profile, aerodynamics, etc.
        """
        self.state = np.array([
            *initial_state['position'],      # x, y, z position
            *initial_state['velocity'],      # vx, vy, vz velocity
            # quaternion for attitude [q0, q1, q2, q3]
            *initial_state['quaternion'],
            *initial_state['angular_velocity']  # wx, wy, wz angular velocity
        ])

        self.params = rocket_params

        # Initialize models
        self.atmosphere = StandardAtmosphere()
        self.gravity = GravityModel(model_type='inverse_square')

        self.time = 0.0
        self.history = {'time': [], 'state': []}

    def calculate_mass(self, t):
        """Calculate rocket mass at time t based on propellant consumption"""
        if t < self.params['burn_time']:
            # Linear mass decrease during burn
            mass_rate = self.params['propellant_mass'] / \
                self.params['burn_time']
            return self.params['dry_mass'] + self.params['propellant_mass'] - mass_rate * t
        else:
            return self.params['dry_mass']

    def calculate_thrust(self, t):
        """Calculate thrust vector at time t"""
        if t < self.params['burn_time']:
            thrust_magnitude = self.params['thrust_profile'](t)

            # Convert from body to world coordinates using current attitude
            q = self.state[6:10]
            dcm = quaternion_to_dcm(q)

            # Assuming thrust is along body x-axis
            thrust_body = np.array([0.0, 0.0, thrust_magnitude])
            thrust_world = dcm @ thrust_body
            return thrust_world
        else:
            return np.zeros(3)

    def calculate_aero_forces(self, pos, vel):
        """Calculate aerodynamic forces based on velocity, air density, and Mach number"""
        v_mag = np.linalg.norm(vel)
        if v_mag < 1e-6:
            return np.zeros(3)

        # Calculate air density using atmosphere model
        altitude = pos[2]
        rho = self.atmosphere.density(altitude)

        # Calculate Mach number
        sound_speed = self.atmosphere.sound_speed(altitude)
        mach = v_mag / sound_speed if sound_speed > 0 else 0.0

        # Calculate dynamic pressure
        q_dyn = 0.5 * rho * v_mag**2

        # Get drag coefficient based on Mach number
        cd = self.params['cd_function'](mach)

        # Calculate reference area - this could be a function of angle of attack
        area = self.params['reference_area']

        # Calculate drag force magnitude
        drag_mag = q_dyn * cd * area

        # Drag acts opposite to velocity direction
        drag_direction = -vel / v_mag

        return drag_mag * drag_direction

    def calculate_moments(self, pos, vel, omega):
        """Calculate aerodynamic moments based on velocity, angular velocity, etc."""
        # This would include calculations for aerodynamic moments,
        # control surface effects, thrust misalignment, etc.
        v_mag = np.linalg.norm(vel)
        if v_mag < 1e-6:
            return np.zeros(3)

        # For a simple model, we can assume moments proportional to angular velocity (damping)
        damping = self.params['aero_damping']
        damping_moment = -damping * omega

        # Add other moment terms as needed
        return damping_moment

    def state_derivative(self, t, state):
        """Calculate state derivative for integration."""
        # Extract state variables
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]

        # Normalize quaternion to prevent numerical drift
        quat = quat / np.linalg.norm(quat)

        # Calculate current mass and its derivative
        m = self.calculate_mass(t)
        if t < self.params['burn_time']:
            m_dot = -self.params['propellant_mass'] / self.params['burn_time']
        else:
            m_dot = 0.0

        # Calculate external forces
        g = self.gravity.acceleration(pos)
        thrust = self.calculate_thrust(t)
        aero_forces = self.calculate_aero_forces(pos, vel)

        # Sum all forces
        forces = thrust + aero_forces + m * g

        # Translational dynamics (your equation)
        # a_cm = F_t/m + (1/m)(-0.5*rho*v^2*C_D*A*v_hat + (1-M_∞^2-(γ+1)*M_∞^2*v_x)∇·v) - g - (v_cm/m)*(dm/dt)

        # For simplicity, we'll implement a version that includes the key components
        acc = forces / m - (vel / m) * m_dot

        # Rotational dynamics
        moments = self.calculate_moments(pos, vel, omega)

        # Euler's equations for rigid body rotation
        I = np.array(self.params['inertia'])

        # wx_dot = (My - Mz + (Iy - Iz)*wy*wz) / Ix  # and similar for other components
        omega_dot = np.zeros(3)
        omega_dot[0] = (moments[0] + (I[1] - I[2])
                        * omega[1] * omega[2]) / I[0]
        omega_dot[1] = (moments[1] + (I[2] - I[0])
                        * omega[2] * omega[0]) / I[1]
        omega_dot[2] = (moments[2] + (I[0] - I[1])
                        * omega[0] * omega[1]) / I[2]

        # Quaternion derivative
        quat_dot = quaternion_derivative(quat, omega)

        # Assemble state derivative
        state_dot = np.concatenate([vel, acc, quat_dot, omega_dot])

        return state_dot

    def simulate(self, t_end, dt=0.01):
        """Run the simulation until t_end with symplectic integration."""
        # For simplicity, we'll use scipy's solve_ivp which has some symplectic-like methods
        # For true symplectic integration, a custom implementation would be needed

        # Define time points
        t_eval = np.arange(0.0, t_end, dt)

        # Run simulation
        solution = solve_ivp(
            self.state_derivative,
            [0, t_end],
            self.state,
            method='DOP853',  # Higher-order method good for oscillatory systems
            t_eval=t_eval,
            rtol=1e-6,
            atol=1e-9
        )

        # Store results
        self.times = solution.t
        self.states = solution.y.T

        # Find apogee
        apogee_idx = np.argmax(self.states[:, 2])  # z-coordinate
        self.apogee = self.states[apogee_idx, 2]
        self.apogee_time = self.times[apogee_idx]

        # Calculate maximum velocity and Mach
        velocities = np.linalg.norm(self.states[:, 3:6], axis=1)
        max_vel_idx = np.argmax(velocities)
        self.max_velocity = velocities[max_vel_idx]
        self.max_velocity_time = self.times[max_vel_idx]

        # Calculate Mach number at max velocity
        altitude_at_max_vel = self.states[max_vel_idx, 2]
        sound_speed = self.atmosphere.sound_speed(altitude_at_max_vel)
        self.max_mach = self.max_velocity / sound_speed if sound_speed > 0 else 0

        # Print key results
        print(f"Apogee: {self.apogee:.2f} m")
        print(f"Time to apogee: {self.apogee_time:.2f} s")
        print(
            f"Maximum velocity: {self.max_velocity:.2f} m/s at t={self.max_velocity_time:.2f} s")
        print(f"Maximum Mach number: {self.max_mach:.4f}")

        return self.times, self.states

    def plot_trajectory(self):
        """Plot the 3D trajectory of the rocket."""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot trajectory
        ax.plot(self.states[:, 0], self.states[:, 1], self.states[:, 2])

        # Mark starting point
        ax.scatter(self.states[0, 0], self.states[0, 1],
                   self.states[0, 2], color='green', s=100, label='Launch')

        # Mark apogee
        apogee_idx = np.argmax(self.states[:, 2])
        ax.scatter(self.states[apogee_idx, 0], self.states[apogee_idx, 1], self.states[apogee_idx, 2],
                   color='red', s=100, label=f'Apogee: {self.apogee:.2f} m')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Rocket Trajectory')
        ax.legend()

        plt.tight_layout()
        plt.show()

    def plot_time_histories(self):
        """Plot time histories of key variables."""
        fig, axs = plt.subplots(3, 2, figsize=(15, 10))

        # Position
        axs[0, 0].plot(self.times, self.states[:, 0], label='X')
        axs[0, 0].plot(self.times, self.states[:, 1], label='Y')
        axs[0, 0].plot(self.times, self.states[:, 2], label='Z')
        axs[0, 0].set_title('Position vs Time')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('Position (m)')
        axs[0, 0].legend()

        # Velocity
        axs[0, 1].plot(self.times, self.states[:, 3], label='Vx')
        axs[0, 1].plot(self.times, self.states[:, 4], label='Vy')
        axs[0, 1].plot(self.times, self.states[:, 5], label='Vz')
        axs[0, 1].set_title('Velocity vs Time')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Velocity (m/s)')
        axs[0, 1].legend()

        # Attitude (quaternion)
        axs[1, 0].plot(self.times, self.states[:, 6], label='q0')
        axs[1, 0].plot(self.times, self.states[:, 7], label='q1')
        axs[1, 0].plot(self.times, self.states[:, 8], label='q2')
        axs[1, 0].plot(self.times, self.states[:, 9], label='q3')
        axs[1, 0].set_title('Quaternion vs Time')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Quaternion Component')
        axs[1, 0].legend()

        # Angular Velocity
        axs[1, 1].plot(self.times, self.states[:, 10], label='ωx')
        axs[1, 1].plot(self.times, self.states[:, 11], label='ωy')
        axs[1, 1].plot(self.times, self.states[:, 12], label='ωz')
        axs[1, 1].set_title('Angular Velocity vs Time')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Angular Velocity (rad/s)')
        axs[1, 1].legend()

        # Calculate altitude (Z position)
        altitude = self.states[:, 2]

        # Calculate total velocity magnitude
        vel_mag = np.sqrt(
            self.states[:, 3]**2 + self.states[:, 4]**2 + self.states[:, 5]**2)

        # Altitude vs time
        axs[2, 0].plot(self.times, altitude)
        axs[2, 0].plot(self.apogee_time, self.apogee, 'ro',
                       label=f'Apogee: {self.apogee:.2f} m')
        axs[2, 0].set_title('Altitude vs Time')
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 0].set_ylabel('Altitude (m)')
        axs[2, 0].legend()

        # Velocity magnitude vs time
        axs[2, 1].plot(self.times, vel_mag)
        axs[2, 1].set_title('Velocity Magnitude vs Time')
        axs[2, 1].set_xlabel('Time (s)')
        axs[2, 1].set_ylabel('Velocity (m/s)')

        plt.tight_layout()
        plt.show()


# Example usage
def main():
    # Define example rocket parameters
    rocket_params = {
        'dry_mass': 1.0,  # kg
        'propellant_mass': 0.5,  # kg
        'burn_time': 3.0,  # seconds
        'thrust_profile': lambda t: 20.0,  # Constant thrust of 20 N
        'reference_area': 0.01,  # m^2
        # Simple CD model
        'cd_function': lambda mach: 0.5 if mach < 0.8 else 0.5 + 0.3*(mach-0.8),
        # kg.m^2, diagonal elements of inertia tensor
        'inertia': [0.01, 0.1, 0.1],
        'aero_damping': 0.01,  # Simple damping coefficient for rotational motion
    }

    # Define initial state
    initial_state = {
        'position': [0.0, 0.0, 0.0],  # m
        'velocity': [0.0, 0.0, 0.0],  # m/s
        # Identity quaternion (no rotation)
        'quaternion': [1.0, 0.0, 0.0, 0.0],
        'angular_velocity': [0.0, 0.0, 0.0]  # rad/s
    }

    # Create and run simulation
    sim = RocketSimulation(initial_state, rocket_params)
    sim.simulate(t_end=30.0, dt=0.05)

    # Plot results
    sim.plot_trajectory()
    sim.plot_time_histories()


if __name__ == "__main__":
    main()
