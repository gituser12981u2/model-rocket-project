import numpy as np
from scipy.integrate import solve_ivp
from typing import Dict, Any, Optional

# Import our modular components
from analysis.metrics import FlightAnalyzer, FlightMetrics
from models.thrust_curve_motor import ThrustCurveMotor
from simulation.rocket_dynamics import RocketDynamics


class RocketSimulation:
    """
    Main simulation orchestrator - delegates to specialized modules
    """

    def __init__(
        self,
        initial_state: Dict[str, Any],
        rocket_params: Dict[str, Any],
        motor_file_path: str,
    ):
        """
        Initialize the rocket simulation

        Parameters:
        -----------
        initial_state : dict
            Initial conditions (position, velocity, quaternion, angular_velocity)
        rocket_params : dict
            Rocket parameters (dry_mass, reference_area, inertia, etc.)
        motor_file_path : str
            Path to RASP motor file
        """
        # Store initial conditions
        self.initial_state = np.array(
            [
                *initial_state["position"],
                *initial_state["velocity"],
                *initial_state["quaternion"],
                *initial_state["angular_velocity"],
            ]
        )

        self.rocket_params = rocket_params

        # Load motor
        self.motor = ThrustCurveMotor(motor_file_path)
        motor_specs = self.motor.get_motor_specs()

        # Initialize physics engine
        self.dynamics = RocketDynamics(rocket_params, self.motor)

        # Initialize analyzers
        self.flight_analyzer = FlightAnalyzer()

        # Store motor specs for easy access
        self.motor_specs = motor_specs

        # Print motor information
        self._print_motor_info()

        # Results storage
        self.times = None
        self.states = None
        self.metrics = None

    def _print_motor_info(self):
        """Print motor information"""
        print("\n=== Motor Information ===")
        print(
            f"Motor: {self.motor_specs['designation']} ({self.motor_specs['manufacturer']})"
        )
        print(f"Class: {self.motor_specs['impulse_class']}")
        print(f"Total Impulse: {self.motor_specs['total_impulse']:.1f} N⋅s")
        print(f"Average Thrust: {self.motor_specs['average_thrust']:.1f} N")
        print(f"Peak Thrust: {self.motor_specs['peak_thrust']:.1f} N")
        print(f"Burn Time: {self.motor_specs['burn_time']:.2f} s")
        print(
            f"Propellant Mass: {self.motor_specs['propellant_mass']*1000:.1f} g")
        print(
            f"Specific Impulse: {self.motor_specs['specific_impulse']:.0f} s")

    def _ground_impact_event(self, t, y):
        """Event function to detect ground impact on descent (second time at z=0)"""
        altitude = y[2]
        vertical_velocity = y[5]  # vz component

        # Only trigger if:
        # 1. Below ground level (altitude < -0.01) - small buffer
        # 2. Moving downward (vz < 0)
        # 3. After at least 5 seconds (to ensure we've launched)
        if t > 5.0 and altitude < -0.01 and vertical_velocity < 0.0:
            print(f"Ground impact detected at t={t:.3f}")
            return altitude
        else:
            return 1.0  # Positive value, no event

    _ground_impact_event.terminal = True
    _ground_impact_event.direction = -1

    def simulate(
        self, t_end: Optional[float] = None, dt: float = 0.01, method: str = "DOP853"
    ) -> tuple:
        """
        Run the simulation

        Parameters:
        -----------
        t_end : float, optional
            End time (auto-calculated if None)
        dt : float
            Time step for output
        method : str
            Integration method

        Returns:
        --------
        tuple
            (times, states) arrays
        """
        if t_end is None:
            # Auto-calculate reasonable end time
            t_end = max(30.0, self.motor_specs["burn_time"] * 20)

        print("\n=== Running Simulation ===")
        print(f"Integration method: {method}")
        print(f"Time step: {dt} s")
        print(f"End time: {t_end} s")

        # Time evaluation points
        t_eval = np.arange(0.0, t_end, dt)

        # Run integration with ground impact detection
        solution = solve_ivp(
            self.dynamics.state_derivative,
            [0, t_end],
            self.initial_state,
            method=method,
            t_eval=t_eval,
            rtol=1e-6,
            atol=1e-9,
            events=self._ground_impact_event,
        )

        # Store results
        self.times = solution.t
        self.states = solution.y.T

        print("Simulation completed")
        print(f"   Integration points: {len(self.times)}")
        print(f"   Final time: {self.times[-1]:.2f} s")

        # Calculate flight metrics
        self.metrics = self.flight_analyzer.analyze_flight(
            self.times, self.states, self.motor, self.rocket_params
        )

        # Print basic results
        print("\n=== Flight Results ===")
        print(
            f"Apogee: {self.metrics.apogee:.1f} m ({self.metrics.apogee/0.3048:.0f} ft)"
        )
        print(f"Max Velocity: {self.metrics.max_velocity:.1f} m/s")
        print(f"Max Mach: {self.metrics.max_mach:.3f}")
        print(f"Flight Time: {self.metrics.flight_time:.1f} s")

        return self.times, self.states

    def get_metrics(self) -> FlightMetrics:
        """Get flight metrics (run simulation first)"""
        if self.metrics is None:
            raise ValueError("Run simulation first using simulate()")
        return self.metrics

    def get_performance_envelope(self) -> Dict[str, np.ndarray]:
        """Get time-varying performance parameters"""
        if self.times is None or self.states is None:
            raise ValueError("Run simulation first using simulate()")

        return self.flight_analyzer.calculate_performance_envelope(
            self.times, self.states, self.motor
        )

    def generate_full_report(self) -> str:
        """Generate comprehensive flight report"""
        if self.metrics is None:
            raise ValueError("Run simulation first using simulate()")

        report = self.flight_analyzer.generate_flight_report(
            self.metrics, self.motor_specs, self.rocket_params
        )

        print(report)
        return report

    def plot_motor_data(self):
        """Plot motor thrust curve and characteristics"""
        self.motor.plot_motor_data()

    def plot_results(self):
        """Plot simulation results (delegates to visualization module)"""
        if self.times is None or self.states is None:
            raise ValueError("Run simulation first using simulate()")

        # Import here to avoid circular imports
        from utils.visualization import RocketPlotter

        plotter = RocketPlotter()
        envelope = self.get_performance_envelope()

        plotter.plot_comprehensive_results(
            self.times, self.states, envelope, self.motor_specs
        )


class SimulationBuilder:
    """
    Builder pattern for easy simulation setup
    """

    def __init__(self):
        self.initial_state = {
            "position": [0.0, 0.0, 0.0],
            "velocity": [0.0, 0.0, 0.0],
            "quaternion": [1.0, 0.0, 0.0, 0.0],
            "angular_velocity": [0.0, 0.0, 0.0],
        }
        self.rocket_params = {}
        self.motor_file = None

    def with_rocket_mass(self, dry_mass_kg: float):
        """Set rocket dry mass"""
        self.rocket_params["dry_mass"] = dry_mass_kg
        return self

    def with_reference_area(self, area_m2: float):
        """Set reference area"""
        self.rocket_params["reference_area"] = area_m2
        return self

    def with_inertia(self, ixx: float, iyy: float, izz: float):
        """Set moments of inertia"""
        self.rocket_params["inertia"] = [ixx, iyy, izz]
        return self

    def with_aerodynamic_damping(self, damping: float):
        """Set aerodynamic damping coefficient"""
        self.rocket_params["aero_damping"] = damping
        return self

    def with_motor(self, motor_file_path: str):
        """Set motor file"""
        self.motor_file = motor_file_path
        return self

    def with_launch_angle(self, angle_degrees: float):
        """Set launch angle from vertical"""
        angle_rad = np.radians(angle_degrees)
        # Quaternion for rotation about y-axis
        self.initial_state["quaternion"] = [
            np.cos(angle_rad / 2),
            0.0,
            np.sin(angle_rad / 2),
            0.0,
        ]
        return self

    def with_initial_velocity(self, vx: float, vy: float, vz: float):
        """Set initial velocity"""
        self.initial_state["velocity"] = [vx, vy, vz]
        return self

    def build(self) -> RocketSimulation:
        """Build the simulation"""
        if self.motor_file is None:
            raise ValueError("Motor file must be specified")

        # Set defaults for missing parameters
        if "dry_mass" not in self.rocket_params:
            raise ValueError("Rocket dry mass must be specified")
        if "reference_area" not in self.rocket_params:
            raise ValueError("Reference area must be specified")
        if "inertia" not in self.rocket_params:
            self.rocket_params["inertia"] = [0.0001, 0.01, 0.01]
        if "aero_damping" not in self.rocket_params:
            self.rocket_params["aero_damping"] = 0.005

        return RocketSimulation(self.initial_state, self.rocket_params, self.motor_file)
