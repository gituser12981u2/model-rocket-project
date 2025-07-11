from models.inertia import InertiaCalculator, RocketComponent
from simulation.rocket_sim import SimulationBuilder


def main():
    """Simple simulation example"""
    print("Rocket Simulation")
    print("=" * 40)

    # Create inertia calculator for your rocket
    calculator = InertiaCalculator(rocket_radius=0.0125)  # 25mm diameter

    # Add your specific components
    calculator.add_component(RocketComponent(
        "nose_cone", 0.00947, 0.15, 0.04, component_type="cone"))
    calculator.add_component(RocketComponent(
        "body_tube", 0.025, 0.21, 0.19, component_type="hollow_cylinder"))
    calculator.add_component(RocketComponent(
        "recovery", 0.015, 0.05, 0.15, component_type="point_mass"))
    calculator.add_component(RocketComponent(
        "fins", 0.010, 0.05, 0.35, radius=0.02, component_type="fins"))
    calculator.add_component(RocketComponent(
        "motor_mount", 0.005, 0.07, 0.35, radius=0.012, component_type="cylinder"))

    # Calculate inertia tensor
    Ixx, Iyy, Izz = calculator.calculate_inertia_tensor()

    # Define rocket parameters
    sim = (
        SimulationBuilder()
        .with_motor("C11-7.eng")
        .with_rocket_mass(0.091)  # 85g dry mass
        .with_reference_area(0.000491)  # 42mm diameter -> π*(0.021)²
        .with_inertia(Ixx, Iyy, Izz)  # [Ixx, Iyy, Izz] in kg⋅m²
        .with_aerodynamic_damping(0.005)
        .build()
    )

    # Run simulation
    sim.simulate(t_end=60.0, dt=0.02)

    # Get results
    metrics = sim.get_metrics()

    # Export data to csv
    # sim.export_to_csv("model_results.csv")

    # Print summary
    print("\n Results:")
    print(
        f"   Apogee: {metrics.apogee:.1f} m ({metrics.apogee/0.3048:.0f} ft)")
    print(f"   Max Velocity: {metrics.max_velocity:.1f} m/s")
    print(f"   Max Mach: {metrics.max_mach:.3f}")
    print(f"   Flight Time: {metrics.flight_time:.1f} s")

    # Plot results
    sim.plot_motor_data()
    sim.plot_results()


if __name__ == "__main__":
    main()
