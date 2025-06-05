"""
Inertia tensor calculation for model rockets
"""

from typing import List, Tuple


class RocketComponent:
    """Individual rocket component for inertia calculations"""

    def __init__(
        self,
        name: str,
        mass: float,
        length: float,
        position: float,
        radius: float = None,
        component_type: str = "cylinder",
    ):
        """
        Initialize rocket component

        Parameters:
        -----------
        name : str
            Component name
        mass : float
            Component mass in kg
        length : float
            Component length in meters
        position : float
            Position of component CG from nose tip in meters
        radius : float, optional
            Component radius in meters (uses rocket radius if None)
        component_type : str
            Type: 'cone', 'cylinder', 'hollow_cylinder', 'fins', 'point_mass'
        """
        self.name = name
        self.mass = mass
        self.length = length
        self.position = position
        self.radius = radius
        self.component_type = component_type


class InertiaCalculator:
    """Calculate rocket inertia tensor from component data"""

    def __init__(self, rocket_radius: float):
        """
        Initialize calculator

        Parameters:
        -----------
        rocket_radius : float
            Main rocket body radius in meters
        """
        self.rocket_radius = rocket_radius
        self.components = []

    def add_component(self, component: RocketComponent):
        """Add a component to the rocket"""
        if component.radius is None:
            component.radius = self.rocket_radius
        self.components.append(component)

    def calculate_cg(self) -> float:
        """Calculate center of gravity position from nose"""
        total_mass = sum(comp.mass for comp in self.components)
        total_moment = sum(
            comp.mass * comp.position for comp in self.components)
        return total_moment / total_mass

    def calculate_component_inertia(
        self, component: RocketComponent
    ) -> Tuple[float, float]:
        """
        Calculate component inertia about its own CG

        Returns:
        --------
        tuple : (Ixx_local, Iyy_local) in kg⋅m²
        """
        mass = component.mass
        length = component.length
        radius = component.radius

        if component.component_type == "cone":
            # Solid cone
            Ixx = (3 / 10) * mass * radius**2
            Iyy = (3 / 5) * mass * radius**2 + (3 / 80) * mass * length**2

        elif component.component_type == "cylinder":
            # Solid cylinder
            Ixx = (1 / 2) * mass * radius**2
            Iyy = (1 / 12) * mass * length**2 + (1 / 4) * mass * radius**2

        elif component.component_type == "hollow_cylinder":
            # Thin-walled hollow cylinder (body tube)
            Ixx = mass * radius**2  # Thin wall approximation
            Iyy = (1 / 12) * mass * length**2 + (1 / 2) * mass * radius**2

        elif component.component_type == "fins":
            # Fins - treat as thin plates at radius
            Ixx = 0.5 * mass * radius**2  # Fins extend radially
            Iyy = (1 / 12) * mass * length**2  # Chord length

        elif component.component_type == "point_mass":
            # Point mass (recovery, electronics, etc.)
            Ixx = 0.0
            Iyy = 0.0

        else:
            Ixx = (1 / 2) * mass * radius**2
            Iyy = (1 / 12) * mass * length**2 + (1 / 4) * mass * radius**2

        return Ixx, Iyy

    def calculate_inertia_tensor(
        self, verbose: bool = True
    ) -> Tuple[float, float, float]:
        """
        Calculate full rocket inertia tensor

        Parameters:
        -----------
        verbose : bool
            Print detailed component breakdown

        Returns:
        --------
        tuple : (Ixx, Iyy, Izz) in kg⋅m²
        """
        if not self.components:
            raise ValueError("No components added to rocket")

        # Calculate rocket CG
        rocket_cg = self.calculate_cg()
        total_mass = sum(comp.mass for comp in self.components)

        if verbose:
            print(f"Rocket CG: {rocket_cg*100:.1f} cm from nose")
            print(f"Total mass: {total_mass*1000:.1f} g")
            print("\nComponent breakdown:")

        # Initialize total inertias
        Ixx_total = 0.0
        Iyy_total = 0.0
        Izz_total = 0.0

        for comp in self.components:
            # Calculate component's local inertia
            Ixx_local, Iyy_local = self.calculate_component_inertia(comp)

            # Distance from component CG to rocket CG
            distance_to_cg = abs(comp.position - rocket_cg)

            # Apply parallel axis theorem
            Ixx_contrib = Ixx_local  # Roll inertia doesn't change
            Iyy_contrib = Iyy_local + comp.mass * distance_to_cg**2
            Izz_contrib = Iyy_contrib  # Same as Iyy for axisymmetric rocket

            # Add to totals
            Ixx_total += Ixx_contrib
            Iyy_total += Iyy_contrib
            Izz_total += Izz_contrib

            if verbose:
                print(
                    f"  {comp.name:12}: {comp.mass*1000:4.0f}g at {comp.position*100:4.1f}cm, "
                    f"dist={distance_to_cg*100:4.1f}cm, type={comp.component_type}"
                )

        if verbose:
            print("\nInertia Tensor [kg*m²]:")
            print(f"  Ixx (roll):  {Ixx_total:.2e}")
            print(f"  Iyy (pitch): {Iyy_total:.2e}")
            print(f"  Izz (yaw):   {Izz_total:.2e}")
            print(
                f"\nFor simulation: .with_inertia({Ixx_total:.6f}, {Iyy_total:.6f}, {Izz_total:.6f})"
            )

        return Ixx_total, Iyy_total, Izz_total


def create_typical_rocket_components(
    rocket_diameter: float, rocket_length: float, total_mass: float
) -> List[RocketComponent]:
    """
    Create typical component breakdown for a model rocket

    Parameters:
    -----------
    rocket_diameter : float
        Rocket diameter in meters
    rocket_length : float
        Rocket length in meters
    total_mass : float
        Total rocket mass in kg

    Returns:
    --------
    List[RocketComponent]
        List of typical components
    """
    radius = rocket_diameter / 2

    # Typical mass fractions for model rockets
    components = [
        RocketComponent(
            "nose_cone",
            total_mass * 0.15,
            rocket_length * 0.20,
            rocket_length * 0.10,
            radius,
            "cone",
        ),
        RocketComponent(
            "body_tube",
            total_mass * 0.25,
            rocket_length * 0.70,
            rocket_length * 0.50,
            radius,
            "hollow_cylinder",
        ),
        RocketComponent(
            "recovery",
            total_mass * 0.15,
            rocket_length * 0.15,
            rocket_length * 0.35,
            radius,
            "point_mass",
        ),
        RocketComponent(
            "fins", total_mass * 0.12, 0.05, rocket_length * 0.90, radius * 1.5, "fins"
        ),
        RocketComponent(
            "motor_mount",
            total_mass * 0.08,
            rocket_length * 0.20,
            rocket_length * 0.90,
            0.012,
            "cylinder",
        ),
        RocketComponent(
            "misc",
            total_mass * 0.25,
            rocket_length * 0.50,
            rocket_length * 0.50,
            radius,
            "point_mass",
        ),
    ]

    return components


# Convenience function for quick calculations
def calculate_rocket_inertia(
    rocket_diameter: float,
    rocket_length: float,
    total_mass: float,
    components: List[RocketComponent] = None,
    verbose: bool = True,
) -> Tuple[float, float, float]:
    """
    Quick inertia calculation for a rocket

    Parameters:
    -----------
    rocket_diameter : float
        Rocket diameter in meters
    rocket_length : float
        Rocket length in meters
    total_mass : float
        Total rocket mass in kg
    components : List[RocketComponent], optional
        Custom component list (uses typical if None)
    verbose : bool
        Print detailed output

    Returns:
    --------
    tuple : (Ixx, Iyy, Izz) in kg*m²
    """
    calculator = InertiaCalculator(rocket_diameter / 2)

    if components is None:
        components = create_typical_rocket_components(
            rocket_diameter, rocket_length, total_mass
        )

    for comp in components:
        calculator.add_component(comp)

    return calculator.calculate_inertia_tensor(verbose)
