import numpy as np


class RASPMotorFile:
    """
    Parser for RASP (.eng) motor data files from ThrustCurve.org

    This class loads certified motor data.
    """

    def __init__(self, file_path: str, use_advanced_interpolation: bool = True):
        """
        Initialize from RASP data

        Parameters:

        file_path_or_data : str
            Either a path a .eng file or the raw file contents
        use_advanced_interpolation : bool
            If True, use PCHIP interpolation for smoother curves
        """
        self.use_advanced_interpolation = use_advanced_interpolation

        with open(file_path, 'r') as f:
            raw_data = f.read()

        self._parse_rasp_data(raw_data)
        self._create_interpolators()

    def _parse_rasp_data(self, raw_data: str):
        """Parse RASP format motor data"""
        lines = raw_data.strip().split('\n')

        # Skip comment lines at the beginning
        header_line = None
        data_start_idx = 0

        for i, line in enumerate(lines):
            line = line.strip()
            if line and not line.startswith(';'):
                header_line = line
                data_start_idx = i + 1
                break

        if not header_line:
            raise ValueError("No header line found in RASP file")

        # Parse header: code diameter length delays propellant_mass total_mass manufacturer
        header_parts = header_line.split()
        if len(header_parts) < 7:


class EstesThrustProfile:
    """Estes model rocket motors thrust profiles"""

    def __init__(self, motor_type: str, use_online_data: bool = True):
        """
        Initialize the thrust profile for a given Estes motor.

        Parameters:
            motor_type : str
                Estes motor designation (e.g., 'A8-3', 'C6-5')
            use_online_data : bool
                If True, attempt to download data from thrustcurve.org
        """
        self.motor_type = motor_type
        self.motor_class = self._extract_motor_class(motor_type)

    def _extract_motor_class(self, motor_type: str) -> str:
        """Extract motor class (e.g., 'A8' from 'A8-3')"""
        return motor_type.split('-')[0] if '-' in motor_type else motor_type

    def thrust(self, t: float) -> float:
        """
        Get thrust at time t.

        Parameters:
        t : float
            Time in seconds

        Returns:
        float
            Thrust in newtons
        """
        if t < 0:
            return 0.0
        return float(self.thrust_interpolator(t))

    def mass_flow_rate(self, t: float) -> float:
        """
        Get the mass flow rate at time t

        Parameters:
        t : float
            Time in seconds

        Returns
        float
            Mass flow rate in kg/s (negative)
        """
        if 0 <= t <= self.burn_time:
            # Approximate mass flow rate based on thrust
            # More sophisticated models would use actual mass vs time data
            thrust_fraction = self.thrust(
                t) / self.peak_thrust if self.peak_thrust > 0 else 0
            return -self.propellant_mass / self.burn_time * thrust_fraction
        return 0.0
