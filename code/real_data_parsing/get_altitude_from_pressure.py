import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
import warnings
warnings.filterwarnings('ignore')


def pressure_to_altitude(pressure_pa, sea_level_pressure=101325):
    """Convert pressure to altitude using barometric formula."""
    return 44330 * (1 - (pressure_pa / sea_level_pressure) ** 0.1903)


def find_launch_time(pressure_pa, time_ms, threshold=3):
    """
    Find launch time by detecting the START of sustained pressure decrease.
    """
    window_size = 50
    if len(pressure_pa) < window_size:
        return 0

    pressure_smooth = np.convolve(pressure_pa, np.ones(
        window_size)/window_size, mode='valid')

    max_decline_rate = 0
    best_start_idx = 0

    for i in range(len(pressure_smooth) - 200):
        if i + 200 < len(pressure_smooth):
            decline_rate = (
                pressure_smooth[i] - pressure_smooth[i + 200]) / 200

            if decline_rate > max_decline_rate and decline_rate > threshold:
                max_decline_rate = decline_rate
                best_start_idx = i

    original_idx = best_start_idx + window_size // 2

    if original_idx < 100:
        pressure_diff = np.diff(pressure_pa)
        for i in range(100, len(pressure_diff) - 100):
            if pressure_diff[i] < -threshold and np.mean(pressure_diff[i:i+50]) < -threshold/2:
                return i

    return max(0, original_idx - 50)


def find_data_corruption_point(pressure_pa, time_ms):
    """
    Find where the sensor likely shorted/corrupted.
    """
    apogee_idx = np.argmin(pressure_pa)
    post_apogee = pressure_pa[apogee_idx:]

    if len(post_apogee) > 50:
        final_pressure = np.mean(post_apogee[-50:])
        apogee_pressure = pressure_pa[apogee_idx]
        recovery = final_pressure - apogee_pressure

        if recovery < 500:
            for i in range(apogee_idx + 50, len(pressure_pa) - 50):
                recent_variation = np.std(pressure_pa[i:i+50])
                if recent_variation < 5:
                    return i
            return min(apogee_idx + 200, len(pressure_pa))

    return len(pressure_pa)


def calculate_spline_analysis(altitude, time_s, smoothing_factors=None):
    """
    Calculate velocity using spline fitting with automatic smoothing selection.
    """
    if smoothing_factors is None:
        smoothing_factors = [1.0, 10.0, 50.0, 100.0, 500.0]

    spline_results = {}

    for s_factor in smoothing_factors:
        try:
            spline = UnivariateSpline(time_s, altitude, s=s_factor)
            altitude_smooth = spline(time_s)
            velocity_spline = spline.derivative()(time_s)

            # Calculate acceleration with gradient of velocity
            acceleration_from_velocity = np.gradient(velocity_spline, time_s)

            # Optional: Apply additional light smoothing to acceleration only
            if len(acceleration_from_velocity) > 11:
                from scipy.signal import savgol_filter
                window_size = min(11, len(acceleration_from_velocity) // 3)
                if window_size % 2 == 0:
                    window_size += 1
                acceleration_smooth = savgol_filter(
                    acceleration_from_velocity, window_size, 3)
            else:
                acceleration_smooth = acceleration_from_velocity

            spline_results[s_factor] = {
                'altitude': altitude_smooth,
                'velocity': velocity_spline,
                'acceleration': acceleration_smooth,
                'max_velocity': np.max(velocity_spline),
                'max_acceleration': np.max(np.abs(acceleration_smooth))
            }
        except Exception as e:
            print(f"Spline calculation failed for s={s_factor}: {e}")

    # Select best spline (reasonable acceleration < 300 m/s²)
    reasonable_results = {k: v for k, v in spline_results.items()
                          if v['max_acceleration'] < 300}

    if reasonable_results:
        best_factor = min(reasonable_results.keys(),
                          key=lambda k: reasonable_results[k]['max_acceleration'])
    else:
        best_factor = max(spline_results.keys())

    return spline_results[best_factor], best_factor


def analyze_rocket_data(csv_file):
    """
    Analyze rocket data with spline.
    """
    print("🚀 Rocket Data Analyzer")
    print("=" * 50)

    # Read data
    df = pd.read_csv(csv_file)
    df.columns = df.columns.str.strip()
    df_clean = df.iloc[2:].copy()

    time_ms = df_clean['Time_ms'].values
    pressure_pa = df_clean['Pressure_Pa'].values

    print(f"Loaded {len(df_clean)} data points")
    print(
        f"Time range: {time_ms[0]/1000:.1f} to {time_ms[-1]/1000:.1f} seconds")
    print(
        f"Pressure range: {np.min(pressure_pa):.0f} to {np.max(pressure_pa):.0f} Pa")

    # Find launch time and data corruption point
    launch_idx = find_launch_time(pressure_pa, time_ms, threshold=3)
    corruption_idx = find_data_corruption_point(pressure_pa, time_ms)

    print(f"Detected launch at: {time_ms[launch_idx]/1000:.1f} seconds")
    if corruption_idx < len(pressure_pa):
        print(
            f"Sensor corruption detected at: {time_ms[corruption_idx]/1000:.1f} seconds")

    # Extract flight data
    useful_data = slice(launch_idx, corruption_idx)
    time_flight = time_ms[useful_data] - time_ms[launch_idx]
    pressure_flight = pressure_pa[useful_data]
    time_s = time_flight / 1000

    print(
        f"Analyzing {len(pressure_flight)} points over {time_s[-1]:.1f} seconds")
    print(
        f"Pressure drop: {np.max(pressure_flight) - np.min(pressure_flight):.0f} Pa")

    # Convert to altitude
    baseline_pressure = np.mean(
        pressure_flight[:min(50, len(pressure_flight)//4)])
    altitude_raw = pressure_to_altitude(pressure_flight, baseline_pressure)

    print(f"Baseline pressure: {baseline_pressure:.1f} Pa")

    # Apply spline analysis
    print("\nApplying spline analysis...")
    spline_result, best_smoothing = calculate_spline_analysis(
        altitude_raw, time_s)

    # Extract results
    altitude = spline_result['altitude']
    velocity = spline_result['velocity']
    acceleration = spline_result['acceleration']

    # Trim initial artifacts (remove first 0.05 seconds to fix acceleration spike)
    trim_time = 0.05
    trim_idx = np.argmax(time_s >= trim_time)
    if trim_idx == 0:
        trim_idx = min(5, len(time_s) // 10)  # Fallback: trim first 5 points

    print(
        f"Trimming first {trim_idx} data points ({time_s[trim_idx]:.3f}s) to remove initial artifacts")

    # Apply trimming to all arrays
    time_s_trimmed = time_s[trim_idx:] - \
        time_s[trim_idx]  # Reset time to start at 0
    altitude_raw_trimmed = altitude_raw[trim_idx:]
    altitude_trimmed = altitude[trim_idx:]
    velocity_trimmed = velocity[trim_idx:]
    acceleration_trimmed = acceleration[trim_idx:]
    pressure_flight_trimmed = pressure_flight[trim_idx:]

    # Update variables to use trimmed versions
    time_s = time_s_trimmed
    altitude = altitude_trimmed
    velocity = velocity_trimmed
    acceleration = acceleration_trimmed
    pressure_flight = pressure_flight_trimmed
    altitude_raw = altitude_raw_trimmed

    # Calculate key metrics
    max_altitude = np.max(altitude)
    max_altitude_idx = np.argmax(altitude)
    max_altitude_time = time_s[max_altitude_idx]

    max_velocity = np.max(velocity)
    max_velocity_idx = np.argmax(velocity)
    max_velocity_time = time_s[max_velocity_idx]

    max_acceleration = np.max(np.abs(acceleration))
    max_acceleration_idx = np.argmax(np.abs(acceleration))
    max_acceleration_time = time_s[max_acceleration_idx]

    # Print results
    print(f"\n=== FLIGHT ANALYSIS RESULTS ===")
    print(f"Spline smoothing factor: {best_smoothing}")
    print(
        f"Data trimmed: {trim_idx} points ({time_s[0] + trim_time:.3f}s) to fix acceleration artifacts")
    print(
        f"Maximum altitude: {max_altitude:.1f} meters ({max_altitude*3.281:.0f} feet)")
    print(f"Time to apogee: {max_altitude_time:.1f} seconds")
    print(
        f"Maximum velocity: {max_velocity:.1f} m/s ({max_velocity*2.237:.0f} mph)")
    print(f"Maximum velocity at: {max_velocity_time:.1f} seconds")
    print(
        f"Maximum acceleration: {max_acceleration:.1f} m/s² ({max_acceleration/9.81:.1f} g)")
    print(f"Mach number: {max_velocity/343:.3f}")
    print(f"Flight time analyzed: {time_s[-1]:.1f} seconds")

    # Create plots
    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    fig.suptitle('Rocket Flight Analysis',
                 fontsize=16, fontweight='bold')

    # Plot 1: Pressure vs Time
    axes[0, 0].plot(time_s, pressure_flight, 'b-', linewidth=2)
    axes[0, 0].set_title('Pressure vs Time')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Pressure (Pa)')
    axes[0, 0].grid(True, alpha=0.3)

    # Plot 2: Altitude vs Time
    axes[0, 1].plot(time_s, altitude_raw, 'lightgray',
                    alpha=0.5, linewidth=1, label='Raw')
    axes[0, 1].plot(time_s, altitude, 'b-', linewidth=2, label='Spline')
    axes[0, 1].scatter(max_altitude_time, max_altitude, color='red', s=100, zorder=5,
                       label=f'Apogee: {max_altitude:.1f}m')
    axes[0, 1].set_title('Altitude vs Time')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Altitude (m)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # Plot 3: Velocity vs Time
    axes[0, 2].plot(time_s, velocity, 'g-', linewidth=2)
    axes[0, 2].scatter(max_velocity_time, max_velocity, color='red', s=100, zorder=5,
                       label=f'Max: {max_velocity:.1f} m/s')
    axes[0, 2].axhline(y=0, color='black', linestyle='-', alpha=0.3)
    axes[0, 2].set_title('Velocity vs Time')
    axes[0, 2].set_xlabel('Time (s)')
    axes[0, 2].set_ylabel('Velocity (m/s)')
    axes[0, 2].legend()
    axes[0, 2].grid(True, alpha=0.3)

    # Plot 4: Acceleration vs Time
    axes[0, 3].plot(time_s, acceleration, 'r-', linewidth=2)
    axes[0, 3].scatter(max_acceleration_time, acceleration[max_acceleration_idx],
                       color='red', s=100, zorder=5, label=f'Max: {max_acceleration:.1f} m/s²')
    axes[0, 3].axhline(y=0, color='black', linestyle='-', alpha=0.3)
    axes[0, 3].set_title('Acceleration vs Time')
    axes[0, 3].set_xlabel('Time (s)')
    axes[0, 3].set_ylabel('Acceleration (m/s²)')
    axes[0, 3].legend()
    axes[0, 3].grid(True, alpha=0.3)

    # Plot 5: Spline Smoothing Comparison
    smoothing_factors = [1.0, 10.0, 50.0, 100.0, 500.0]
    spline_results = {}
    for s in smoothing_factors:
        try:
            spline = UnivariateSpline(time_s, altitude_raw, s=s)
            spline_results[s] = np.max(spline.derivative()(time_s))
        except:
            spline_results[s] = 0

    smoothing_vals = list(spline_results.keys())
    max_vels = list(spline_results.values())

    axes[1, 0].plot(smoothing_vals, max_vels, 'bo-', linewidth=2, markersize=8)
    axes[1, 0].scatter(best_smoothing, max_velocity, color='red', s=150, zorder=5,
                       label=f'Selected: s={best_smoothing}')
    axes[1, 0].set_xlabel('Spline Smoothing Factor')
    axes[1, 0].set_ylabel('Max Velocity (m/s)')
    axes[1, 0].set_xscale('log')
    axes[1, 0].set_title('Spline Parameter Selection')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Plot 6: Phase Plot (Velocity vs Altitude)
    axes[1, 1].plot(velocity, altitude, 'purple', linewidth=2)
    axes[1, 1].scatter(velocity[0], altitude[0],
                       color='green', s=100, label='Launch')
    axes[1, 1].scatter(velocity[max_altitude_idx], altitude[max_altitude_idx],
                       color='red', s=100, label='Apogee')
    axes[1, 1].set_title('Phase Plot (Velocity vs Altitude)')
    axes[1, 1].set_xlabel('Velocity (m/s)')
    axes[1, 1].set_ylabel('Altitude (m)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    # Plot 7: Motor Burn vs Coast Phase
    motor_burnout_time = 0.81  # C11 burn time
    motor_phase = time_s <= motor_burnout_time

    axes[1, 2].plot(time_s[motor_phase], velocity[motor_phase],
                    'red', linewidth=3, label='Motor burn')
    axes[1, 2].plot(time_s[~motor_phase], velocity[~motor_phase],
                    'blue', linewidth=2, label='Coasting')
    axes[1, 2].axvline(x=motor_burnout_time, color='black', linestyle='--', alpha=0.7,
                       label=f'Burnout ({motor_burnout_time}s)')
    axes[1, 2].set_title('Motor Burn vs Coast Phase')
    axes[1, 2].set_xlabel('Time (s)')
    axes[1, 2].set_ylabel('Velocity (m/s)')
    axes[1, 2].legend()
    axes[1, 2].grid(True, alpha=0.3)

    # Plot 8: Flight Trajectory Summary
    axes[1, 3].text(0.05, 0.95, 'Flight Summary:', transform=axes[1, 3].transAxes,
                    fontsize=14, fontweight='bold', verticalalignment='top')

    summary_text = f"""
Max Altitude: {max_altitude:.1f} m
Time to Apogee: {max_altitude_time:.1f} s
Max Velocity: {max_velocity:.1f} m/s
Max Acceleration: {max_acceleration:.1f} m/s²
Mach Number: {max_velocity/343:.3f}
Flight Time: {time_s[-1]:.1f} s

Burnout Velocity: {velocity[time_s <= motor_burnout_time][-1]:.1f} m/s
Coast Time: {max_altitude_time - motor_burnout_time:.1f} s

Data Points: {len(time_s)}
Smoothing: s={best_smoothing}
"""

    axes[1, 3].text(0.05, 0.85, summary_text, transform=axes[1, 3].transAxes,
                    fontsize=11, verticalalignment='top', fontfamily='monospace')
    axes[1, 3].set_xlim(0, 1)
    axes[1, 3].set_ylim(0, 1)
    axes[1, 3].axis('off')

    plt.tight_layout()
    plt.show()

    # Save results to CSV
    results_df = pd.DataFrame({
        'Time_s': time_s,
        'Altitude_m': altitude,
        'Velocity_ms': velocity,
        'Acceleration_ms2': acceleration,
        'Pressure_Pa': pressure_flight
    })

    output_filename = csv_file.replace('.csv', '_results.csv')
    results_df.to_csv(output_filename, index=False)
    print(f"\n Results saved to: {output_filename}")

    return {
        'time': time_s,
        'altitude': altitude,
        'velocity': velocity,
        'acceleration': acceleration,
        'pressure': pressure_flight,
        'max_altitude': max_altitude,
        'max_velocity': max_velocity,
        'max_acceleration': max_acceleration,
        'time_to_apogee': max_altitude_time,
        'flight_time': time_s[-1],
        'smoothing_factor': best_smoothing
    }


if __name__ == "__main__":
    csv_filename = "data.csv"
    results = analyze_rocket_data(csv_filename)

    print(f"\n{'='*50}")
    print(f"Analysis complete! Results saved to CSV file.")
    print(
        f"Peak Altitude: {results['max_altitude']:.1f}m ({results['max_altitude']*3.281:.0f}ft)")
    print(
        f"Peak Velocity: {results['max_velocity']:.1f}m/s (Mach {results['max_velocity']/343:.3f})")
    print(f"Time to Apogee: {results['time_to_apogee']:.1f}s")
