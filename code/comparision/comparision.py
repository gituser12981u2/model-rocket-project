import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.stats import pearsonr
import os


class FlightDataComparator:
    """
    Compare model simulation results with real flight data
    """

    def __init__(self, model_file: str, data_file: str):
        """
        Initialize comparator with file paths

        Parameters:
        -----------
        model_file : str
            Path to model results CSV file
        data_file : str
            Path to real data CSV file
        """
        self.model_file = model_file
        self.data_file = data_file
        self.model_data = None
        self.real_data = None
        self.model_ascent = None
        self.real_interpolated = None
        self.metrics = None

    def load_data(self):
        """Load and validate both data files"""
        print("Loading data files...")

        # Check if files exist
        if not os.path.exists(self.model_file):
            raise FileNotFoundError(f"Model file not found: {self.model_file}")
        if not os.path.exists(self.data_file):
            raise FileNotFoundError(f"Data file not found: {self.data_file}")

        # Load CSV files
        try:
            self.model_data = pd.read_csv(self.model_file)
            self.real_data = pd.read_csv(self.data_file)
        except Exception as e:
            raise ValueError(f"Error reading CSV files: {e}")

        # Validate columns (excluding pressure)
        required_cols = ['Time_s', 'Altitude_m',
                         'Velocity_ms', 'Acceleration_ms2']

        for col in required_cols:
            if col not in self.model_data.columns:
                raise ValueError(f"Missing column '{col}' in model data")
            if col not in self.real_data.columns:
                raise ValueError(f"Missing column '{col}' in real data")

        print(f"Model data: {len(self.model_data)} points")
        print(f"Real data: {len(self.real_data)} points")

        # Print data ranges
        print(f"\nModel data ranges:")
        print(
            f"  Time: {self.model_data['Time_s'].min():.2f} to {self.model_data['Time_s'].max():.2f} s")
        print(
            f"  Altitude: {self.model_data['Altitude_m'].min():.1f} to {self.model_data['Altitude_m'].max():.1f} m")

        print(f"\nReal data ranges:")
        print(
            f"  Time: {self.real_data['Time_s'].min():.2f} to {self.real_data['Time_s'].max():.2f} s")
        print(
            f"  Altitude: {self.real_data['Altitude_m'].min():.1f} to {self.real_data['Altitude_m'].max():.1f} m")

    def normalize_model_data(self):
        """
        Normalize model data to only include ascent phase (up to apogee)
        """
        print("\nNormalizing model data to ascent phase only...")

        # Find apogee in model data
        apogee_idx = self.model_data['Altitude_m'].idxmax()
        apogee_altitude = self.model_data.loc[apogee_idx, 'Altitude_m']
        apogee_time = self.model_data.loc[apogee_idx, 'Time_s']

        print(
            f"Model apogee: {apogee_altitude:.1f} m at t={apogee_time:.2f} s")

        # Keep only data up to and including apogee
        self.model_ascent = self.model_data.iloc[:apogee_idx + 1].copy()

        print(f"Model ascent data: {len(self.model_ascent)} points")
        print(
            f"  Time range: {self.model_ascent['Time_s'].min():.2f} to {self.model_ascent['Time_s'].max():.2f} s")

    def interpolate_data(self):
        """
        Interpolate real data to match model time points for comparison
        """
        print("\nInterpolating real data to match model time points...")

        model_times = self.model_ascent['Time_s'].values
        real_times = self.real_data['Time_s'].values

        # Find overlapping time range
        min_time = max(model_times.min(), real_times.min())
        max_time = min(model_times.max(), real_times.max())

        print(f"Overlapping time range: {min_time:.2f} to {max_time:.2f} s")

        # Filter model data to overlapping range
        mask = (model_times >= min_time) & (model_times <= max_time)
        self.model_ascent = self.model_ascent[mask].copy()

        # Interpolate real data
        self.real_interpolated = pd.DataFrame()
        self.real_interpolated['Time_s'] = self.model_ascent['Time_s'].values

        for col in ['Altitude_m', 'Velocity_ms', 'Acceleration_ms2']:
            # Create interpolation function
            interp_func = interp1d(
                self.real_data['Time_s'],
                self.real_data[col],
                kind='linear',
                bounds_error=False,
                fill_value='extrapolate'
            )

            # Interpolate to model time points
            self.real_interpolated[col] = interp_func(
                self.model_ascent['Time_s'])

        print(f"Final comparison dataset: {len(self.model_ascent)} points")

    def calculate_metrics(self):
        """
        Calculate comprehensive comparison metrics between model and real data
        """
        print("\n" + "="*50)
        print("COMPARISON METRICS")
        print("="*50)

        metrics = {}

        for param in ['Altitude_m', 'Velocity_ms', 'Acceleration_ms2']:
            model_vals = self.model_ascent[param].values
            real_vals = self.real_interpolated[param].values

            # Remove any NaN values
            valid_mask = ~(np.isnan(model_vals) | np.isnan(real_vals))
            model_clean = model_vals[valid_mask]
            real_clean = real_vals[valid_mask]

            if len(model_clean) == 0:
                print(f"\n{param}: No valid data points for comparison")
                continue

            # Basic error metrics
            errors = model_clean - real_clean
            abs_errors = np.abs(errors)

            mae = np.mean(abs_errors)
            rmse = np.sqrt(np.mean(errors**2))
            max_error = np.max(abs_errors)
            min_error = np.min(abs_errors)

            # Percentile-based metrics
            p50_error = np.median(abs_errors)  # Median absolute error
            p95_error = np.percentile(abs_errors, 95)
            p99_error = np.percentile(abs_errors, 99)

            # Relative errors (avoid division by zero)
            real_nonzero_mask = real_clean != 0
            if np.any(real_nonzero_mask):
                rel_errors = abs_errors[real_nonzero_mask] / \
                    np.abs(real_clean[real_nonzero_mask])
                mape = np.mean(rel_errors) * 100
                median_ape = np.median(rel_errors) * 100
                max_ape = np.max(rel_errors) * 100
            else:
                mape = median_ape = max_ape = np.nan

            # Statistical metrics
            if len(model_clean) > 1:
                corr, p_value = pearsonr(model_clean, real_clean)

                # R-squared (coefficient of determination)
                ss_res = np.sum(errors**2)
                ss_tot = np.sum((real_clean - np.mean(real_clean))**2)
                r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else np.nan

                # Nash-Sutcliffe Efficiency
                nse = 1 - (ss_res / ss_tot) if ss_tot != 0 else np.nan

                # Index of Agreement (Willmott, 1981)
                numerator = np.sum(errors**2)
                denominator = np.sum((np.abs(model_clean - np.mean(real_clean)) +
                                      np.abs(real_clean - np.mean(real_clean)))**2)
                index_agreement = 1 - \
                    (numerator / denominator) if denominator != 0 else np.nan

                # Bias metrics
                mean_bias = np.mean(errors)
                bias_fraction = mean_bias / \
                    np.mean(real_clean) if np.mean(real_clean) != 0 else np.nan

                # Variance ratio
                var_ratio = np.var(
                    model_clean) / np.var(real_clean) if np.var(real_clean) != 0 else np.nan

                # Normalized metrics
                nrmse = rmse / (np.max(real_clean) - np.min(real_clean)) if (
                    np.max(real_clean) - np.min(real_clean)) != 0 else np.nan
                nmae = mae / \
                    np.mean(real_clean) if np.mean(real_clean) != 0 else np.nan

                # Skill metrics
                # Kling-Gupta Efficiency
                alpha = np.std(
                    model_clean) / np.std(real_clean) if np.std(real_clean) != 0 else np.nan
                beta = np.mean(
                    model_clean) / np.mean(real_clean) if np.mean(real_clean) != 0 else np.nan
                kge = 1 - np.sqrt((corr - 1)**2 + (alpha - 1)**2 + (beta - 1)**2) if not (
                    np.isnan(corr) or np.isnan(alpha) or np.isnan(beta)) else np.nan

            else:
                corr = p_value = r_squared = nse = index_agreement = np.nan
                mean_bias = bias_fraction = var_ratio = nrmse = nmae = kge = np.nan
                alpha = beta = np.nan

            # Threshold-based metrics (within X% accuracy)
            within_5pct = np.sum(rel_errors <= 0.05) / len(rel_errors) * \
                100 if np.any(real_nonzero_mask) else np.nan
            within_10pct = np.sum(rel_errors <= 0.10) / len(rel_errors) * \
                100 if np.any(real_nonzero_mask) else np.nan
            within_20pct = np.sum(rel_errors <= 0.20) / len(rel_errors) * \
                100 if np.any(real_nonzero_mask) else np.nan

            # Peak value comparison
            model_peak = np.max(model_clean)
            real_peak = np.max(real_clean)
            peak_error = np.abs(model_peak - real_peak)
            peak_error_pct = (peak_error / real_peak *
                              100) if real_peak != 0 else np.nan

            # Time to peak comparison (for parameters that have clear peaks)
            if param in ['Altitude_m', 'Velocity_ms']:
                model_peak_idx = np.argmax(model_clean)
                real_peak_idx = np.argmax(real_clean)
                model_peak_time = self.model_ascent['Time_s'].iloc[model_peak_idx]
                real_peak_time = self.real_interpolated['Time_s'].iloc[real_peak_idx]
                time_to_peak_error = np.abs(model_peak_time - real_peak_time)
            else:
                time_to_peak_error = np.nan

            # Store all metrics
            metrics[param] = {
                # Basic error metrics
                'MAE': mae,
                'RMSE': rmse,
                'Max Error': max_error,
                'Min Error': min_error,
                'Median Error': p50_error,
                '95th Percentile Error': p95_error,
                '99th Percentile Error': p99_error,

                # Relative error metrics
                'MAPE (%)': mape,
                'Median APE (%)': median_ape,
                'Max APE (%)': max_ape,

                # Normalized metrics
                'NRMSE': nrmse,
                'NMAE': nmae,

                # Statistical metrics
                'Correlation': corr,
                'P-value': p_value,
                'R-squared': r_squared,
                'Nash-Sutcliffe Efficiency': nse,
                'Index of Agreement': index_agreement,
                'Kling-Gupta Efficiency': kge,

                # Bias metrics
                'Mean Bias': mean_bias,
                'Bias Fraction': bias_fraction,
                'Variance Ratio': var_ratio,

                # Threshold metrics
                'Within 5% (%)': within_5pct,
                'Within 10% (%)': within_10pct,
                'Within 20% (%)': within_20pct,

                # Peak metrics
                'Peak Error': peak_error,
                'Peak Error (%)': peak_error_pct,
                'Time to Peak Error (s)': time_to_peak_error,

                # Metadata
                'Data Points': len(model_clean),
                'Model Peak': model_peak,
                'Real Peak': real_peak
            }

            # Print results with enhanced formatting
            units = {'Altitude_m': 'm', 'Velocity_ms': 'm/s',
                     'Acceleration_ms2': 'm/s²'}
            unit = units[param]

            print(f"\n{param.replace('_', ' ').title()}:")
            print(f"  ─── Basic Error Metrics ───")
            print(f"  Mean Absolute Error:        {mae:.3f} {unit}")
            print(f"  Root Mean Square Error:     {rmse:.3f} {unit}")
            print(f"  Maximum Error:              {max_error:.3f} {unit}")
            print(f"  Median Error:               {p50_error:.3f} {unit}")
            print(f"  95th Percentile Error:      {p95_error:.3f} {unit}")

            print(f"  ─── Relative Error Metrics ───")
            if not np.isnan(mape):
                print(f"  Mean Absolute Percentage Error: {mape:.1f}%")
                print(f"  Median Absolute Percentage Error: {median_ape:.1f}%")
                print(f"  Maximum Absolute Percentage Error: {max_ape:.1f}%")

            print(f"  ─── Statistical Metrics ───")
            if not np.isnan(corr):
                print(
                    f"  Correlation:                {corr:.3f} (p={p_value:.3e})")
                print(f"  R-squared:                  {r_squared:.3f}")
                print(f"  Nash-Sutcliffe Efficiency:  {nse:.3f}")
                print(f"  Index of Agreement:         {index_agreement:.3f}")
                print(f"  Kling-Gupta Efficiency:     {kge:.3f}")

            print(f"  ─── Bias Metrics ───")
            if not np.isnan(mean_bias):
                print(f"  Mean Bias:                  {mean_bias:.3f} {unit}")
                print(f"  Bias Fraction:              {bias_fraction:.3f}")
                print(f"  Variance Ratio:             {var_ratio:.3f}")

            print(f"  ─── Threshold Metrics ───")
            if not np.isnan(within_5pct):
                print(
                    f"  Within 5% accuracy:         {within_5pct:.1f}% of points")
                print(
                    f"  Within 10% accuracy:        {within_10pct:.1f}% of points")
                print(
                    f"  Within 20% accuracy:        {within_20pct:.1f}% of points")

            print(f"  ─── Peak Comparison ───")
            print(f"  Model Peak:                 {model_peak:.3f} {unit}")
            print(f"  Real Peak:                  {real_peak:.3f} {unit}")
            print(
                f"  Peak Error:                 {peak_error:.3f} {unit} ({peak_error_pct:.1f}%)")
            if not np.isnan(time_to_peak_error):
                print(
                    f"  Time to Peak Error:         {time_to_peak_error:.3f} s")

            print(f"  ─── Data Quality ───")
            print(f"  Valid Data Points:          {len(model_clean)}")

        self.metrics = metrics
        return metrics

    def plot_comparison(self, save_plots: bool = True):
        """
        Create enhanced comparison plots with additional visualizations
        """
        print("\nGenerating comprehensive comparison plots...")

        # Create a cleaner figure with better spacing
        fig = plt.figure(figsize=(16, 10))

        # Parameters to plot
        params = ['Altitude_m', 'Velocity_ms', 'Acceleration_ms2']
        titles = ['Altitude vs Time',
                  'Velocity vs Time', 'Acceleration vs Time']
        units = ['m', 'm/s', 'm/s²']
        colors = ['blue', 'green', 'red']

        # 1. Time series comparisons (top row)
        for i, (param, title, unit, color) in enumerate(zip(params, titles, units, colors)):
            ax = plt.subplot(2, 4, i + 1)

            # Plot data
            ax.plot(self.model_ascent['Time_s'], self.model_ascent[param],
                    color=color, linewidth=2, label='Model', alpha=0.8)
            ax.plot(self.real_interpolated['Time_s'], self.real_interpolated[param],
                    'k--', linewidth=2, label='Real Data', alpha=0.8)

            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'{param.split("_")[0].title()} ({unit})')
            ax.set_title(title)
            ax.legend()
            ax.grid(True, alpha=0.3)

        # 2. Statistical metrics plot in the fourth position of top row
        ax = plt.subplot(2, 4, 4)

        # Create a summary metrics bar chart
        metrics_data = []
        metric_names = []

        for param in params:
            if hasattr(self, 'metrics') and param in self.metrics:
                m = self.metrics[param]
                corr = m.get('Correlation', 0)
                r2 = m.get('R-squared', 0)
                nse = m.get('Nash-Sutcliffe Efficiency', 0)

                metrics_data.append([corr, r2, nse])
                metric_names.append(param.split('_')[0].title())

        if metrics_data:
            x = np.arange(len(metric_names))
            width = 0.25

            ax.bar(x - width, [m[0] for m in metrics_data],
                   width, label='Correlation', alpha=0.8)
            ax.bar(x, [m[1] for m in metrics_data],
                   width, label='R²', alpha=0.8)
            ax.bar(x + width, [m[2] for m in metrics_data],
                   width, label='NSE', alpha=0.8)

            ax.set_xlabel('Parameter')
            ax.set_ylabel('Metric Value')
            ax.set_title('Statistical Metrics')
            ax.set_xticks(x)
            ax.set_xticklabels(metric_names)
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.set_ylim(0, 1)

        # 3. Scatter plots with regression lines (bottom row)
        for i, (param, title, unit, color) in enumerate(zip(params, titles, units, colors)):
            ax = plt.subplot(2, 4, i + 5)

            model_vals = self.model_ascent[param].values
            real_vals = self.real_interpolated[param].values

            # Scatter plot
            ax.scatter(real_vals, model_vals, alpha=0.6, s=20, color=color)

            # Perfect correlation line
            min_val = min(np.min(real_vals), np.min(model_vals))
            max_val = max(np.max(real_vals), np.max(model_vals))
            ax.plot([min_val, max_val], [min_val, max_val],
                    'k--', alpha=0.7, label='Perfect Match')

            # Linear regression line
            if len(model_vals) > 1:
                from scipy import stats
                slope, intercept, r_value, p_value, std_err = stats.linregress(
                    real_vals, model_vals)
                line_x = np.array([min_val, max_val])
                line_y = slope * line_x + intercept
                ax.plot(line_x, line_y, color='red', linewidth=2, alpha=0.8,
                        label=f'Fit (R²={r_value**2:.3f})')

            ax.set_xlabel(f'Real {param.split("_")[0].title()} ({unit})')
            ax.set_ylabel(f'Model {param.split("_")[0].title()} ({unit})')
            ax.set_title(f'{param.split("_")[0].title()} Correlation')
            ax.legend()
            ax.grid(True, alpha=0.3)

        # 4. Error distribution histogram (bottom right)
        ax = plt.subplot(2, 4, 8)

        all_errors = []
        error_labels = []
        error_colors = []

        for i, (param, color) in enumerate(zip(params, colors)):
            errors = self.model_ascent[param].values - \
                self.real_interpolated[param].values
            # Normalize errors by the range of real data for comparison
            real_range = np.max(
                self.real_interpolated[param]) - np.min(self.real_interpolated[param])
            if real_range > 0:
                normalized_errors = errors / real_range * 100  # Convert to percentage
                all_errors.extend(normalized_errors)
                error_labels.extend(
                    [param.split('_')[0].title()] * len(normalized_errors))
                error_colors.extend([color] * len(normalized_errors))

        # Create histogram
        unique_labels = [param.split('_')[0].title() for param in params]
        unique_colors = colors

        for label, color in zip(unique_labels, unique_colors):
            label_errors = [err for err, lbl in zip(
                all_errors, error_labels) if lbl == label]
            if label_errors:
                ax.hist(label_errors, bins=20, alpha=0.6,
                        color=color, label=label, density=True)

        ax.axvline(x=0, color='black', linestyle='-', alpha=0.5)
        ax.set_xlabel('Normalized Error (%)')
        ax.set_ylabel('Density')
        ax.set_title('Error Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.suptitle('Flight Data Comparison Analysis', fontsize=16, y=0.95)
        plt.tight_layout(rect=[0, 0.03, 1, 0.93])

        if save_plots:
            plt.savefig('comprehensive_flight_comparison.png',
                        dpi=300, bbox_inches='tight')
            print("Enhanced plots saved as 'comprehensive_flight_comparison.png'")

        plt.show()

        # Create additional specialized plots
        self._create_residual_plots(save_plots)
        self._create_performance_dashboard(save_plots)

    def _create_residual_plots(self, save_plots: bool = True):
        """Create detailed residual analysis plots with improved spacing"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))

        params = ['Altitude_m', 'Velocity_ms', 'Acceleration_ms2']
        titles = ['Altitude', 'Velocity', 'Acceleration']
        units = ['m', 'm/s', 'm/s²']
        colors = ['blue', 'green', 'red']

        for i, (param, title, unit, color) in enumerate(zip(params, titles, units, colors)):
            model_vals = self.model_ascent[param].values
            real_vals = self.real_interpolated[param].values
            residuals = model_vals - real_vals

            # Residuals vs fitted values
            ax1 = axes[0, i]
            ax1.scatter(real_vals, residuals, alpha=0.6, color=color, s=20)
            ax1.axhline(y=0, color='black', linestyle='--', alpha=0.7)
            ax1.set_xlabel(f'Real {title} ({unit})')
            ax1.set_ylabel(f'Residuals ({unit})')
            ax1.set_title(f'{title} Residuals vs Fitted')
            ax1.grid(True, alpha=0.3)

            # Q-Q plot for normality assessment
            ax2 = axes[1, i]
            from scipy import stats
            stats.probplot(residuals, dist="norm", plot=ax2)
            ax2.set_title(f'{title} Residuals Q-Q Plot')
            ax2.grid(True, alpha=0.3)

        plt.suptitle('Residual Analysis', fontsize=16, y=0.95)
        plt.tight_layout(rect=[0, 0.03, 1, 0.92])

        if save_plots:
            plt.savefig('residual_analysis.png', dpi=300, bbox_inches='tight')
            print("Residual analysis saved as 'residual_analysis.png'")

        plt.show()

    def _create_performance_dashboard(self, save_plots: bool = True):
        """Create a performance dashboard with key metrics"""
        fig = plt.figure(figsize=(16, 10))

        # Calculate key metrics for dashboard
        params = ['Altitude_m', 'Velocity_ms', 'Acceleration_ms2']
        param_names = ['Altitude', 'Velocity', 'Acceleration']

        # Prepare data for dashboard
        correlations = []
        r_squared = []
        mape_values = []
        peak_errors = []

        for param in params:
            if hasattr(self, 'metrics') and param in self.metrics:
                m = self.metrics[param]
                correlations.append(m.get('Correlation', 0))
                r_squared.append(m.get('R-squared', 0))
                mape_values.append(m.get('MAPE (%)', 0))
                peak_errors.append(m.get('Peak Error (%)', 0))
            else:
                correlations.append(0)
                r_squared.append(0)
                mape_values.append(0)
                peak_errors.append(0)

        # 1. Correlation radar chart
        ax1 = plt.subplot(2, 3, 1, projection='polar')
        angles = np.linspace(0, 2 * np.pi, len(param_names),
                             endpoint=False).tolist()
        angles += angles[:1]  # Complete the circle
        correlations_plot = correlations + correlations[:1]

        ax1.plot(angles, correlations_plot, 'bo-', linewidth=2, markersize=6)
        ax1.fill(angles, correlations_plot, alpha=0.25, color='blue')
        ax1.set_xticks(angles[:-1])
        ax1.set_xticklabels(param_names)
        ax1.set_ylim(0, 1)
        ax1.set_title('Correlation Scores', pad=20)
        ax1.grid(True)

        # 2. R-squared bar chart
        ax2 = plt.subplot(2, 3, 2)
        bars = ax2.bar(param_names, r_squared, color=[
                       'blue', 'green', 'red'], alpha=0.7)
        ax2.set_ylabel('R-squared')
        ax2.set_title('R-squared Values')
        ax2.set_ylim(0, 1)
        ax2.grid(True, alpha=0.3)

        # Add value labels on bars
        for bar, val in zip(bars, r_squared):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                     f'{val:.3f}', ha='center', va='bottom')

        # 3. MAPE comparison
        ax3 = plt.subplot(2, 3, 3)
        bars = ax3.bar(param_names, mape_values, color=[
                       'blue', 'green', 'red'], alpha=0.7)
        ax3.set_ylabel('MAPE (%)')
        ax3.set_title('Mean Absolute Percentage Error')
        ax3.grid(True, alpha=0.3)

        # Add value labels on bars
        for bar, val in zip(bars, mape_values):
            height = bar.get_height()
            ax3.text(bar.get_x() + bar.get_width()/2., height + max(mape_values) * 0.01,
                     f'{val:.1f}%', ha='center', va='bottom')

        # 4. Peak error comparison
        ax4 = plt.subplot(2, 3, 4)
        bars = ax4.bar(param_names, peak_errors, color=[
                       'blue', 'green', 'red'], alpha=0.7)
        ax4.set_ylabel('Peak Error (%)')
        ax4.set_title('Peak Value Errors')
        ax4.grid(True, alpha=0.3)

        # Add value labels on bars
        for bar, val in zip(bars, peak_errors):
            height = bar.get_height()
            ax4.text(bar.get_x() + bar.get_width()/2., height + max(peak_errors) * 0.01,
                     f'{val:.1f}%', ha='center', va='bottom')

        # 5. Overall performance gauge
        ax5 = plt.subplot(2, 3, 5)
        overall_score = np.mean(correlations)

        # Create a simple gauge
        theta = np.linspace(0, np.pi, 100)
        r = np.ones_like(theta)

        # Background arc
        ax5.plot(theta, r, 'lightgray', linewidth=20)

        # Performance arc
        score_theta = np.linspace(
            0, overall_score * np.pi, int(overall_score * 100))
        score_r = np.ones_like(score_theta)

        if overall_score > 0.8:
            color = 'green'
        elif overall_score > 0.6:
            color = 'orange'
        else:
            color = 'red'

        ax5.plot(score_theta, score_r, color=color, linewidth=20)
        ax5.set_ylim(0, 1.2)
        ax5.set_xlim(-0.1, np.pi + 0.1)
        ax5.set_title(f'Overall Score: {overall_score:.3f}')
        ax5.set_xticks([0, np.pi/2, np.pi])
        ax5.set_xticklabels(['0', '0.5', '1.0'])
        ax5.set_yticks([])

        # 6. Performance summary text
        ax6 = plt.subplot(2, 3, 6)
        ax6.axis('off')

        summary_text = f"""
Model Performance Summary:

Average Correlation: {np.mean(correlations):.3f}
Average R-squared: {np.mean(r_squared):.3f}
Average MAPE: {np.mean(mape_values):.1f}%

Best Parameter: {param_names[np.argmax(correlations)]}
Worst Parameter: {param_names[np.argmin(correlations)]}
        """

        ax6.text(0.1, 0.9, summary_text, transform=ax6.transAxes,
                 fontsize=11, verticalalignment='top', fontfamily='monospace',
                 bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))

        plt.suptitle('Model Performance Dashboard', fontsize=16)
        plt.tight_layout()

        if save_plots:
            plt.savefig('performance_dashboard.png',
                        dpi=300, bbox_inches='tight')
            print("Performance dashboard saved as 'performance_dashboard.png'")

        plt.show()

    def generate_report(self):
        """
        Generate a comprehensive comparison report
        """
        metrics = self.calculate_metrics()
        self.plot_comparison()

        print("\n" + "="*60)
        print("MODEL PERFORMANCE SUMMARY")
        print("="*60)

        # Extract key metrics for overall assessment
        params = ['Altitude_m', 'Velocity_ms', 'Acceleration_ms2']
        param_names = ['Altitude', 'Velocity', 'Acceleration']

        print(f"\n{'Parameter':<15} {'Correlation':<12} {'R²':<8} {'NSE':<8} {'KGE':<8} {'MAPE':<8} {'Peak Err':<10}")
        print("-" * 75)

        overall_scores = {}

        for param, name in zip(params, param_names):
            if param in metrics:
                m = metrics[param]
                corr = m.get('Correlation', np.nan)
                r2 = m.get('R-squared', np.nan)
                nse = m.get('Nash-Sutcliffe Efficiency', np.nan)
                kge = m.get('Kling-Gupta Efficiency', np.nan)
                mape = m.get('MAPE (%)', np.nan)
                peak_err = m.get('Peak Error (%)', np.nan)

                print(
                    f"{name:<15} {corr:>8.3f}    {r2:>6.3f}  {nse:>6.3f}  {kge:>6.3f}  {mape:>6.1f}%  {peak_err:>8.1f}%")

                # Store for overall assessment
                overall_scores[param] = {
                    'correlation': corr,
                    'r_squared': r2,
                    'nse': nse,
                    'kge': kge,
                    'mape': mape
                }

        # Calculate overall performance scores
        print(f"\n{'OVERALL PERFORMANCE ASSESSMENT'}")
        print("-" * 40)

        # Average correlation across all parameters
        avg_corr = np.nanmean([overall_scores[p]['correlation']
                              for p in overall_scores])
        avg_r2 = np.nanmean([overall_scores[p]['r_squared']
                            for p in overall_scores])
        avg_nse = np.nanmean([overall_scores[p]['nse']
                             for p in overall_scores])
        avg_kge = np.nanmean([overall_scores[p]['kge']
                             for p in overall_scores])
        avg_mape = np.nanmean([overall_scores[p]['mape']
                              for p in overall_scores])

        print(f"Average Correlation:           {avg_corr:.3f}")
        print(f"Average R-squared:             {avg_r2:.3f}")
        print(f"Average Nash-Sutcliffe Eff:    {avg_nse:.3f}")
        print(f"Average Kling-Gupta Eff:       {avg_kge:.3f}")
        print(f"Average MAPE:                  {avg_mape:.1f}%")

        # Detailed parameter-specific assessment
        print(f"\n{'PARAMETER-SPECIFIC INSIGHTS'}")
        print("-" * 40)

        for param, name in zip(params, param_names):
            if param in metrics:
                m = metrics[param]
                print(f"\n{name}:")

                # Best performing metrics
                strengths = []
                weaknesses = []

                corr = m.get('Correlation', np.nan)
                if corr > 0.9:
                    strengths.append(f"High correlation ({corr:.3f})")
                elif corr < 0.7:
                    weaknesses.append(f"Low correlation ({corr:.3f})")

                mape = m.get('MAPE (%)', np.nan)
                if not np.isnan(mape):
                    if mape < 10:
                        strengths.append(f"Low MAPE ({mape:.1f}%)")
                    elif mape > 25:
                        weaknesses.append(f"High MAPE ({mape:.1f}%)")

                within_10 = m.get('Within 10% (%)', np.nan)
                if not np.isnan(within_10):
                    if within_10 > 80:
                        strengths.append(
                            f"Good accuracy ({within_10:.0f}% within 10%)")
                    elif within_10 < 50:
                        weaknesses.append(
                            f"Poor accuracy ({within_10:.0f}% within 10%)")

                peak_err = m.get('Peak Error (%)', np.nan)
                if not np.isnan(peak_err):
                    if peak_err < 5:
                        strengths.append(
                            f"Accurate peak prediction ({peak_err:.1f}% error)")
                    elif peak_err > 15:
                        weaknesses.append(
                            f"Inaccurate peak prediction ({peak_err:.1f}% error)")

                bias = m.get('Mean Bias', np.nan)
                if not np.isnan(bias):
                    if abs(bias) < 0.1 * np.abs(m.get('Real Peak', 1)):
                        strengths.append("Low systematic bias")
                    else:
                        bias_direction = "over-predicting" if bias > 0 else "under-predicting"
                        weaknesses.append(
                            f"Systematic bias ({bias_direction})")

                if strengths:
                    print(f"  Strengths: {', '.join(strengths)}")
                if weaknesses:
                    print(f"  Areas for improvement: {', '.join(weaknesses)}")
                if not strengths and not weaknesses:
                    print(f"  Performance: Moderate")

        # Model validation recommendations
        print(f"\n{'MODEL VALIDATION RECOMMENDATIONS'}")
        print("-" * 40)

        recommendations = []

        if avg_corr < 0.8:
            recommendations.append(
                "• Review model physics - low correlation suggests systematic differences")

        if avg_mape > 20:
            recommendations.append(
                "• Consider parameter calibration - high MAPE indicates significant quantitative differences")

        # Check for systematic bias
        alt_bias = metrics.get('Altitude_m', {}).get('Mean Bias', 0)
        vel_bias = metrics.get('Velocity_ms', {}).get('Mean Bias', 0)

        if abs(alt_bias) > 10:  # More than 10m bias
            if alt_bias > 0:
                recommendations.append(
                    "• Model over-predicts altitude - check drag coefficients or atmospheric model")
            else:
                recommendations.append(
                    "• Model under-predicts altitude - check thrust curve or mass properties")

        if abs(vel_bias) > 5:  # More than 5 m/s bias
            if vel_bias > 0:
                recommendations.append(
                    "• Model over-predicts velocity - review aerodynamic drag modeling")
            else:
                recommendations.append(
                    "• Model under-predicts velocity - verify motor performance or mass flow")

        # Check peak errors
        for param, name in zip(params, param_names):
            if param in metrics:
                peak_err = metrics[param].get('Peak Error (%)', np.nan)
                if not np.isnan(peak_err) and peak_err > 15:
                    recommendations.append(
                        f"• Large {name.lower()} peak error ({peak_err:.1f}%) - review critical flight phase modeling")

        if not recommendations:
            recommendations.append(
                "• Model performance is satisfactory - consider validation with additional flight data")

        for rec in recommendations:
            print(rec)

        return metrics


def main():
    """
    Main function to run the comparison
    """
    # File paths
    model_file = "model_results.csv"
    data_file = "data_results.csv"

    try:
        # Create comparator
        comparator = FlightDataComparator(model_file, data_file)

        # Run comparison
        comparator.load_data()
        comparator.normalize_model_data()
        comparator.interpolate_data()

        # Generate report
        metrics = comparator.generate_report()

        print("\nComparison completed successfully!")

    except Exception as e:
        print(f"Error during comparison: {e}")
        print("\nPlease check that both CSV files exist and have the correct format:")
        print("Required columns: Time_s, Altitude_m, Velocity_ms, Acceleration_ms2")


if __name__ == "__main__":
    main()
