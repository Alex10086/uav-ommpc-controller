#!/usr/bin/env python3
"""Tracking errors on already aligned PX4-native samples; no clock/lag fitting.

analyze_phase(data, output_prefix) writes *_errors.csv and *_peak.png/.pdf.
All vectors must share one coordinate frame. Position error is reference minus
actual: positive tangent error means behind the reference. Cross-track is the
magnitude of the perpendicular 3-D vector, not a signed planar lateral error.
The first supplied sample defines phase-relative zero. RMSE is sample-weighted,
not time-weighted. Optional inputs must be finite on every supplied sample;
omit unavailable channels rather than filling them or extrapolating.
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


MOVING_SPEED_M_S = 0.05
_REQUIRED_VECTORS = ('reference_position', 'reference_velocity', 'position', 'velocity')
_OPTIONAL_VECTORS = ('actual_rates', 'desired_rates', 'attitude_error', 'model_residual', 'estimation_error')


def _validated(data):
    time = np.asarray(data['time_s'], dtype=float)
    if time.ndim != 1 or not len(time) or not np.all(np.isfinite(time)):
        raise ValueError('time_s must be a nonempty finite 1-D array')
    if np.any(time[1:] <= time[:-1]):
        raise ValueError('time_s must be strictly increasing')
    arrays = {'time_s': time}
    for name in _REQUIRED_VECTORS + _OPTIONAL_VECTORS + ('state_age_ms',):
        if name not in data:
            if name in _REQUIRED_VECTORS:
                raise ValueError(f'missing required channel: {name}')
            continue
        value = np.asarray(data[name], dtype=float)
        shape = (len(time),) if name == 'state_age_ms' else (len(time), 3)
        if value.shape != shape or not np.all(np.isfinite(value)):
            raise ValueError(f'{name} must be finite with shape {shape}')
        arrays[name] = value
    return arrays


def _norm(vectors):
    return np.hypot.reduce(vectors, axis=1)


def _rmse(values):
    """Avoid empty means and unnecessary overflow from squaring large errors."""
    if not len(values):
        return None
    scale = float(np.max(np.abs(values)))
    return scale * float(np.sqrt(np.mean((values / scale) ** 2))) if scale else 0.0


def _nullable(value):
    value = np.asarray(value)
    if value.ndim:
        return [_nullable(item) for item in value]
    return float(value) if np.isfinite(value) else None


def _statistics(error, total, velocity_error, tangent, cross, mask):
    count = int(np.count_nonzero(mask))
    direction_defined = mask & np.isfinite(tangent)
    return {
        'sample_count': count,
        'position_rmse_m': _rmse(total[mask]),
        'position_axis_rmse_m': [_rmse(error[mask, i]) for i in range(3)],
        'velocity_rmse_m_s': _rmse(_norm(velocity_error[mask])),
        'velocity_axis_rmse_m_s': [_rmse(velocity_error[mask, i]) for i in range(3)],
        'direction_defined_sample_count': int(np.count_nonzero(direction_defined)),
        'tangent_rmse_m': _rmse(tangent[direction_defined]),
        'tangent_mean_m': float(np.mean(tangent[direction_defined])) if np.any(direction_defined) else None,
        'cross_track_rmse_m': _rmse(cross[direction_defined]),
    }


def _write_csv(path, arrays, relative, error, velocity_error, total, moving, tangent, cross_vector, cross):
    columns = {'time_s': arrays['time_s'], 'phase_time_s': relative}
    for name in _REQUIRED_VECTORS + _OPTIONAL_VECTORS:
        if name in arrays:
            for i, axis in enumerate('xyz'):
                columns[f'{name}_{axis}'] = arrays[name][:, i]
    for name, values in (('position_error_m', error), ('velocity_error_m_s', velocity_error),
                         ('cross_track_vector_m', cross_vector)):
        for i, axis in enumerate('xyz'):
            columns[f'{name}_{axis}'] = values[:, i]
    columns.update(position_error_m=total, moving=moving.astype(int),
                   tangent_error_m=tangent, cross_track_error_m=cross)
    if 'state_age_ms' in arrays:
        columns['state_age_ms'] = arrays['state_age_ms']
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        # Python float conversion retains round-trip precision. Undefined
        # stationary decompositions are empty CSV cells, never fabricated zeros.
        for row in zip(*columns.values()):
            writer.writerow([value if np.isfinite(value) else '' for value in row])


def _write_plots(paths, arrays, relative, error, total, tangent, cross, peak, window, coverage):
    panels = [
        ('Position [m]', [('reference', arrays['reference_position'], '--'),
                          ('actual', arrays['position'], '-')]),
        ('Velocity [m/s]', [('reference', arrays['reference_velocity'], '--'),
                           ('actual', arrays['velocity'], '-')]),
        ('Position error [m]', [('total', total, '-'), ('tangent (signed)', tangent, '-'),
                                ('cross-track (norm)', cross, '-')]),
        ('Axis error [m]', [('reference - actual', error, '-')]),
    ]
    rates = [(label, arrays[name], style) for name, label, style in
             (('actual_rates', 'actual', '-'), ('desired_rates', 'desired', '--')) if name in arrays]
    if rates:
        panels.append(('Body rates [rad/s]', rates))
    for name, label in (('attitude_error', 'Attitude error [rad]'),
                        ('state_age_ms', 'State age [ms]'),
                        ('estimation_error', 'EKF estimate - truth at sample time [m]'),
                        ('model_residual', 'Lateral dynamics residual [m/s²]')):
        if name in arrays:
            panels.append((label, [(name, arrays[name], '-')]))
    fig, axes = plt.subplots(len(panels), 2, figsize=(15, 2.6 * len(panels)),
                             squeeze=False, sharex='col')
    try:
        for row, (ylabel, series) in enumerate(panels):
            for column, mask in enumerate((np.ones(len(relative), dtype=bool), window)):
                ax = axes[row, column]
                for series_index, (label, values, style) in enumerate(series):
                    if values.ndim == 2:
                        for i, axis in enumerate('xyz'):
                            ax.plot(relative[mask], values[mask, i], linestyle=style,
                                    color=f'C{i}', marker='.', markersize=2,
                                    label=f'{label} {axis}')
                    else:
                        ax.plot(relative[mask], values[mask], linestyle=style,
                                color=f'C{series_index}', marker='.', markersize=2, label=label)
                ax.axvline(relative[peak], color='black', linestyle=':', linewidth=1)
                ax.set_ylabel(ylabel)
                ax.grid(True, alpha=0.3)
                ax.legend(loc='best', fontsize=7, ncol=2)
                selected = relative[mask]
                if selected[-1] > selected[0]:
                    ax.set_xlim(selected[0], selected[-1])
        axes[0, 0].set_title('Full phase (native-clock alignment; no fitted shift)')
        axes[0, 1].set_title(
            f'First maximum ±1 s: observed -{coverage["before_peak_s"]:.3f}/'
            f'+{coverage["after_peak_s"]:.3f} s; {coverage["sample_count"]} samples')
        for ax in axes[-1]:
            ax.set_xlabel('Time from first phase sample [s]')
        fig.suptitle(f'First maximum position error: {total[peak]:.4f} m at '
                     f'{relative[peak]:.6f} s (native {arrays["time_s"][peak]:.9f} s)')
        fig.tight_layout(rect=(0, 0, 1, 0.98))
        for path in paths:
            fig.savefig(path, dpi=160)
    finally:
        plt.close(fig)


def analyze_phase(data, output_prefix):
    """Validate aligned samples, export errors and return JSON-safe metrics.

    Required: time_s (N,), reference_position/reference_velocity/position/velocity
    (N,3). Optional: actual_rates/desired_rates/attitude_error/model_residual
    (N,3), state_age_ms (N,). N >= 1; native times strictly increase. Stationary
    samples (reference speed <= 0.05 m/s) retain total/axis errors but have no
    tangent or cross-track decomposition. Maxima include stationary samples.
    """
    arrays = _validated(data)
    time = arrays['time_s']
    with np.errstate(over='raise', invalid='raise', divide='raise'):
        try:
            relative = time - time[0]
            error = arrays['reference_position'] - arrays['position']
            velocity_error = arrays['reference_velocity'] - arrays['velocity']
            total = _norm(error)
            speed = _norm(arrays['reference_velocity'])
            moving = speed > MOVING_SPEED_M_S
            tangent = np.full(len(time), np.nan)
            cross_vector = np.full(error.shape, np.nan)
            direction = arrays['reference_velocity'][moving] / speed[moving, None]
            tangent[moving] = np.sum(error[moving] * direction, axis=1)
            cross_vector[moving] = error[moving] - tangent[moving, None] * direction
            cross = _norm(cross_vector)
            full = _statistics(error, total, velocity_error, tangent, cross, np.ones(len(time), dtype=bool))
            moving_stats = _statistics(error, total, velocity_error, tangent, cross, moving)
        except FloatingPointError as exc:
            raise ValueError('input magnitude exceeds finite diagnostic arithmetic') from exc
    peak = int(np.argmax(total))  # First occurrence, including stationary samples.
    requested_start = float(time[peak] - 1.0)
    requested_end = float(time[peak] + 1.0)
    window = (time >= requested_start) & (time <= requested_end)
    observed = time[window]
    coverage = {
        'requested_native_start_s': requested_start,
        'requested_native_end_s': requested_end,
        'observed_native_start_s': float(observed[0]),
        'observed_native_end_s': float(observed[-1]),
        'observed_phase_start_s': float(relative[window][0]),
        'observed_phase_end_s': float(relative[window][-1]),
        'before_peak_s': float(time[peak] - observed[0]),
        'after_peak_s': float(observed[-1] - time[peak]),
        'sample_count': int(np.count_nonzero(window)),
        'clipped_at_phase_start': bool(time[0] > requested_start),
        'clipped_at_phase_end': bool(time[-1] < requested_end),
        'max_sample_gap_s': float(np.max(np.diff(observed))) if len(observed) > 1 else None,
    }
    peak_metrics = {
        'sample_index': peak,
        'native_time_s': float(time[peak]),
        'phase_time_s': float(relative[peak]),
        'position_error_m': float(total[peak]),
        'position_error_xyz_m': _nullable(error[peak]),
        'velocity_error_xyz_m_s': _nullable(velocity_error[peak]),
        'tangent_error_m': _nullable(tangent[peak]),
        'cross_track_error_m': _nullable(cross[peak]),
        'cross_track_vector_m': _nullable(cross_vector[peak]),
        'moving': bool(moving[peak]),
    }
    for name in _REQUIRED_VECTORS + _OPTIONAL_VECTORS + ('state_age_ms',):
        if name in arrays:
            peak_metrics[name] = _nullable(arrays[name][peak])
    prefix = Path(output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    csv_path = Path(f'{prefix}_errors.csv')
    png_path, pdf_path = Path(f'{prefix}_peak.png'), Path(f'{prefix}_peak.pdf')
    _write_csv(csv_path, arrays, relative, error, velocity_error, total, moving, tangent, cross_vector, cross)
    _write_plots((png_path, pdf_path), arrays, relative, error, total, tangent, cross, peak, window, coverage)
    return {
        'sample_count': len(time),
        'native_start_s': float(time[0]),
        'native_end_s': float(time[-1]),
        'duration_s': float(relative[-1]),
        'moving_speed_threshold_m_s': MOVING_SPEED_M_S,
        'error_convention': 'reference_minus_actual; positive_tangent_means_behind',
        'rmse_weighting': 'equal_per_sample',
        'full': full,
        'moving': moving_stats,
        'peak': peak_metrics,
        'peak_window': coverage,
        'outputs': {'errors_csv': str(csv_path), 'peak_png': str(png_path), 'peak_pdf': str(pdf_path)},
    }
