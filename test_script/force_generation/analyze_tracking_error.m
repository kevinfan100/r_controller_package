% analyze_tracking_error.m
% Analysis script for Vd-Vm tracking error in hardware mode
%
% This script analyzes where and why the maximum tracking error occurs
% in the force generation test with Simulink hardware mode.
%
% Run this after run_force_generation_test.m to use its workspace variables.

fprintf('\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('        Vd-Vm Tracking Error Analysis (Hardware Mode)\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% Check required variables exist
required_vars = {'t', 'vd', 'vm', 'f_d_interp', 'force_frequency', 'Ts', ...
                 'idx_start', 'sample_rate_mode'};
missing_vars = {};
for i = 1:length(required_vars)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', required_vars{i}))
        missing_vars{end+1} = required_vars{i}; %#ok<SAGROW>
    end
end
if ~isempty(missing_vars)
    error('Missing variables: %s\nPlease run run_force_generation_test.m first.', ...
          strjoin(missing_vars, ', '));
end

% Import variables from base workspace
t = evalin('base', 't');
vd = evalin('base', 'vd');
vm = evalin('base', 'vm');
f_d_interp = evalin('base', 'f_d_interp');
force_frequency = evalin('base', 'force_frequency');
Ts = evalin('base', 'Ts');
idx_start = evalin('base', 'idx_start');
sample_rate_mode = evalin('base', 'sample_rate_mode');

% Optional variables
if evalin('base', 'exist(''force_amplitude'', ''var'')')
    force_amplitude = evalin('base', 'force_amplitude');
else
    force_amplitude = max(abs(f_d_interp(:)));
end

%% Configuration
analysis_channel = 1;  % Which channel to analyze (1-6)
zoom_half_width_cycles = 0.15;  % Half-width of zoom window in cycles

% Rate transition mode labels
mode_labels = {'ZOH (1600 Hz)', 'Linear (1600 Hz)', 'Direct (100 kHz)'};
fprintf('【Configuration】\n');
fprintf('  Analysis channel: P%d\n', analysis_channel);
fprintf('  Signal frequency: %.1f Hz\n', force_frequency);
fprintf('  Sample rate mode: %d (%s)\n', sample_rate_mode, mode_labels{sample_rate_mode});
fprintf('\n');


%% Section 1: Find Maximum Error Location

fprintf('【Section 1: Maximum Error Analysis】\n');
fprintf('────────────────────────────────────\n');

% Calculate tracking error for analysis channel (steady-state only)
voltage_error = vd - vm;
ve_ch = voltage_error(idx_start:end, analysis_channel);
t_steady = t(idx_start:end);
vd_steady = vd(idx_start:end, analysis_channel);
vm_steady = vm(idx_start:end, analysis_channel);

% Find maximum absolute error
[max_error_abs, max_error_idx_rel] = max(abs(ve_ch));
max_error_idx = idx_start + max_error_idx_rel - 1;  % Global index
t_max_error = t(max_error_idx);
max_error_signed = ve_ch(max_error_idx_rel);

fprintf('  Maximum |error|: %.4f mV at t = %.4f ms\n', max_error_abs*1000, t_max_error*1000);
fprintf('  Signed error: %.4f mV (vd - vm)\n', max_error_signed*1000);

% Calculate signal phase at max error point
T_period = 1 / force_frequency;
phase_at_max = mod(t_max_error, T_period) / T_period * 360;
fprintf('  Phase at max error: %.1f deg (within period)\n', phase_at_max);

% Calculate derivative of vd at max error point
if max_error_idx > 1 && max_error_idx < length(t)
    dvd_dt = (vd(max_error_idx+1, analysis_channel) - vd(max_error_idx-1, analysis_channel)) / (2*Ts);
    fprintf('  dVd/dt at max error: %.2f V/s\n', dvd_dt);
end

% Find vd value at max error (to determine position in waveform)
vd_at_max = vd(max_error_idx, analysis_channel);
vd_max_overall = max(vd_steady);
vd_min_overall = min(vd_steady);
vd_range = vd_max_overall - vd_min_overall;
vd_normalized = (vd_at_max - vd_min_overall) / vd_range;  % 0 to 1

fprintf('  Vd at max error: %.4f V (%.1f%% of range)\n', vd_at_max, vd_normalized*100);

% Determine waveform position
if vd_normalized < 0.25
    position_desc = 'near negative peak → rising edge';
elseif vd_normalized < 0.5
    position_desc = 'rising edge → approaching zero crossing';
elseif vd_normalized < 0.75
    position_desc = 'zero crossing → approaching positive peak';
else
    position_desc = 'near positive peak → falling edge';
end

% More precise: check derivative sign
if exist('dvd_dt', 'var')
    if dvd_dt > 0
        edge_type = 'RISING edge';
    else
        edge_type = 'FALLING edge';
    end
    fprintf('  Waveform position: %s (%s)\n', edge_type, position_desc);
end

fprintf('\n');


%% Section 2: Theoretical Analysis

fprintf('【Section 2: Theoretical Analysis】\n');
fprintf('──────────────────────────────────\n');

% For a sinusoidal signal through a first-order system:
% - Phase lag causes max error at zero-crossing (max slope)
% - Error magnitude ≈ A × sin(phase_lag) for small phase lag

% Calculate expected phase lag from 1600 Hz sampling
Ts_1600 = 1/1600;  % 625 µs
delay_1600_deg = Ts_1600 / T_period * 360;  % Delay in degrees
fprintf('  1600 Hz sample period: %.1f µs\n', Ts_1600*1e6);
fprintf('  Max delay from 1600 Hz: %.1f deg @ %.1f Hz\n', delay_1600_deg, force_frequency);

% For linear interpolation: average delay is 0.5 × Ts_1600
if sample_rate_mode == 2  % Linear
    avg_delay_us = Ts_1600 / 2 * 1e6;
    avg_delay_deg = delay_1600_deg / 2;
    fprintf('  Linear interp avg delay: %.1f µs (%.1f deg)\n', avg_delay_us, avg_delay_deg);
elseif sample_rate_mode == 1  % ZOH
    fprintf('  ZOH mode: delay varies 0 ~ %.1f µs\n', Ts_1600*1e6);
end

% Calculate where zero-crossings occur in a period
fprintf('\n  Sinusoidal signal characteristics:\n');
fprintf('    Zero crossings at: 0°, 180° (max dV/dt)\n');
fprintf('    Peaks at: 90°, 270° (dV/dt = 0)\n');
fprintf('    Max error expected near: %.1f° or %.1f°\n', ...
        90 - avg_delay_deg, 270 - avg_delay_deg);

fprintf('\n');


%% Section 3: Create Zoom Plot Around Max Error

fprintf('【Section 3: Generate Zoom Plot】\n');
fprintf('────────────────────────────────\n');

% Calculate zoom window
zoom_half_width_s = zoom_half_width_cycles * T_period;
t_zoom_start = max(t(1), t_max_error - zoom_half_width_s);
t_zoom_end = min(t(end), t_max_error + zoom_half_width_s);

idx_zoom = (t >= t_zoom_start) & (t <= t_zoom_end);
t_zoom = t(idx_zoom);
vd_zoom = vd(idx_zoom, analysis_channel);
vm_zoom = vm(idx_zoom, analysis_channel);
ve_zoom = voltage_error(idx_zoom, analysis_channel);

fprintf('  Zoom window: %.3f ~ %.3f ms (±%.2f cycles)\n', ...
        t_zoom_start*1000, t_zoom_end*1000, zoom_half_width_cycles);
fprintf('  Points in window: %d\n', sum(idx_zoom));

% Create figure with 3 subplots
fig = figure('Position', [100 100 1400 900], 'Name', 'Tracking Error Analysis');

% Color scheme
color_vd = [0 0.4470 0.7410];      % Blue
color_vm = [0.8500 0.3250 0.0980]; % Orange
color_error = [0.4660 0.6740 0.1880]; % Green

%% Subplot 1: Vd and Vm overlay (zoomed)
subplot(3, 1, 1);
hold on;

% Plot Vd and Vm
h_vd = plot(t_zoom*1000, vd_zoom*1000, '-', 'Color', color_vd, 'LineWidth', 2.5, ...
            'DisplayName', 'Vd (desired)');
h_vm = plot(t_zoom*1000, vm_zoom*1000, '--', 'Color', color_vm, 'LineWidth', 2.5, ...
            'DisplayName', 'Vm (measured)');

% Mark max error point
plot(t_max_error*1000, vd(max_error_idx, analysis_channel)*1000, 'ko', ...
     'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 2);
plot(t_max_error*1000, vm(max_error_idx, analysis_channel)*1000, 'ko', ...
     'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 2);

% Add vertical line at max error
xline(t_max_error*1000, 'k:', 'LineWidth', 1.5);

% Add 1600 Hz boundary markers
Ts_1600 = 1/1600;
t_boundaries = (floor(t_zoom_start/Ts_1600):ceil(t_zoom_end/Ts_1600)) * Ts_1600;
t_boundaries = t_boundaries(t_boundaries >= t_zoom_start & t_boundaries <= t_zoom_end);
for tb = t_boundaries
    xline(tb*1000, 'r:', 'LineWidth', 1, 'Alpha', 0.5);
end

xlabel('Time [ms]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Voltage [mV]', 'FontSize', 14, 'FontWeight', 'bold');
title(sprintf('P%d: Vd vs Vm (Zoom around max error at t=%.3f ms)', ...
              analysis_channel, t_max_error*1000), ...
      'FontSize', 16, 'FontWeight', 'bold');
legend([h_vd, h_vm], 'Location', 'best', 'FontSize', 12);
grid on;
box on;
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

%% Subplot 2: Tracking Error (zoomed)
subplot(3, 1, 2);
hold on;

plot(t_zoom*1000, ve_zoom*1000, '-', 'Color', color_error, 'LineWidth', 2.5);

% Mark max error point
plot(t_max_error*1000, max_error_signed*1000, 'ko', ...
     'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 2);

% Zero line
yline(0, 'k--', 'LineWidth', 1);

% Vertical line at max error
xline(t_max_error*1000, 'k:', 'LineWidth', 1.5);

% 1600 Hz boundary markers
for tb = t_boundaries
    xline(tb*1000, 'r:', 'LineWidth', 1, 'Alpha', 0.5);
end

xlabel('Time [ms]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Error (Vd - Vm) [mV]', 'FontSize', 14, 'FontWeight', 'bold');
title(sprintf('Tracking Error: Max = %.3f mV @ t=%.3f ms', ...
              max_error_signed*1000, t_max_error*1000), ...
      'FontSize', 16, 'FontWeight', 'bold');
grid on;
box on;
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

%% Subplot 3: Vm vs Vd (Lissajous, zoomed window only)
subplot(3, 1, 3);
hold on;

% Plot Vm vs Vd trajectory
plot(vd_zoom*1000, vm_zoom*1000, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 2.5);

% Mark max error point with both Vd and Vm values
plot(vd(max_error_idx, analysis_channel)*1000, ...
     vm(max_error_idx, analysis_channel)*1000, 'ko', ...
     'MarkerSize', 14, 'MarkerFaceColor', 'r', 'LineWidth', 2);

% Reference line (y = x)
lims = [min([vd_zoom; vm_zoom]), max([vd_zoom; vm_zoom])] * 1000;
plot(lims, lims, 'k--', 'LineWidth', 2);

% Add arrow showing error direction at max error point
vd_pt = vd(max_error_idx, analysis_channel)*1000;
vm_pt = vm(max_error_idx, analysis_channel)*1000;
% Error vector: from actual (vm) to desired (vd) on the y=x line
quiver(vd_pt, vm_pt, 0, vd_pt - vm_pt, 0, 'r', 'LineWidth', 2, ...
       'MaxHeadSize', 0.5, 'AutoScale', 'off');

xlabel('Vd [mV]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Vm [mV]', 'FontSize', 14, 'FontWeight', 'bold');
title(sprintf('Vm vs Vd (%.1f cycles window) - Red dot: max error point', ...
              2*zoom_half_width_cycles), ...
      'FontSize', 16, 'FontWeight', 'bold');
axis equal;
grid on;
box on;
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

% Add text annotation for error magnitude
text(vd_pt, vm_pt, sprintf('  Δ = %.2f mV', max_error_signed*1000), ...
     'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');

%% Add overall title
sgtitle(sprintf('Tracking Error Analysis: P%d @ %.1f Hz (%s)', ...
                analysis_channel, force_frequency, mode_labels{sample_rate_mode}), ...
        'FontSize', 18, 'FontWeight', 'bold');

fprintf('  Figure created.\n');
fprintf('\n');


%% Section 4: Error Distribution Analysis

fprintf('【Section 4: Error vs Phase Analysis】\n');
fprintf('──────────────────────────────────────\n');

% Calculate phase for each point in steady state
phase_steady = mod(t_steady, T_period) / T_period * 360;  % 0 to 360 deg

% Bin error by phase
num_bins = 36;  % 10 degree bins
bin_edges = linspace(0, 360, num_bins+1);
bin_centers = (bin_edges(1:end-1) + bin_edges(2:end)) / 2;

error_by_phase = zeros(num_bins, 1);
error_std_by_phase = zeros(num_bins, 1);

for i = 1:num_bins
    in_bin = (phase_steady >= bin_edges(i)) & (phase_steady < bin_edges(i+1));
    if sum(in_bin) > 0
        error_by_phase(i) = mean(ve_ch(in_bin)) * 1000;  % mV
        error_std_by_phase(i) = std(ve_ch(in_bin)) * 1000;
    end
end

% Find phase with maximum mean error
[~, max_phase_idx] = max(abs(error_by_phase));
max_error_phase = bin_centers(max_phase_idx);
fprintf('  Phase with max mean error: %.0f deg\n', max_error_phase);

% Create phase-error figure
fig2 = figure('Position', [150 150 1000 500], 'Name', 'Error vs Phase');

subplot(1, 2, 1);
bar(bin_centers, error_by_phase, 'FaceColor', color_error, 'EdgeColor', 'none');
hold on;
xline(90, 'r--', 'LineWidth', 2, 'Label', 'Peak (90°)');
xline(270, 'r--', 'LineWidth', 2, 'Label', 'Peak (270°)');
xline(0, 'b--', 'LineWidth', 2, 'Label', 'Zero (0°)');
xline(180, 'b--', 'LineWidth', 2, 'Label', 'Zero (180°)');
xlabel('Phase [deg]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Mean Error [mV]', 'FontSize', 14, 'FontWeight', 'bold');
title('Error vs Signal Phase', 'FontSize', 16, 'FontWeight', 'bold');
xlim([0 360]);
grid on;
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

subplot(1, 2, 2);
polarplot(deg2rad(bin_centers), abs(error_by_phase), '-o', ...
          'LineWidth', 2, 'MarkerSize', 6, 'Color', color_error);
title('Error Magnitude vs Phase (Polar)', 'FontSize', 16, 'FontWeight', 'bold');
set(gca, 'FontSize', 12);

fprintf('\n');


%% Section 5: Summary and Interpretation

fprintf('════════════════════════════════════════════════════════════\n');
fprintf('                    Analysis Summary\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');

fprintf('【Why Maximum Error Occurs at This Location】\n');
fprintf('\n');
fprintf('  1. PHASE LAG EFFECT:\n');
fprintf('     - Controller has finite bandwidth → output lags input\n');
fprintf('     - At rising/falling edges (near zero crossing), dVd/dt is maximum\n');
fprintf('     - Phase lag × high slope = large instantaneous error\n');
fprintf('\n');
fprintf('  2. 1600 Hz SAMPLING DELAY:\n');
fprintf('     - f_d sampled at 1600 Hz (Ts = 625 µs)\n');
fprintf('     - At %.1f Hz: one 1600 Hz period = %.1f deg of signal phase\n', ...
        force_frequency, delay_1600_deg);
if sample_rate_mode == 2
    fprintf('     - Linear interpolation adds ~%.1f deg average delay\n', avg_delay_deg);
end
fprintf('\n');
fprintf('  3. COMBINED EFFECT:\n');
fprintf('     - Max error occurs where: (signal slope) × (total delay) is maximum\n');
fprintf('     - For sine wave: this is near zero-crossing points\n');
fprintf('     - Observed max error phase: %.1f deg\n', phase_at_max);
fprintf('\n');

% Interpretation of observed vs expected
fprintf('【Comparison: Expected vs Observed】\n');
if abs(phase_at_max - 0) < 30 || abs(phase_at_max - 180) < 30 || abs(phase_at_max - 360) < 30
    fprintf('  ✓ Max error is near zero-crossing (expected for phase-lag dominated system)\n');
else
    fprintf('  ⚠ Max error is NOT at zero-crossing - may indicate other effects\n');
    fprintf('    (Check: controller saturation, nonlinearity, or measurement noise)\n');
end

fprintf('\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('                  Analysis Complete\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');
