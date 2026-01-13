% run_force_control_test.m
% Phase 2 Integration Test: Force Control with Inverse Model
%
% This script demonstrates the complete force control pipeline:
%   1. Set desired force signal f_d
%   2. Call inverse_model to compute vd timeseries
%   3. Execute Simulink simulation (R-Controller tracks vd)
%   4. Call force_model to compute estimated force f_m
%   5. Plot comparison graphs
%
% Note: This is a standalone demonstration. For full Simulink integration,
% the Vd_Generator block needs to be modified to accept external vd.

clear; clc; close all;

% Add paths
script_dir = fileparts(mfilename('fullpath'));
package_root = fullfile(script_dir, '..');
addpath(fullfile(package_root, 'model'));


%%                        SECTION 1: Configuration


fprintf('\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('           Force Control Test (Phase 2)\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');

% ─────────────────────────────────────────────────────────────────────────
% 1.1 Test Name
% ─────────────────────────────────────────────────────────────────────────
test_name = 'force_control_test';

% ─────────────────────────────────────────────────────────────────────────
% 1.2 Controller Selection
% ─────────────────────────────────────────────────────────────────────────
% Select which controller to use:
%   'r_controller' - R-Controller (discrete-time, feedforward + DOB + PI)
%   'pi_controller' - PI Controller (classic proportional-integral)
controller_type = 'pi_controller';   % 'r_controller' or 'pi_controller'

% ─────────────────────────────────────────────────────────────────────────
% 1.3 Desired Force Signal (f_d)
% ─────────────────────────────────────────────────────────────────────────
signal_type = 'sine';           % 'sine' or 'step'

% Force direction and magnitude
force_direction = [1; 0; 0];    % Unit direction vector [Fx; Fy; Fz]
force_amplitude = 5.0;          % Force amplitude [pN]

% Sine mode parameters
force_frequency = 200;           % Force frequency [Hz]
force_phase = 0;                % Phase [deg]

% Step mode parameters
step_time = 0.1;                % Step transition time [s]

% ─────────────────────────────────────────────────────────────────────────
% 1.4 Bead Position (Measuring Coordinate)
% ─────────────────────────────────────────────────────────────────────────
bead_position = [0; 0; 0];      % Bead position [x; y; z] in um

% ─────────────────────────────────────────────────────────────────────────
% 1.5 Simulation Time
% ─────────────────────────────────────────────────────────────────────────
% Sine mode
total_cycles = 50;              % Total simulation cycles
skip_cycles = 20;               % Skip transient cycles
display_cycles = 5;             % Display last N cycles

% Step mode
step_sim_time = 0.5;            % Step mode simulation time [s]

% ─────────────────────────────────────────────────────────────────────────
% 1.5.1 Position Update Rate (Hardware Constraint)
% ─────────────────────────────────────────────────────────────────────────
pos_update_rate = 1600;         % Position update frequency [Hz]
interp_method = 'linear';       % Interpolation method: 'linear' or 'previous' (ZOH)
USE_REALTIME_INTERP = true;     % true = Real-time (1-period delay), false = Ideal (non-causal)

% ─────────────────────────────────────────────────────────────────────────
% 1.6 R-Controller Parameters
% ─────────────────────────────────────────────────────────────────────────
fB_f = 3000;                    % Feedforward bandwidth [Hz]
fB_c = 1000;                    % Controller bandwidth [Hz]
fB_e = 5000;                    % Estimator bandwidth [Hz]

% ─────────────────────────────────────────────────────────────────────────
% 1.7 PI Controller Parameters
% ─────────────────────────────────────────────────────────────────────────
Kp_value = 2;                   % Proportional gain
zc = 2206;                      % Zero location [rad/s]
Ki_value = Kp_value * zc;       % Integral gain (Ki = Kp * zc = 4412)

% ─────────────────────────────────────────────────────────────────────────
% 1.8 Simulink Integration
% ─────────────────────────────────────────────────────────────────────────
USE_SIMULINK = true;            % true: use Simulink R-Controller
                                 % false: assume perfect tracking (vm = vd)

% ─────────────────────────────────────────────────────────────────────────
% 1.9 Output Control
% ─────────────────────────────────────────────────────────────────────────
ENABLE_PLOT = true;
SAVE_PNG = true;
SAVE_MAT = true;
output_dir = fullfile(package_root, 'test_results', 'force_control');

% ─────────────────────────────────────────────────────────────────────────
% 1.10 Plot Style (consistent with run_rcontroller_test.m)
% ─────────────────────────────────────────────────────────────────────────
measurement_linewidth = 3.0;     % Measurement line width
reference_linewidth = 2.5;       % Reference line width
axis_linewidth = 1.5;            % Axis line width
xlabel_fontsize = 14;            % X-axis label font size
ylabel_fontsize = 14;            % Y-axis label font size
title_fontsize = 15;             % Title font size
tick_fontsize = 12;              % Tick font size
legend_fontsize = 11;            % Legend font size


%%                        SECTION 2: System Initialization


fprintf('【System Initialization】\n');
fprintf('────────────────────────\n');

% Validate controller type
controller_type = lower(controller_type);
if ~ismember(controller_type, {'r_controller', 'pi_controller'})
    error('Invalid controller_type: %s. Use ''r_controller'' or ''pi_controller''.', controller_type);
end

% Set ControllerType for Simulink (1=R-Controller, 2=PI-Controller)
if strcmpi(controller_type, 'r_controller')
    ControllerType = 1;
    controller_label = 'R-Controller';
else
    ControllerType = 2;
    controller_label = 'PI-Controller';
end

% Load system parameters (for Phase 2 models)
inv_params = system_params();

% Load R-Controller parameters
ctrl_params = r_controller_calc_params(fB_c, fB_e, fB_f);

% System constants
Ts = 1e-5;                      % Sampling time [s] (100 kHz)

% Calculate simulation time
if strcmpi(signal_type, 'sine')
    sim_time = total_cycles / force_frequency;
else
    sim_time = step_sim_time;
end

% Normalize force direction
force_direction = force_direction / norm(force_direction);

fprintf('  Controller: %s\n', controller_label);
fprintf('  Signal type: %s\n', signal_type);
fprintf('  Force direction: [%.2f, %.2f, %.2f]\n', force_direction);
fprintf('  Force amplitude: %.2f pN\n', force_amplitude);
if strcmpi(signal_type, 'sine')
    fprintf('  Frequency: %.1f Hz\n', force_frequency);
end
fprintf('  Bead position: [%.1f, %.1f, %.1f] um\n', bead_position);
fprintf('  Simulation time: %.3f s\n', sim_time);
if ControllerType == 1
    fprintf('  R-Controller: fB_f=%d, fB_c=%d, fB_e=%d Hz\n', fB_f, fB_c, fB_e);
else
    fprintf('  PI-Controller: Kp=%.1f, Ki=%.1f (zc=%d)\n', Kp_value, Ki_value, zc);
end
fprintf('\n');


%%                        SECTION 3: Generate f_d and Compute vd (with Hardware Constraint)


fprintf('【Generate Vd Timeseries (Hardware Constraint Mode)】\n');
fprintf('────────────────────────\n');

% ─────────────────────────────────────────────────────────────────────────
% 3.1 Define two time axes
% ─────────────────────────────────────────────────────────────────────────
Ts_ctrl = Ts;                           % 100 kHz (R-Controller)
Ts_pos  = 1 / pos_update_rate;          % 1600 Hz (Position update)

N_ctrl = round(sim_time / Ts_ctrl) + 1;
N_pos  = round(sim_time / Ts_pos) + 1;

t_ctrl = (0:N_ctrl-1)' * Ts_ctrl;       % 100 kHz time axis
t_pos  = (0:N_pos-1)' * Ts_pos;         % 1600 Hz time axis

fprintf('  Position update rate: %d Hz (%d points)\n', pos_update_rate, N_pos);
fprintf('  Controller rate: %.0f kHz (%d points)\n', 1/Ts_ctrl/1000, N_ctrl);
fprintf('  Frequency ratio: %.1f (%.1f μs per update)\n', Ts_pos / Ts_ctrl, Ts_pos * 1e6);

% ─────────────────────────────────────────────────────────────────────────
% 3.2 Generate f_d at 1600 Hz time points
% ─────────────────────────────────────────────────────────────────────────
f_d_low = zeros(N_pos, 3);
if strcmpi(signal_type, 'sine')
    envelope_low = force_amplitude * sin(2*pi*force_frequency*t_pos + deg2rad(force_phase));
    f_d_low = envelope_low .* force_direction';
else
    idx_step_low = t_pos >= step_time;
    f_d_low(idx_step_low, :) = repmat(force_amplitude * force_direction', sum(idx_step_low), 1);
end

% ─────────────────────────────────────────────────────────────────────────
% 3.3 Compute vd at 1600 Hz (call inverse_model)
% ─────────────────────────────────────────────────────────────────────────
fprintf('  Computing inverse_model @ %d Hz...', pos_update_rate);
tic;
vd_low = zeros(N_pos, 6);
for i = 1:N_pos
    vd_low(i, :) = inverse_model(f_d_low(i, :)', bead_position, inv_params)';
end
fprintf(' Done (%.2f sec)\n', toc);

% ─────────────────────────────────────────────────────────────────────────
% 3.4 Interpolate vd to 100 kHz
% ─────────────────────────────────────────────────────────────────────────
% CORRECTED Causal Interpolation Logic:
%   ZOH ('previous'): No extra delay needed - uses latest available sample
%                     Inherent delay: ~0.5T (holds value until next update)
%   Linear ('linear'): Needs 1-period delay - requires two samples to interpolate
%                      Inherent delay: ~1.0T (interpolates from previous samples)

if USE_REALTIME_INTERP
    fprintf('  Interpolating vd to 100 kHz (REAL-TIME, %s)...', interp_method);
    tic;

    if strcmp(interp_method, 'previous')  % ZOH
        % ZOH: No extra delay - use current time directly
        % At time t, use vd[floor(t/Ts_pos)] (latest available sample)
        t_query = t_ctrl;
        expected_delay_T = 0.5;  % ZOH inherent delay
    else  % Linear
        % Linear: Need 1-period delay to have two samples available
        % At time t in [kT, (k+1)T), interpolate between vd[k-1] and vd[k]
        t_query = t_ctrl - Ts_pos;
        expected_delay_T = 1.0;  % Linear inherent delay
    end

    % Clamp negative values to 0
    t_query(t_query < 0) = 0;

    vd = interp1(t_pos, vd_low, t_query, interp_method, 'extrap');

    interp_delay_us = expected_delay_T * Ts_pos * 1e6;
    fprintf(' Done (%.2f sec, expected delay=%.1fT=%.1f us)\n', toc, expected_delay_T, interp_delay_us);
else
    % Ideal mode: 非因果插值（知道未來）
    fprintf('  Interpolating vd to 100 kHz (IDEAL, %s, non-causal)...', interp_method);
    tic;
    vd = interp1(t_pos, vd_low, t_ctrl, interp_method, 'extrap');
    fprintf(' Done (%.2f sec)\n', toc);
end

% ─────────────────────────────────────────────────────────────────────────
% 3.5 Reference signal f_d (ideal sine wave, no interpolation delay)
% ─────────────────────────────────────────────────────────────────────────
% Use ideal continuous signal as reference for delay/error analysis
if strcmpi(signal_type, 'sine')
    f_d = force_amplitude * sin(2*pi*force_frequency*t_ctrl + deg2rad(force_phase)) .* force_direction';
else
    % For step signal, use interpolated version
    f_d = interp1(t_pos, f_d_low, t_ctrl, 'previous', 'extrap');
end

% Update main time axis
t = t_ctrl;
N = N_ctrl;

% Statistics
fprintf('  vd range (1600 Hz): [%.4f, %.4f] V\n', min(vd_low(:)), max(vd_low(:)));
fprintf('  vd range (100 kHz): [%.4f, %.4f] V\n', min(vd(:)), max(vd(:)));
if USE_REALTIME_INTERP
    fprintf('  Mode: REAL-TIME (causal, %s, ~%.1fT delay)\n', interp_method, expected_delay_T);
else
    fprintf('  Mode: IDEAL (non-causal)\n');
end
fprintf('\n');


%%                        SECTION 4: Simulink Simulation or Ideal Tracking


if USE_SIMULINK
    fprintf('【Simulink Simulation】\n');
    fprintf('────────────────────────\n');

    % Prepare vd_timeseries for Simulink From Workspace block
    vd_timeseries = timeseries(vd, t);
    vd_timeseries.Name = 'vd_external';
    assignin('base', 'vd_timeseries', vd_timeseries);

    % Set SignalType = 3 (External mode)
    SignalType = 3;
    assignin('base', 'SignalType', SignalType);

    % Set other required parameters (not used in mode 3, but needed)
    assignin('base', 'Channel', 1);
    assignin('base', 'Amplitude', 0);
    assignin('base', 'Frequency', force_frequency);
    assignin('base', 'Phase', 0);
    assignin('base', 'StepTime', 0);
    assignin('base', 'd', 0);  % Preview samples
    assignin('base', 'params', ctrl_params);

    % Set controller type and parameters
    assignin('base', 'ControllerType', ControllerType);
    assignin('base', 'Kp_value', Kp_value);
    assignin('base', 'Ki_value', Ki_value);

    % Load Simulink model
    model_name = 'r_controller_system_integrated';
    model_path = fullfile(package_root, 'model', [model_name '.slx']);

    if ~bdIsLoaded(model_name)
        load_system(model_path);
    end

    % Set simulation parameters
    set_param(model_name, 'StopTime', num2str(sim_time));

    % Run simulation
    fprintf('  Running Simulink... ');
    tic;
    out = sim(model_name);
    fprintf('Done (%.2f sec)\n', toc);

    % Extract vm from simulation output
    vm_sim = out.Vm;

    % vm is logged at fixed Ts rate, create proper time vector
    N_vm = size(vm_sim, 1);
    t_vm = (0:N_vm-1)' * Ts;

    % Resample to match our time axis if needed
    if N_vm ~= N
        fprintf('  Resampling vm (%d -> %d points)...\n', N_vm, N);
        vm = zeros(N, 6);
        for ch = 1:6
            vm(:, ch) = interp1(t_vm, vm_sim(:, ch), t, 'linear', 'extrap');
        end
    else
        vm = vm_sim;
    end

    fprintf('  vm range: [%.4f, %.4f] V\n', min(vm(:)), max(vm(:)));
else
    fprintf('【Ideal Tracking Mode】\n');
    fprintf('────────────────────────\n');
    fprintf('  Assuming perfect tracking: vm = vd\n');
    vm = vd;
end

fprintf('\n');


%%                        SECTION 5: Compute f_m using Force Model


fprintf('【Compute Force Model】\n');
fprintf('────────────────────────\n');

fprintf('  Computing force_model...');
tic;
f_m = zeros(N, 3);
for i = 1:N
    f_m(i, :) = force_model(vm(i, :)', bead_position, inv_params)';
end
fprintf(' Done (%.2f sec)\n', toc);


%%                        SECTION 6: Analysis


fprintf('\n【Analysis】\n');
fprintf('────────────────────────\n');

% Compute analysis window
if strcmpi(signal_type, 'sine')
    T_period = 1 / force_frequency;
    idx_start = round(skip_cycles * T_period / Ts) + 1;
    idx_display = round((total_cycles - display_cycles) * T_period / Ts) + 1;
else
    idx_start = round(step_time / Ts) + 1;
    idx_display = 1;
end

% Force error
force_error = f_d - f_m;
force_error_rms = rms(force_error(idx_start:end, :));
force_error_max = max(abs(force_error(idx_start:end, :)));

fprintf('  Force Error (RMS): [%.4f, %.4f, %.4f] pN\n', force_error_rms);
fprintf('  Force Error (Max): [%.4f, %.4f, %.4f] pN\n', force_error_max);

% Relative error
if force_amplitude > 0
    relative_error = norm(force_error_rms) / force_amplitude * 100;
    fprintf('  Relative Error: %.2f%%\n', relative_error);
end

fprintf('\n');


%%                        SECTION 7: Plotting (Tab Interface)


if ENABLE_PLOT
    fprintf('【Generate Plots (Tab Interface)】\n');
    fprintf('────────────────────────\n');

    % Color definitions
    colors = [
        0.0000, 0.4470, 0.7410;  % Blue
        0.8500, 0.3250, 0.0980;  % Orange
        0.9290, 0.6940, 0.1250;  % Yellow
        0.4940, 0.1840, 0.5560;  % Purple
        0.4660, 0.6740, 0.1880;  % Green
        0.3010, 0.7450, 0.9330   % Cyan
    ];
    labels = {'Fx', 'Fy', 'Fz'};
    error_colors = [0.0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880];

    % Create main figure with Tab interface
    fig_main = uifigure('Name', sprintf('Force Control Test [%s]: %s', controller_label, test_name), ...
        'Position', [100 100 1400 900]);
    tabgroup = uitabgroup(fig_main);
    tabgroup.Units = 'normalized';
    tabgroup.Position = [0 0 1 1];

    % Store tab handles for export
    tab_handles = struct();

    % Calculate per-direction relative error (%)
    per_dir_error_pct = force_error_rms / force_amplitude * 100;

    % ═══════════════════════════════════════════════════════════════════════
    % Tab 1: Force Comparison (f_d vs f_m) - 3 axes
    % ═══════════════════════════════════════════════════════════════════════
    tab1 = uitab(tabgroup, 'Title', 'Force Comparison');
    tab_handles.force_comparison = tab1;

    tl1 = tiledlayout(tab1, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl1, 'Force Comparison: Desired (f_d) vs Estimated (f_m)', ...
        'FontWeight', 'bold', 'FontSize', title_fontsize);

    for ax_idx = 1:3
        ax = nexttile(tl1);
        plot(ax, t(idx_display:end)*1000, f_d(idx_display:end, ax_idx), 'b-', ...
            'LineWidth', measurement_linewidth);
        hold(ax, 'on');
        plot(ax, t(idx_display:end)*1000, f_m(idx_display:end, ax_idx), 'r--', ...
            'LineWidth', reference_linewidth);
        xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
        ylabel(ax, sprintf('%s [pN]', labels{ax_idx}), 'FontSize', ylabel_fontsize);
        legend(ax, 'f_d (desired)', 'f_m (estimated)', 'Location', 'northeast', ...
            'FontSize', legend_fontsize);
        title(ax, sprintf('%s Direction (Error RMS: %.4f pN, %.2f%%)', ...
            labels{ax_idx}, force_error_rms(ax_idx), per_dir_error_pct(ax_idx)), ...
            'FontSize', title_fontsize, 'FontWeight', 'bold');
        ax.FontSize = tick_fontsize;
        ax.LineWidth = axis_linewidth;
        grid(ax, 'on');
        box(ax, 'on');
    end
    fprintf('  Tab 1: Force Comparison (3-axis)\n');

    % ═══════════════════════════════════════════════════════════════════════
    % Tab 2: Inverse Model Output (vd - 6 channels)
    % ═══════════════════════════════════════════════════════════════════════
    tab2 = uitab(tabgroup, 'Title', 'Inverse Model (vd)');
    tab_handles.inverse_model = tab2;

    tl2 = tiledlayout(tab2, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl2, 'Inverse Model Output: Desired Hall Sensor Voltage (v_d)', ...
        'FontWeight', 'bold', 'FontSize', title_fontsize);

    for ch = 1:6
        ax = nexttile(tl2);
        plot(ax, t(idx_display:end)*1000, vd(idx_display:end, ch), ...
            'Color', colors(ch,:), 'LineWidth', measurement_linewidth);
        xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
        ylabel(ax, 'v_d [V]', 'FontSize', ylabel_fontsize);
        title(ax, sprintf('P%d (Range: %.4f ~ %.4f V)', ch, ...
            min(vd(idx_display:end, ch)), max(vd(idx_display:end, ch))), ...
            'FontSize', title_fontsize-2, 'FontWeight', 'bold');
        ax.FontSize = tick_fontsize;
        ax.LineWidth = axis_linewidth;
        grid(ax, 'on');
        box(ax, 'on');
    end
    fprintf('  Tab 2: Inverse Model Output (vd)\n');

    % ═══════════════════════════════════════════════════════════════════════
    % Tab 3: Error Analysis - 3 axes complete
    % ═══════════════════════════════════════════════════════════════════════
    tab3 = uitab(tabgroup, 'Title', 'Error Analysis');
    tab_handles.error_analysis = tab3;

    tl3 = tiledlayout(tab3, 3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl3, sprintf('Force Estimation Error Analysis (Total RMS: %.2f%%)', relative_error), ...
        'FontWeight', 'bold', 'FontSize', title_fontsize);

    % Left column: Error time series for each axis
    for ax_idx = 1:3
        ax = nexttile(tl3, (ax_idx-1)*2 + 1);
        plot(ax, t(idx_start:end)*1000, force_error(idx_start:end, ax_idx), ...
            'Color', error_colors(ax_idx,:), 'LineWidth', measurement_linewidth);
        hold(ax, 'on');
        yline(ax, 0, 'k--', 'LineWidth', 0.5);
        yline(ax, force_error_rms(ax_idx), 'r--', 'LineWidth', reference_linewidth*0.5);
        yline(ax, -force_error_rms(ax_idx), 'r--', 'LineWidth', reference_linewidth*0.5);
        xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
        ylabel(ax, sprintf('%s Error [pN]', labels{ax_idx}), 'FontSize', ylabel_fontsize);
        title(ax, sprintf('%s: RMS=%.4f pN (%.2f%%), Max=%.4f pN', labels{ax_idx}, ...
            force_error_rms(ax_idx), per_dir_error_pct(ax_idx), force_error_max(ax_idx)), ...
            'FontSize', title_fontsize-2, 'FontWeight', 'bold');
        ax.FontSize = tick_fontsize;
        ax.LineWidth = axis_linewidth;
        grid(ax, 'on');
        box(ax, 'on');
    end

    % Right column: Error spectrum for each axis (sine mode) or statistics (step mode)
    for ax_idx = 1:3
        ax = nexttile(tl3, (ax_idx-1)*2 + 2);
        if strcmpi(signal_type, 'sine')
            error_fft = abs(fft(force_error(idx_start:end, ax_idx)));
            freq_axis = (0:length(error_fft)-1) / (length(error_fft) * Ts);
            n_half = floor(length(error_fft)/2);
            plot(ax, freq_axis(1:n_half), error_fft(1:n_half), ...
                'Color', error_colors(ax_idx,:), 'LineWidth', measurement_linewidth);
            xlim(ax, [0, 5*force_frequency]);
            xlabel(ax, 'Frequency [Hz]', 'FontSize', xlabel_fontsize);
            ylabel(ax, 'Magnitude', 'FontSize', ylabel_fontsize);
            title(ax, sprintf('%s Error Spectrum', labels{ax_idx}), ...
                'FontSize', title_fontsize-2, 'FontWeight', 'bold');
        else
            % Step mode: show error histogram
            histogram(ax, force_error(idx_start:end, ax_idx), 50, ...
                'FaceColor', error_colors(ax_idx,:), 'EdgeColor', 'none', 'FaceAlpha', 0.7);
            hold(ax, 'on');
            xline(ax, 0, 'k--', 'LineWidth', 1);
            xline(ax, mean(force_error(idx_start:end, ax_idx)), 'r-', 'LineWidth', reference_linewidth);
            xlabel(ax, sprintf('%s Error [pN]', labels{ax_idx}), 'FontSize', xlabel_fontsize);
            ylabel(ax, 'Count', 'FontSize', ylabel_fontsize);
            title(ax, sprintf('%s Error Distribution', labels{ax_idx}), ...
                'FontSize', title_fontsize-2, 'FontWeight', 'bold');
        end
        ax.FontSize = tick_fontsize;
        ax.LineWidth = axis_linewidth;
        grid(ax, 'on');
        box(ax, 'on');
    end
    fprintf('  Tab 3: Error Analysis (3-axis)\n');

    % ═══════════════════════════════════════════════════════════════════════
    % Tab 4-6: Simulink Controller Tabs (only when USE_SIMULINK = true)
    % ═══════════════════════════════════════════════════════════════════════
    if USE_SIMULINK
        % Calculate voltage tracking error
        voltage_error = vd - vm;
        voltage_error_rms = rms(voltage_error(idx_start:end, :));
        voltage_error_max = max(abs(voltage_error(idx_start:end, :)));

        % ───────────────────────────────────────────────────────────────────
        % Tab 4: Voltage Tracking (v_d vs v_m) - 6 channels
        % ───────────────────────────────────────────────────────────────────
        tab4 = uitab(tabgroup, 'Title', 'Voltage Tracking');
        tab_handles.voltage_tracking = tab4;

        tl4 = tiledlayout(tab4, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl4, 'Voltage Tracking: v_d (desired) vs v_m (measured)', ...
            'FontWeight', 'bold', 'FontSize', title_fontsize);

        for ch = 1:6
            ax = nexttile(tl4);
            plot(ax, t(idx_display:end)*1000, vd(idx_display:end, ch), 'b-', ...
                'LineWidth', measurement_linewidth);
            hold(ax, 'on');
            plot(ax, t(idx_display:end)*1000, vm(idx_display:end, ch), 'r--', ...
                'LineWidth', reference_linewidth);
            xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
            ylabel(ax, 'Voltage [V]', 'FontSize', ylabel_fontsize);
            title(ax, sprintf('P%d (Error RMS: %.4f V)', ch, voltage_error_rms(ch)), ...
                'FontSize', title_fontsize-2, 'FontWeight', 'bold');
            if ch == 1
                legend(ax, 'v_d', 'v_m', 'Location', 'best', 'FontSize', legend_fontsize-2);
            end
            ax.FontSize = tick_fontsize;
            ax.LineWidth = axis_linewidth;
            grid(ax, 'on');
            box(ax, 'on');
        end
        fprintf('  Tab 4: Voltage Tracking (v_d vs v_m)\n');

        % ───────────────────────────────────────────────────────────────────
        % Tab 5: Control Input (u) - 6 channels
        % ───────────────────────────────────────────────────────────────────
        tab5 = uitab(tabgroup, 'Title', 'Control Input (u)');
        tab_handles.control_input = tab5;

        % Extract u from simulation output
        u_data = out.u;

        tl5 = tiledlayout(tab5, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        if ControllerType == 1
            ctrl_param_str = sprintf('R-Controller (fB: f=%d, c=%d, e=%d Hz)', fB_f, fB_c, fB_e);
        else
            ctrl_param_str = sprintf('PI-Controller (Kp=%.1f, Ki=%.1f)', Kp_value, Ki_value);
        end
        title(tl5, sprintf('Control Input u - %s', ctrl_param_str), ...
            'FontWeight', 'bold', 'FontSize', title_fontsize);

        for ch = 1:6
            ax = nexttile(tl5);
            plot(ax, t(idx_display:end)*1000, u_data(idx_display:end, ch), ...
                'Color', colors(ch,:), 'LineWidth', measurement_linewidth);
            xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
            ylabel(ax, 'u [V]', 'FontSize', ylabel_fontsize);
            title(ax, sprintf('P%d (Range: %.3f ~ %.3f)', ch, ...
                min(u_data(idx_display:end, ch)), max(u_data(idx_display:end, ch))), ...
                'FontSize', title_fontsize-2, 'FontWeight', 'bold');
            ax.FontSize = tick_fontsize;
            ax.LineWidth = axis_linewidth;
            grid(ax, 'on');
            box(ax, 'on');
        end
        fprintf('  Tab 5: Control Input (u)\n');

        % ───────────────────────────────────────────────────────────────────
        % Tab 6: Disturbance Estimation (u_w1) - 6 channels
        % ───────────────────────────────────────────────────────────────────
        tab6 = uitab(tabgroup, 'Title', 'Disturbance (u_w1)');
        tab_handles.disturbance = tab6;

        % Extract u_w1 from simulation output
        u_w1_data = out.u_w1;

        tl6 = tiledlayout(tab6, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl6, 'Disturbance Estimation (u_w1)', ...
            'FontWeight', 'bold', 'FontSize', title_fontsize);

        for ch = 1:6
            ax = nexttile(tl6);
            plot(ax, t(idx_display:end)*1000, u_w1_data(idx_display:end, ch), ...
                'Color', colors(ch,:), 'LineWidth', measurement_linewidth);
            xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
            ylabel(ax, 'u_{w1} [V]', 'FontSize', ylabel_fontsize);
            title(ax, sprintf('P%d', ch), 'FontSize', title_fontsize-2, 'FontWeight', 'bold');
            ax.FontSize = tick_fontsize;
            ax.LineWidth = axis_linewidth;
            grid(ax, 'on');
            box(ax, 'on');
        end
        fprintf('  Tab 6: Disturbance Estimation (u_w1)\n');

        % ───────────────────────────────────────────────────────────────────
        % Tab 7: Interpolation Detail (vd @ 1600 Hz vs vd interpolated vs vm)
        % ───────────────────────────────────────────────────────────────────
        tab7 = uitab(tabgroup, 'Title', 'Interpolation Detail');
        tab_handles.interp_detail = tab7;

        % Calculate detail window: show 2-3 periods of 1600 Hz updates
        detail_duration = 3 / pos_update_rate;  % 3 position update periods
        if strcmpi(signal_type, 'sine')
            % For sine: show around peak region in steady state
            T_period = 1 / force_frequency;
            t_detail_start = skip_cycles * T_period + T_period * 0.2;  % Start at 20% of a cycle
        else
            % For step: show around step transition
            t_detail_start = step_time - detail_duration * 0.5;
            t_detail_start = max(0, t_detail_start);
        end
        t_detail_end = t_detail_start + detail_duration;

        % Find indices for detail view
        idx_detail_ctrl = (t_ctrl >= t_detail_start) & (t_ctrl <= t_detail_end);
        idx_detail_pos = (t_pos >= t_detail_start) & (t_pos <= t_detail_end);

        t_detail_ctrl = t_ctrl(idx_detail_ctrl);
        t_detail_pos = t_pos(idx_detail_pos);
        vd_detail_interp = vd(idx_detail_ctrl, :);
        vd_detail_1600 = vd_low(idx_detail_pos, :);
        vm_detail = vm(idx_detail_ctrl, :);

        % Compute ideal interpolation for comparison (non-causal)
        vd_ideal_detail = interp1(t_pos, vd_low, t_detail_ctrl, interp_method, 'extrap');

        tl7 = tiledlayout(tab7, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        if USE_REALTIME_INTERP
            interp_mode_str = sprintf('REAL-TIME %s (1-period delay)', interp_method);
        else
            interp_mode_str = sprintf('IDEAL %s (non-causal)', interp_method);
        end
        title(tl7, sprintf('Interpolation Detail [%s]: vd@%dHz (dots), ideal (green), actual (blue), vm (red)', ...
            interp_mode_str, pos_update_rate), 'FontWeight', 'bold', 'FontSize', title_fontsize);

        for ch = 1:6
            ax = nexttile(tl7);

            % Plot vd @ 1600 Hz sample points - black dots (reference)
            plot(ax, t_detail_pos*1000, vd_detail_1600(:, ch), 'ko', ...
                'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', sprintf('vd @ %d Hz', pos_update_rate));
            hold(ax, 'on');

            % Plot ideal interpolation (non-causal) - green dashed line
            plot(ax, t_detail_ctrl*1000, vd_ideal_detail(:, ch), 'g--', ...
                'LineWidth', reference_linewidth, 'DisplayName', 'vd ideal (non-causal)');

            % Plot actual interpolated vd (100 kHz) - blue solid line
            plot(ax, t_detail_ctrl*1000, vd_detail_interp(:, ch), 'b-', ...
                'LineWidth', measurement_linewidth, 'DisplayName', 'vd actual (used)');

            % Plot vm (measured) - red dotted line
            plot(ax, t_detail_ctrl*1000, vm_detail(:, ch), 'r:', ...
                'LineWidth', reference_linewidth, 'DisplayName', 'vm (measured)');

            % Add vertical lines at 1600 Hz sample points
            for k = 1:length(t_detail_pos)
                xline(ax, t_detail_pos(k)*1000, ':', 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
            end

            xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
            ylabel(ax, 'Voltage [V]', 'FontSize', ylabel_fontsize);
            title(ax, sprintf('P%d', ch), 'FontSize', title_fontsize-2, 'FontWeight', 'bold');
            ax.FontSize = tick_fontsize;
            ax.LineWidth = axis_linewidth;
            grid(ax, 'on');
            box(ax, 'on');

            if ch == 1
                legend(ax, 'Location', 'best', 'FontSize', legend_fontsize-2);
            end
        end
        fprintf('  Tab 7: Interpolation Detail (%.2f ms window)\n', detail_duration*1000);

        % ───────────────────────────────────────────────────────────────────
        % Tab 8: Tracking Error due to Interpolation
        % ───────────────────────────────────────────────────────────────────
        tab8 = uitab(tabgroup, 'Title', 'Tracking Error');
        tab_handles.tracking_error = tab8;

        % Compute tracking error
        tracking_error = vd - vm;
        tracking_error_rms = rms(tracking_error(idx_start:end, :));

        tl8 = tiledlayout(tab8, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl8, sprintf('Voltage Tracking Error (vd - vm) [%s] @ %d Hz', interp_mode_str, pos_update_rate), ...
            'FontWeight', 'bold', 'FontSize', title_fontsize);

        for ch = 1:6
            ax = nexttile(tl8);
            plot(ax, t(idx_display:end)*1000, tracking_error(idx_display:end, ch)*1000, ...
                'Color', colors(ch,:), 'LineWidth', measurement_linewidth);
            hold(ax, 'on');
            yline(ax, 0, 'k--', 'LineWidth', 0.5);
            xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
            ylabel(ax, 'Error [mV]', 'FontSize', ylabel_fontsize);
            title(ax, sprintf('P%d (RMS: %.3f mV)', ch, tracking_error_rms(ch)*1000), ...
                'FontSize', title_fontsize-2, 'FontWeight', 'bold');
            ax.FontSize = tick_fontsize;
            ax.LineWidth = axis_linewidth;
            grid(ax, 'on');
            box(ax, 'on');
        end
        fprintf('  Tab 8: Tracking Error Analysis\n');
    end

    % Store main figure handle for export
    fig_handles = struct('main', fig_main, 'tabs', tab_handles);

    fprintf('\n');
end


%%                        SECTION 8: Summary


fprintf('════════════════════════════════════════════════════════════\n');
fprintf('                      Test Summary\n');
fprintf('════════════════════════════════════════════════════════════\n');

fprintf('\n【Force Estimation Error】\n');
fprintf('  RMS: [%.4f, %.4f, %.4f] pN\n', force_error_rms);
fprintf('  Max: [%.4f, %.4f, %.4f] pN\n', force_error_max);
if exist('relative_error', 'var')
    fprintf('  Relative Error: %.2f%%\n', relative_error);
end

fprintf('\n【Mode】\n');
if USE_SIMULINK
    fprintf('  Simulink %s integration enabled.\n', controller_label);
    fprintf('  vm is obtained from actual %s tracking.\n', controller_label);
else
    fprintf('  Ideal tracking mode (vm = vd).\n');
    fprintf('  For realistic results, set USE_SIMULINK = true.\n');
end


%%                        SECTION 9: Save Results


if SAVE_MAT || SAVE_PNG
    fprintf('\n【Save Results】\n');
    fprintf('────────────────────────\n');

    % Create output directory
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    test_dir = fullfile(output_dir, sprintf('%s_%s', test_name, timestamp));
    if ~exist(test_dir, 'dir')
        mkdir(test_dir);
    end

    if SAVE_PNG && ENABLE_PLOT
        export_resolution = 300;

        % Export each tab individually
        try
            if isfield(fig_handles, 'main') && isvalid(fig_handles.main)
                % Tab 1: Force Comparison
                tabgroup.SelectedTab = tab1;
                drawnow; pause(0.1);
                exportgraphics(tab1, fullfile(test_dir, 'tab1_force_comparison.png'), 'Resolution', export_resolution);

                % Tab 2: Inverse Model
                tabgroup.SelectedTab = tab2;
                drawnow; pause(0.1);
                exportgraphics(tab2, fullfile(test_dir, 'tab2_inverse_model.png'), 'Resolution', export_resolution);

                % Tab 3: Error Analysis
                tabgroup.SelectedTab = tab3;
                drawnow; pause(0.1);
                exportgraphics(tab3, fullfile(test_dir, 'tab3_error_analysis.png'), 'Resolution', export_resolution);

                tab_count = 3;

                % Additional tabs for Simulink mode
                if USE_SIMULINK
                    % Tab 4: Voltage Tracking
                    tabgroup.SelectedTab = tab4;
                    drawnow; pause(0.1);
                    exportgraphics(tab4, fullfile(test_dir, 'tab4_voltage_tracking.png'), 'Resolution', export_resolution);

                    % Tab 5: Control Input
                    tabgroup.SelectedTab = tab5;
                    drawnow; pause(0.1);
                    exportgraphics(tab5, fullfile(test_dir, 'tab5_control_input.png'), 'Resolution', export_resolution);

                    % Tab 6: Disturbance
                    tabgroup.SelectedTab = tab6;
                    drawnow; pause(0.1);
                    exportgraphics(tab6, fullfile(test_dir, 'tab6_disturbance.png'), 'Resolution', export_resolution);

                    % Tab 7: Interpolation Detail
                    tabgroup.SelectedTab = tab7;
                    drawnow; pause(0.1);
                    exportgraphics(tab7, fullfile(test_dir, 'tab7_interp_detail.png'), 'Resolution', export_resolution);

                    % Tab 8: Tracking Error
                    tabgroup.SelectedTab = tab8;
                    drawnow; pause(0.1);
                    exportgraphics(tab8, fullfile(test_dir, 'tab8_tracking_error.png'), 'Resolution', export_resolution);

                    tab_count = 8;
                end

                fprintf('  Figures saved (%d tabs, %d DPI)\n', tab_count, export_resolution);
            end
        catch ME
            fprintf('  Warning: Could not export figures: %s\n', ME.message);
        end
    end

    if SAVE_MAT
        results = struct();
        results.config.test_name = test_name;
        results.config.controller_type = controller_type;
        results.config.controller_label = controller_label;
        results.config.signal_type = signal_type;
        results.config.force_direction = force_direction;
        results.config.force_amplitude = force_amplitude;
        results.config.force_frequency = force_frequency;
        results.config.bead_position = bead_position;
        results.config.sim_time = sim_time;
        results.config.fB_f = fB_f;
        results.config.fB_c = fB_c;
        results.config.fB_e = fB_e;
        results.config.Kp_value = Kp_value;
        results.config.Ki_value = Ki_value;
        results.config.USE_SIMULINK = USE_SIMULINK;
        results.config.pos_update_rate = pos_update_rate;
        results.config.interp_method = interp_method;
        results.config.USE_REALTIME_INTERP = USE_REALTIME_INTERP;
        results.config.interp_delay_us = Ts_pos * 1e6;  % Delay in microseconds

        results.data.t = t;
        results.data.f_d = f_d;
        results.data.vd = vd;
        results.data.vm = vm;
        results.data.f_m = f_m;
        results.data.force_error = force_error;

        % Low-rate (1600 Hz) data for interpolation analysis
        results.data.t_pos = t_pos;
        results.data.f_d_low = f_d_low;
        results.data.vd_low = vd_low;

        results.analysis.force_error_rms = force_error_rms;
        results.analysis.force_error_max = force_error_max;
        if exist('relative_error', 'var')
            results.analysis.relative_error = relative_error;
        end

        % Additional Simulink data
        if USE_SIMULINK
            results.data.u = u_data;
            results.data.u_w1 = u_w1_data;
            results.analysis.voltage_error_rms = voltage_error_rms;
            results.analysis.voltage_error_max = voltage_error_max;
            results.analysis.tracking_error_rms = tracking_error_rms;
        end

        results.params = inv_params;
        results.meta.timestamp = datestr(now);

        save(fullfile(test_dir, 'results.mat'), 'results', '-v7.3');
        fprintf('  Data saved (.mat)\n');
    end

    fprintf('  Output directory: %s\n', test_dir);
end

fprintf('\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('                    Test Complete\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');


%% ════════════════════════════════════════════════════════════════════════
%                    Helper Function
%% ════════════════════════════════════════════════════════════════════════
function status = get_result_status(relative_error)
    if relative_error < 1
        status = 'EXCELLENT (< 1%)';
    elseif relative_error < 5
        status = 'PASS (< 5%)';
    elseif relative_error < 10
        status = 'MARGINAL (< 10%)';
    else
        status = 'FAIL (>= 10%)';
    end
end
