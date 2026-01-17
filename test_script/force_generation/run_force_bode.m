% run_force_bode.m
% Force Control Frequency Response Test - Bode Plot Analysis
%
% This script measures the frequency response of the force control pipeline:
%   f_d (desired force) -> Inverse Model -> v_d -> R-Controller -> v_m -> Force Model -> f_m
%
% Features:
%   1. Sweep through multiple frequency points (1 Hz ~ 2 kHz, 14 points)
%   2. Test single-axis force response (X, Y, or Z)
%   3. Use FFT analysis to compute magnitude and phase
%   4. Quality checks (steady-state, DC error)
%   5. Generate Bode Plot with -3dB point annotation
%   6. Measure cross-axis crosstalk
%   7. Save results (.mat and .png)

clear; clc; close all;

fprintf('\n');
fprintf('================================================================\n');
fprintf('       Force Control Frequency Response Test (Bode Plot)\n');
fprintf('================================================================\n');
fprintf('\n');

%% ========================================================================
%                         SECTION 1: Configuration
%% ========================================================================

% Add paths
script_dir = fileparts(mfilename('fullpath'));
package_root = fullfile(script_dir, '..', '..');
addpath(fullfile(package_root, 'model'));
addpath(fullfile(package_root, 'model', 'inner_loop_ctrl'));
addpath(fullfile(package_root, 'model', 'flux_allocation'));

% -------------------------------------------------------------------------
% 1.1 Controller Selection
% -------------------------------------------------------------------------
% Select which controller to use:
%   'r_controller' - R-Controller (discrete-time, feedforward + DOB + PI)
%   'pi_controller' - PI Controller (classic proportional-integral)
controller_type = 'pi_controller';   % 'r_controller' or 'pi_controller'

% -------------------------------------------------------------------------
% 1.2 Force Direction (single axis)
% -------------------------------------------------------------------------
force_axis = 'X';               % 'X', 'Y', or 'Z'

% -------------------------------------------------------------------------
% 1.3 Force Amplitude
% -------------------------------------------------------------------------
force_amplitude = 5.0;         % [pN]

% -------------------------------------------------------------------------
% 1.4 Bead Position (Measuring Coordinate)
% -------------------------------------------------------------------------
bead_position = [0; 0; 0];    % [x; y; z] in um

% -------------------------------------------------------------------------
% 1.5 Frequency Points
% -------------------------------------------------------------------------
% IMPORTANT: Maximum test frequency must be < pos_update_rate / 2 (Nyquist)
%   - For pos_update_rate = 1600 Hz, Nyquist = 800 Hz
%   - Recommended max frequency: ~500 Hz (safe margin below Nyquist)
%   - Testing at or above Nyquist causes severe aliasing/sampling errors
%
% QUICK_TEST mode: Use fewer points for faster testing
% FULL mode: All points for complete characterization
%
QUICK_TEST = false;             % Set to false for full frequency sweep

if QUICK_TEST
    % Quick test: 6 points (10 Hz ~ 500 Hz)
    frequencies = [10, 50, 100, 200, 400, 500];  % Hz
else
    % Full sweep: 11 points (1 Hz ~ 500 Hz)
    % Note: Max freq limited by pos_update_rate (Nyquist = 800 Hz)
    frequencies = [1, 10, 20, 50, 100, ...
                   125, 200, 250];  % Hz
end

% -------------------------------------------------------------------------
% 1.6 Hardware Constraint (Position Update Rate)
% -------------------------------------------------------------------------
% Real hardware has limited position update rate (e.g., 1600 Hz from camera)
% This affects high-frequency response due to interpolation delay
%
% IMPORTANT: Test frequencies must satisfy: max(frequencies) < pos_update_rate / 2
%            Otherwise, aliasing will cause incorrect results!
%
pos_update_rate = 1600;         % Position update frequency [Hz]
interp_method = 'linear';       % Interpolation method: 'linear' or 'previous' (ZOH)
USE_REALTIME_INTERP = true;     % true = Real-time (1-period delay for linear)
                                % false = Ideal (non-causal interpolation)

% -------------------------------------------------------------------------
% 1.7 R-Controller Bandwidth
% -------------------------------------------------------------------------
fB_f = 3000;                    % Feedforward bandwidth [Hz]
fB_c = 1000;                    % Controller bandwidth [Hz]
fB_e = 5000;                    % Estimator bandwidth [Hz]

% -------------------------------------------------------------------------
% 1.8 PI Controller Parameters
% -------------------------------------------------------------------------
Kp_value = 2;                   % Proportional gain
zc = 2206;                      % Zero location [rad/s]
Ki_value = Kp_value * zc;       % Integral gain (Ki = Kp * zc = 4412)

% -------------------------------------------------------------------------
% 1.9 Simulation Cycles
% -------------------------------------------------------------------------
% NOTE: Lower frequencies require longer simulation times!
%   1 Hz with 100 cycles = 100 s sim time => ~5 hours computation
%   10 Hz with 100 cycles = 10 s sim time => ~30 min computation
%
% max_sim_time limits the simulation duration for low frequencies.
% Recommended: 2-5 s for quick test, 10-20 s for better accuracy
%
total_cycles = 100;             % Total cycles to simulate
skip_cycles = 60;               % Skip transient cycles
fft_cycles = 40;                % Use for FFT analysis
min_sim_time = 0.1;             % Minimum simulation time [s] (for high freq)
max_sim_time = 5.0;             % Maximum simulation time [s] (for low freq)
                                % WARNING: Setting this > 10 will be very slow!

% -------------------------------------------------------------------------
% 1.10 Quality Check Parameters
% -------------------------------------------------------------------------
steady_state_threshold = 0.02;  % Steady-state threshold (2% of amplitude)
dc_tolerance = 0.01;            % DC tolerance (1% of amplitude)

% -------------------------------------------------------------------------
% 1.11 Output Control
% -------------------------------------------------------------------------
SAVE_PNG = true;
SAVE_MAT = true;
output_dir = fullfile(package_root, 'test_results', 'force_generation', 'force_bode');

% -------------------------------------------------------------------------
% 1.12 Plot Style
% -------------------------------------------------------------------------
measurement_linewidth = 2.5;
reference_linewidth = 2.0;
axis_linewidth = 1.5;
xlabel_fontsize = 14;
ylabel_fontsize = 14;
title_fontsize = 15;
tick_fontsize = 12;
legend_fontsize = 11;
marker_size = 8;


%% ========================================================================
%                         SECTION 2: Initialization
%% ========================================================================

fprintf('[Configuration]\n');
fprintf('------------------------\n');

% System constants
Ts = 1e-5;                      % Sampling time [s] (100 kHz)

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

% Load system parameters (for inverse_model / force_model)
inv_params = system_params();

% Load R-Controller parameters
ctrl_params = model_base_ctrl_calc_params(fB_c, fB_e, fB_f);

% Set force direction vector based on axis selection
switch upper(force_axis)
    case 'X'
        force_direction = [1; 0; 0];
        axis_idx = 1;
        axis_labels = {'Fx', 'Fy', 'Fz'};
    case 'Y'
        force_direction = [0; 1; 0];
        axis_idx = 2;
        axis_labels = {'Fx', 'Fy', 'Fz'};
    case 'Z'
        force_direction = [0; 0; 1];
        axis_idx = 3;
        axis_labels = {'Fx', 'Fy', 'Fz'};
    otherwise
        error('Invalid force_axis: %s. Must be X, Y, or Z.', force_axis);
end

% Display configuration
fprintf('  Controller: %s\n', controller_label);
fprintf('  Force axis: %s\n', force_axis);
fprintf('  Force amplitude: %.2f pN\n', force_amplitude);
fprintf('  Bead position: [%.1f, %.1f, %.1f] um\n', bead_position);
fprintf('  Frequency range: %.1f Hz ~ %.1f Hz (%d points)\n', ...
        frequencies(1), frequencies(end), length(frequencies));
if ControllerType == 1
    fprintf('  R-Controller: fB_f=%d, fB_c=%d, fB_e=%d Hz\n', fB_f, fB_c, fB_e);
else
    fprintf('  PI-Controller: Kp=%.1f, Ki=%.1f (zc=%d)\n', Kp_value, Ki_value, zc);
end
fprintf('  Hardware constraint: %d Hz position update\n', pos_update_rate);
if USE_REALTIME_INTERP
    interp_mode_str = 'real-time';
else
    interp_mode_str = 'ideal';
end
fprintf('  Interpolation: %s (%s)\n', interp_method, interp_mode_str);
fprintf('  Simulation cycles: total=%d, skip=%d, FFT=%d\n', ...
        total_cycles, skip_cycles, fft_cycles);
fprintf('  Max simulation time: %.1f s\n', max_sim_time);
if QUICK_TEST
    fprintf('  Mode: QUICK_TEST (faster, fewer points)\n');
else
    fprintf('  Mode: FULL (all frequencies, slower)\n');
end

% Estimate total time
total_sim_time_est = 0;
for fi = 1:length(frequencies)
    period_est = 1 / frequencies(fi);
    sim_time_est = total_cycles * period_est;
    sim_time_est = max(min_sim_time, min(sim_time_est, max_sim_time));
    total_sim_time_est = total_sim_time_est + sim_time_est;
end
% Rough estimate: computation takes ~180x longer than sim_time
total_compute_est = total_sim_time_est * 180;
fprintf('\n  Estimated total time: %.1f min (%.1f hours)\n', ...
        total_compute_est/60, total_compute_est/3600);
fprintf('  (This is a rough estimate based on 100 kHz sampling)\n');
fprintf('\n');

% Initialize result matrices
num_freq = length(frequencies);
magnitude_ratio = zeros(num_freq, 1);           % Main axis |f_m/f_d|
magnitude_dB = zeros(num_freq, 1);              % Main axis [dB]
phase_lag_rad = zeros(num_freq, 1);             % Main axis phase [rad]
phase_lag_deg = zeros(num_freq, 1);             % Main axis phase [deg]
crosstalk_ratio = zeros(num_freq, 3);           % All 3 axes |f_m_i/f_d|
sim_times = zeros(num_freq, 1);

% Quality check results
quality_steady_state = true(num_freq, 1);
quality_dc_error = zeros(num_freq, 1);
quality_dc_pass = true(num_freq, 1);

% Check model exists
model_name = 'main_system';
model_path = fullfile(package_root, 'model', [model_name '.slx']);
if ~exist(model_path, 'file')
    error('Model file not found: %s', model_path);
end

% Create output directory
test_timestamp = datestr(now, 'yyyymmdd_HHMMSS');
ctrl_short = {'r_ctrl', 'pi_ctrl'};
test_folder_name = sprintf('%s_%s_axis_%s', ctrl_short{ControllerType}, force_axis, test_timestamp);
test_dir = fullfile(output_dir, test_folder_name);
if ~exist(test_dir, 'dir')
    mkdir(test_dir);
    fprintf('  Created output directory: %s\n', test_dir);
end

% Load Simulink model
if ~bdIsLoaded(model_name)
    load_system(model_path);
end
fprintf('  Model loaded: %s\n', model_name);

% Validate frequency range against hardware constraint
nyquist_freq = pos_update_rate / 2;
max_test_freq = max(frequencies);
if max_test_freq >= nyquist_freq
    warning('Max test frequency (%.0f Hz) >= Nyquist frequency (%.0f Hz)!', ...
            max_test_freq, nyquist_freq);
    fprintf('  WARNING: Results at frequencies >= %.0f Hz will be unreliable!\n', nyquist_freq);
    fprintf('           Consider reducing max frequency or increasing pos_update_rate.\n');
elseif max_test_freq > nyquist_freq * 0.7
    fprintf('  Note: Max test freq (%.0f Hz) is close to Nyquist (%.0f Hz).\n', ...
            max_test_freq, nyquist_freq);
    fprintf('        Some high-frequency results may show interpolation artifacts.\n');
end
fprintf('\n');


%% ========================================================================
%                      SECTION 3: Frequency Sweep Loop
%% ========================================================================

fprintf('================================================================\n');
fprintf('               Starting Frequency Sweep\n');
fprintf('================================================================\n');
fprintf('\n');

for freq_idx = 1:num_freq
    freq = frequencies(freq_idx);
    period = 1 / freq;

    % Calculate simulation time
    sim_time = total_cycles * period;
    sim_time = max(min_sim_time, min(sim_time, max_sim_time));
    sim_times(freq_idx) = sim_time;

    fprintf('--------------------------------------------------------\n');
    fprintf('[%2d/%2d] Frequency: %8.2f Hz (Period: %.4f s, Sim: %.2f s)\n', ...
            freq_idx, num_freq, freq, period, sim_time);
    fprintf('--------------------------------------------------------\n');

    % ---------------------------------------------------------------------
    % 3.1 Define Time Axes (Hardware Constraint Mode)
    % ---------------------------------------------------------------------
    % Two time axes: 100 kHz (controller) and pos_update_rate Hz (position)
    Ts_ctrl = Ts;                           % 100 kHz (R-Controller)
    Ts_pos  = 1 / pos_update_rate;          % 1600 Hz (Position update)

    N_ctrl = round(sim_time / Ts_ctrl) + 1;
    N_pos  = round(sim_time / Ts_pos) + 1;

    t_ctrl = (0:N_ctrl-1)' * Ts_ctrl;       % 100 kHz time axis
    t_pos  = (0:N_pos-1)' * Ts_pos;         % 1600 Hz time axis

    % Main time axis for analysis
    t = t_ctrl;
    N = N_ctrl;

    % ---------------------------------------------------------------------
    % 3.2 Generate f_d at Position Update Rate (1600 Hz)
    % ---------------------------------------------------------------------
    % f_d_low: N_pos x 3 matrix [Fx, Fy, Fz] at 1600 Hz
    f_d_low = force_amplitude * sin(2*pi*freq*t_pos) .* force_direction';

    % Interpolate f_d to 100 kHz (same method as vd interpolation)
    % This ensures f_d and f_m have the same delay/distortion from hardware constraint
    if USE_REALTIME_INTERP
        if strcmp(interp_method, 'previous')  % ZOH
            t_query_fd = t_ctrl;
        else  % Linear
            t_query_fd = t_ctrl - Ts_pos;
        end
        t_query_fd(t_query_fd < 0) = 0;
        f_d = interp1(t_pos, f_d_low, t_query_fd, interp_method, 'extrap');
    else
        f_d = interp1(t_pos, f_d_low, t_ctrl, interp_method, 'extrap');
    end

    % ---------------------------------------------------------------------
    % 3.3 Inverse Model: f_d -> v_d (at 1600 Hz for efficiency)
    % ---------------------------------------------------------------------
    fprintf('  Computing inverse_model @ %d Hz... ', pos_update_rate);
    tic;
    vd_low = zeros(N_pos, 6);
    for i = 1:N_pos
        vd_low(i, :) = inverse_model(f_d_low(i, :)', bead_position, inv_params)';
    end
    fprintf('Done (%.2f s)\n', toc);

    % ---------------------------------------------------------------------
    % 3.4 Interpolate vd to 100 kHz (with Hardware Constraint)
    % ---------------------------------------------------------------------
    % Causal Interpolation Logic:
    %   ZOH ('previous'): No extra delay - uses latest available sample
    %                     Inherent delay: ~0.5T
    %   Linear ('linear'): Needs 1-period delay - requires two samples
    %                      Inherent delay: ~1.0T
    if USE_REALTIME_INTERP
        if strcmp(interp_method, 'previous')  % ZOH
            t_query = t_ctrl;
            expected_delay_T = 0.5;
        else  % Linear
            t_query = t_ctrl - Ts_pos;
            expected_delay_T = 1.0;
        end
        t_query(t_query < 0) = 0;
        vd = interp1(t_pos, vd_low, t_query, interp_method, 'extrap');
    else
        % Ideal mode: non-causal interpolation
        vd = interp1(t_pos, vd_low, t_ctrl, interp_method, 'extrap');
        expected_delay_T = 0;
    end

    % ---------------------------------------------------------------------
    % 3.5 Simulink Simulation (Controller)
    % ---------------------------------------------------------------------
    % Prepare vd_timeseries for Simulink From Workspace block
    vd_timeseries = timeseries(vd, t);
    vd_timeseries.Name = 'vd_external';
    assignin('base', 'vd_timeseries', vd_timeseries);

    % Set SignalType = 3 (External mode)
    SignalType = 3;
    assignin('base', 'SignalType', SignalType);

    % Set other required parameters
    assignin('base', 'Channel', 1);
    assignin('base', 'Amplitude', 0);
    assignin('base', 'Frequency', freq);
    assignin('base', 'Phase', 0);
    assignin('base', 'StepTime', 0);
    assignin('base', 'd', 0);
    assignin('base', 'params', ctrl_params);

    % Set controller type and parameters
    assignin('base', 'ControllerType', ControllerType);
    assignin('base', 'Kp_value', Kp_value);
    assignin('base', 'Ki_value', Ki_value);

    % Set simulation parameters
    set_param(model_name, 'StopTime', num2str(sim_time));
    set_param(model_name, 'Solver', 'ode5');
    set_param(model_name, 'FixedStep', num2str(Ts));

    % Run simulation
    fprintf('  Running Simulink... ');
    tic;
    try
        out = sim(model_name);
        fprintf('Done (%.2f s)\n', toc);
    catch ME
        fprintf('FAILED: %s\n', ME.message);
        continue;
    end

    % Extract vm from simulation output
    vm_sim = out.Vm;
    N_vm = size(vm_sim, 1);
    t_vm = (0:N_vm-1)' * Ts;

    % Resample if needed
    if N_vm ~= N
        vm = zeros(N, 6);
        for ch = 1:6
            vm(:, ch) = interp1(t_vm, vm_sim(:, ch), t, 'linear', 'extrap');
        end
    else
        vm = vm_sim;
    end

    % ---------------------------------------------------------------------
    % 3.6 Force Model: v_m -> f_m
    % ---------------------------------------------------------------------
    fprintf('  Computing force_model... ');
    tic;
    f_m = zeros(N, 3);
    for i = 1:N
        f_m(i, :) = force_model(vm(i, :)', bead_position, inv_params)';
    end
    fprintf('Done (%.2f s)\n', toc);

    % ---------------------------------------------------------------------
    % 3.7 Steady-State Data Selection
    % ---------------------------------------------------------------------
    % Dynamically adjust skip/fft cycles based on actual simulation time
    actual_cycles = sim_time / period;

    if actual_cycles < total_cycles
        % Simulation was limited by max_sim_time, adjust proportionally
        adj_skip_cycles = floor(actual_cycles * 0.6);   % 60% for transient
        adj_fft_cycles = floor(actual_cycles * 0.4);    % 40% for FFT

        % Ensure at least 2 cycles for FFT
        if adj_fft_cycles < 2
            adj_fft_cycles = min(2, floor(actual_cycles));
            adj_skip_cycles = floor(actual_cycles) - adj_fft_cycles;
        end

        fprintf('  Note: Adjusted cycles (skip=%d, fft=%d) for limited sim_time\n', ...
                adj_skip_cycles, adj_fft_cycles);
    else
        adj_skip_cycles = skip_cycles;
        adj_fft_cycles = fft_cycles;
    end

    skip_time = adj_skip_cycles * period;
    fft_time = adj_fft_cycles * period;

    idx_steady = (t >= skip_time) & (t <= skip_time + fft_time);

    f_d_steady = f_d(idx_steady, :);
    f_m_steady = f_m(idx_steady, :);
    N_fft = sum(idx_steady);

    % Safety check
    if N_fft < 10
        fprintf('  ERROR: Not enough data points for FFT (N_fft=%d)\n', N_fft);
        continue;
    end

    % ---------------------------------------------------------------------
    % 3.8 Quality Check: Steady-State Detection
    % ---------------------------------------------------------------------
    samples_per_cycle = round(period / Ts);
    num_cycles_check = min(5, floor(N_fft / samples_per_cycle));

    if num_cycles_check >= 2
        % Check main axis for steady-state
        steady_passed = true;
        for k = 2:num_cycles_check
            idx_prev = ((k-2)*samples_per_cycle + 1) : ((k-1)*samples_per_cycle);
            idx_curr = ((k-1)*samples_per_cycle + 1) : (k*samples_per_cycle);

            if max(idx_curr) <= size(f_m_steady, 1)
                cycle_diff = max(abs(f_m_steady(idx_curr, axis_idx) - f_m_steady(idx_prev, axis_idx)));
                if cycle_diff > steady_state_threshold * force_amplitude
                    steady_passed = false;
                    break;
                end
            end
        end
        quality_steady_state(freq_idx) = steady_passed;
    end

    % ---------------------------------------------------------------------
    % 3.9 FFT Analysis
    % ---------------------------------------------------------------------
    fs = 1 / Ts;
    freq_axis = (0:N_fft-1) * fs / N_fft;

    % Find FFT bin for target frequency
    [~, freq_bin] = min(abs(freq_axis - freq));
    actual_freq = freq_axis(freq_bin);

    % Check frequency error
    freq_error_pct = abs(freq - actual_freq) / freq * 100;
    if freq_error_pct > 0.1
        fprintf('  Warning: Frequency bin error %.3f%% (target: %.2f Hz, actual: %.2f Hz)\n', ...
                freq_error_pct, freq, actual_freq);
    end

    % FFT of main axis
    Fd_fft = fft(f_d_steady(:, axis_idx));
    Fm_fft = fft(f_m_steady(:, axis_idx));

    Fd_mag = abs(Fd_fft(freq_bin)) * 2 / N_fft;
    Fd_phase = angle(Fd_fft(freq_bin));

    Fm_mag = abs(Fm_fft(freq_bin)) * 2 / N_fft;
    Fm_phase = angle(Fm_fft(freq_bin));

    % Compute magnitude ratio and phase lag
    magnitude_ratio(freq_idx) = Fm_mag / Fd_mag;
    magnitude_dB(freq_idx) = 20 * log10(magnitude_ratio(freq_idx));

    phase_diff = Fm_phase - Fd_phase;
    % Normalize to [-pi, pi]
    while phase_diff > pi
        phase_diff = phase_diff - 2*pi;
    end
    while phase_diff < -pi
        phase_diff = phase_diff + 2*pi;
    end
    phase_lag_rad(freq_idx) = phase_diff;
    phase_lag_deg(freq_idx) = phase_diff * 180 / pi;

    % ---------------------------------------------------------------------
    % 3.10 Cross-Axis Analysis (Crosstalk)
    % ---------------------------------------------------------------------
    for ax = 1:3
        Fm_ax_fft = fft(f_m_steady(:, ax));
        Fm_ax_mag = abs(Fm_ax_fft(freq_bin)) * 2 / N_fft;
        crosstalk_ratio(freq_idx, ax) = Fm_ax_mag / Fd_mag;
    end

    % ---------------------------------------------------------------------
    % 3.11 DC Error Check
    % ---------------------------------------------------------------------
    dc_f_m = mean(f_m_steady(:, axis_idx));
    quality_dc_error(freq_idx) = dc_f_m;
    quality_dc_pass(freq_idx) = abs(dc_f_m) < dc_tolerance * force_amplitude;

    % Display results for this frequency
    fprintf('  Results: |f_m/f_d| = %.4f (%.2f dB), Phase = %.2f deg\n', ...
            magnitude_ratio(freq_idx), magnitude_dB(freq_idx), phase_lag_deg(freq_idx));
    if ~quality_steady_state(freq_idx)
        fprintf('  Warning: Steady-state check FAILED\n');
    end
    if ~quality_dc_pass(freq_idx)
        fprintf('  Warning: DC error = %.4f pN\n', dc_f_m);
    end
    fprintf('\n');
end

% Unwrap phase for continuous display (avoid discontinuities at +/-180 deg)
phase_lag_rad = unwrap(phase_lag_rad);
phase_lag_deg = phase_lag_rad * 180 / pi;


%% ========================================================================
%                      SECTION 4: Analysis & Summary
%% ========================================================================

fprintf('================================================================\n');
fprintf('                         Analysis\n');
fprintf('================================================================\n');
fprintf('\n');

% -3dB bandwidth estimation
idx_below_3dB = find(magnitude_dB < -3);
if ~isempty(idx_below_3dB) && idx_below_3dB(1) > 1
    idx_after = idx_below_3dB(1);
    idx_before = idx_after - 1;

    % Linear interpolation
    f1 = frequencies(idx_before);
    f2 = frequencies(idx_after);
    m1 = magnitude_dB(idx_before);
    m2 = magnitude_dB(idx_after);

    bandwidth_3dB = f1 + (f2 - f1) * (-3 - m1) / (m2 - m1);
    fprintf('  -3dB Bandwidth: %.2f Hz\n', bandwidth_3dB);
else
    bandwidth_3dB = NaN;
    if all(magnitude_dB >= -3)
        fprintf('  -3dB Bandwidth: > %.1f Hz (all points above -3dB)\n', frequencies(end));
    else
        fprintf('  -3dB Bandwidth: Could not estimate\n');
    end
end

% DC gain (lowest frequency)
dc_gain = magnitude_ratio(1);
dc_gain_dB = magnitude_dB(1);
fprintf('  DC Gain (%.0f Hz): %.4f (%.2f dB)\n', frequencies(1), dc_gain, dc_gain_dB);

% Phase at different frequencies (only report if frequency exists)
fprintf('  Phase at %.0f Hz: %.2f deg\n', frequencies(1), phase_lag_deg(1));
idx_100 = find(frequencies == 100);
if ~isempty(idx_100)
    fprintf('  Phase at 100 Hz: %.2f deg\n', phase_lag_deg(idx_100));
end
idx_1000 = find(frequencies == 1000);
if ~isempty(idx_1000)
    fprintf('  Phase at 1000 Hz: %.2f deg\n', phase_lag_deg(idx_1000));
end

% Cross-axis isolation (at mid-frequency if 100 Hz not available)
other_axes = setdiff([1,2,3], axis_idx);
if isempty(idx_100)
    idx_100 = round(num_freq / 2);  % Use mid-frequency point
    fprintf('\n  Cross-Axis Isolation (at %.0f Hz):\n', frequencies(idx_100));
else
    fprintf('\n  Cross-Axis Isolation (at 100 Hz):\n');
end
for ax = other_axes
    isolation_dB = 20 * log10(crosstalk_ratio(idx_100, ax) / magnitude_ratio(idx_100));
    fprintf('    %s -> %s: %.2f dB\n', axis_labels{axis_idx}, axis_labels{ax}, isolation_dB);
end

fprintf('\n');


%% ========================================================================
%                      SECTION 5: Plot Bode Diagram
%% ========================================================================

fprintf('[Generate Bode Plot]\n');
fprintf('------------------------\n');

% -------------------------------------------------------------------------
% Color and marker settings (consistent with run_frequency_sweep.m)
% -------------------------------------------------------------------------
% Force axis colors: Fx=Blue, Fy=Green, Fz=Red
force_colors = [
    0.0000, 0.0000, 1.0000;  % Fx: Blue
    0.0000, 0.5000, 0.0000;  % Fy: Green
    1.0000, 0.0000, 0.0000;  % Fz: Red
];

% Markers for each axis
force_markers = {'o', 's', '^'};  % Fx: circle, Fy: square, Fz: triangle

% Unified line width and marker size (same as run_frequency_sweep.m)
unified_linewidth = 3.5;
unified_markersize = 9;

% -------------------------------------------------------------------------
% Compute PI Controller Theoretical Response (if PI mode)
% -------------------------------------------------------------------------
% PI Controller: C(s) = Kp(1 + zc/s) = Kp(s + zc)/s
% With hardware constraint delay: tau = Ts_pos (for linear interpolation)
%
% The closed-loop transfer function with delay is complex, so we compute
% the frequency response numerically.
%
if ControllerType == 2  % PI Controller
    fprintf('  Computing PI theoretical response...\n');

    % Dense frequency points for smooth theoretical curve
    freq_theory = logspace(log10(frequencies(1)), log10(frequencies(end)*1.5), 500);

    % Hardware delay (linear interpolation introduces ~1 period delay)
    if USE_REALTIME_INTERP && strcmp(interp_method, 'linear')
        tau = 1 / pos_update_rate;  % 1-period delay
    elseif USE_REALTIME_INTERP && strcmp(interp_method, 'previous')
        tau = 0.5 / pos_update_rate;  % 0.5-period delay for ZOH
    else
        tau = 0;  % No delay in ideal mode
    end

    % Compute closed-loop frequency response
    % Open-loop: L(jw) = Kp * (jw + zc) / jw * e^(-j*w*tau)
    % Closed-loop: H(jw) = L(jw) / (1 + L(jw))

    A_theory_pi = zeros(size(freq_theory));
    phi_theory_pi = zeros(size(freq_theory));

    for i = 1:length(freq_theory)
        w = 2 * pi * freq_theory(i);
        jw = 1j * w;

        % PI controller: C(jw) = Kp * (jw + zc) / jw
        C_jw = Kp_value * (jw + zc) / jw;

        % Plant with delay: P(jw) = e^(-j*w*tau)
        P_jw = exp(-1j * w * tau);

        % Open-loop: L(jw) = C(jw) * P(jw)
        L_jw = C_jw * P_jw;

        % Closed-loop: H(jw) = L(jw) / (1 + L(jw))
        H_jw = L_jw / (1 + L_jw);

        A_theory_pi(i) = abs(H_jw);
        phi_theory_pi(i) = angle(H_jw) * 180 / pi;
    end

    % Unwrap phase for continuous display
    phi_theory_pi = unwrap(phi_theory_pi * pi / 180) * 180 / pi;

    fprintf('  PI theoretical response computed (tau = %.4f s)\n', tau);
end

% -------------------------------------------------------------------------
% Create main figure (same style as run_frequency_sweep.m)
% -------------------------------------------------------------------------
fig = figure('Name', sprintf('Force Frequency Response - %s Excitation', force_axis), ...
    'Position', [100, 100, 1200, 800]);

% -------------------------------------------------------------------------
% Upper plot: Magnitude (Linear scale, 0 ~ 1.25, all 3 axes)
% -------------------------------------------------------------------------
subplot('Position', [0.1, 0.55, 0.85, 0.35]);
hold on; grid off;

% Define theory curve color (gray, same as run_frequency_sweep.m)
theory_color = [0.5, 0.5, 0.5];

% Plot theoretical curve first (bottom layer) - PI mode only
if ControllerType == 2
    plot_handle_theory_mag = semilogx(freq_theory, A_theory_pi, '-', ...
        'LineWidth', unified_linewidth, ...
        'Color', theory_color, ...
        'DisplayName', 'Theory');
end

% Plot all 3 axes responses (Fm_x/Fd, Fm_y/Fd, Fm_z/Fd)
plot_handles_mag = gobjects(3, 1);

for ax = 1:3
    mag = crosstalk_ratio(:, ax);  % This contains |Fm_ax / Fd_excited|
    plot_handles_mag(ax) = semilogx(frequencies, mag, ['--' force_markers{ax}], ...
        'LineWidth', unified_linewidth, ...
        'Color', force_colors(ax, :), ...
        'MarkerFaceColor', 'none', ...
        'MarkerEdgeColor', force_colors(ax, :), ...
        'MarkerSize', unified_markersize, ...
        'DisplayName', axis_labels{ax});
end

% Y-axis settings (same as run_frequency_sweep.m)
ylim([0, 1.25]);
yticks([0, 0.25, 0.5, 0.75, 1.0, 1.25]);

% Labels and formatting
ylabel('Magnitude', 'FontSize', 22, 'FontWeight', 'bold');
xlim([frequencies(1), frequencies(end)]);

% Axis formatting (same as run_frequency_sweep.m)
ax1 = gca;
ax1.XScale = 'log';
ax1.XTick = [1, 10, 100, 1000, 10000];
ax1.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
ax1.FontSize = 18;
ax1.FontWeight = 'bold';
ax1.LineWidth = 2.5;
ax1.Box = 'on';

% Legend at top (northoutside, matching run_frequency_sweep.m style)
if ControllerType == 2  % PI mode - include Theory curve
    lgd = legend([plot_handles_mag; plot_handle_theory_mag], ...
        {axis_labels{1}, axis_labels{2}, axis_labels{3}, 'Theory'}, ...
        'Location', 'northoutside', 'NumColumns', 4, ...
        'FontSize', 13, 'FontWeight', 'bold', 'Orientation', 'horizontal');
else  % R-Controller mode
    lgd = legend(plot_handles_mag, 'Location', 'northoutside', 'NumColumns', 3, ...
        'FontSize', 13, 'FontWeight', 'bold', 'Orientation', 'horizontal');
end
lgd.EdgeColor = [0 0 0];  % Black border
lgd.LineWidth = 2.0;      % Thick border

% -------------------------------------------------------------------------
% Lower plot: Phase (only excited axis)
% -------------------------------------------------------------------------
subplot('Position', [0.1, 0.1, 0.85, 0.35]);
hold on; grid off;

% Plot theoretical phase curve first (bottom layer) - PI mode only
if ControllerType == 2
    semilogx(freq_theory, phi_theory_pi, '-', ...
        'LineWidth', unified_linewidth, ...
        'Color', theory_color, ...
        'DisplayName', 'Theory');
end

% Plot phase of excited axis only
semilogx(frequencies, phase_lag_deg, ['--' force_markers{axis_idx}], ...
    'LineWidth', unified_linewidth, ...
    'Color', force_colors(axis_idx, :), ...
    'MarkerFaceColor', 'none', ...
    'MarkerEdgeColor', force_colors(axis_idx, :), ...
    'MarkerSize', unified_markersize, ...
    'DisplayName', sprintf('%s Phase', axis_labels{axis_idx}));

% Labels and formatting
ylabel('Phase (deg)', 'FontSize', 22, 'FontWeight', 'bold');
xlabel('Frequency (Hz)', 'FontSize', 22, 'FontWeight', 'bold');
xlim([frequencies(1), frequencies(end)]);

% Axis formatting
ax2 = gca;
ax2.XScale = 'log';
ax2.XTick = [1, 10, 100, 1000, 10000];
ax2.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
ax2.FontSize = 18;
ax2.FontWeight = 'bold';
ax2.LineWidth = 2.5;
ax2.Box = 'on';

fprintf('  Bode plot generated\n');
fprintf('\n');


%% ========================================================================
%                      SECTION 6: Results Table
%% ========================================================================

fprintf('================================================================\n');
fprintf('                      Results Table\n');
fprintf('================================================================\n');
fprintf('\n');

fprintf('  Freq [Hz]  |  |f_m/f_d|  |  [dB]   |  Phase [deg]  |  Steady  |  DC Pass\n');
fprintf('  -----------+-----------+---------+---------------+----------+----------\n');

for i = 1:num_freq
    steady_str = 'PASS';
    if ~quality_steady_state(i)
        steady_str = 'FAIL';
    end
    dc_str = 'PASS';
    if ~quality_dc_pass(i)
        dc_str = 'FAIL';
    end

    fprintf('  %9.1f  |  %8.4f  |  %6.2f  |  %11.2f  |  %6s  |  %6s\n', ...
        frequencies(i), magnitude_ratio(i), magnitude_dB(i), phase_lag_deg(i), ...
        steady_str, dc_str);
end

fprintf('\n');


%% ========================================================================
%                      SECTION 7: Save Results
%% ========================================================================

if SAVE_MAT || SAVE_PNG
    fprintf('[Save Results]\n');
    fprintf('------------------------\n');

    if SAVE_PNG
        % Save Bode plot
        bode_filename = fullfile(test_dir, 'force_bode_plot.png');
        exportgraphics(fig, bode_filename, 'Resolution', 300);
        fprintf('  Saved: %s\n', bode_filename);
    end

    if SAVE_MAT
        results = struct();

        % Configuration
        results.config.controller_type = controller_type;
        results.config.controller_label = controller_label;
        results.config.ControllerType = ControllerType;
        results.config.force_axis = force_axis;
        results.config.force_direction = force_direction;
        results.config.force_amplitude = force_amplitude;
        results.config.bead_position = bead_position;
        results.config.frequencies = frequencies;
        results.config.fB_f = fB_f;
        results.config.fB_c = fB_c;
        results.config.fB_e = fB_e;
        results.config.Kp_value = Kp_value;
        results.config.Ki_value = Ki_value;
        results.config.pos_update_rate = pos_update_rate;
        results.config.interp_method = interp_method;
        results.config.USE_REALTIME_INTERP = USE_REALTIME_INTERP;
        results.config.total_cycles = total_cycles;
        results.config.skip_cycles = skip_cycles;
        results.config.fft_cycles = fft_cycles;
        results.config.Ts = Ts;

        % Data
        results.data.magnitude_ratio = magnitude_ratio;
        results.data.magnitude_dB = magnitude_dB;
        results.data.phase_lag_rad = phase_lag_rad;
        results.data.phase_lag_deg = phase_lag_deg;
        results.data.crosstalk_ratio = crosstalk_ratio;
        results.data.sim_times = sim_times;

        % Analysis
        results.analysis.dc_gain = dc_gain;
        results.analysis.dc_gain_dB = dc_gain_dB;
        results.analysis.bandwidth_3dB = bandwidth_3dB;

        % Quality
        results.quality.steady_state = quality_steady_state;
        results.quality.dc_error = quality_dc_error;
        results.quality.dc_pass = quality_dc_pass;

        % Theoretical data (PI mode only)
        if ControllerType == 2
            results.theory.freq = freq_theory;
            results.theory.magnitude = A_theory_pi;
            results.theory.phase_deg = phi_theory_pi;
            results.theory.tau = tau;
            results.theory.Kp = Kp_value;
            results.theory.zc = zc;
        end

        % Metadata
        results.meta.timestamp = datestr(now);
        results.meta.test_dir = test_dir;

        mat_filename = fullfile(test_dir, 'force_bode_data.mat');
        save(mat_filename, 'results', '-v7.3');
        fprintf('  Saved: %s\n', mat_filename);
    end

    fprintf('  Output directory: %s\n', test_dir);
end

fprintf('\n');
fprintf('================================================================\n');
fprintf('                      Test Complete\n');
fprintf('================================================================\n');
fprintf('\n');
