% run_force_bode.m
% Force Control Frequency Response Test - Bode Plot Analysis
%
% This script measures the frequency response of the force control pipeline:
%   f_d (desired force) -> Inverse Model -> v_d -> Model Based Control -> v_m -> Force Model -> f_m
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
addpath(fullfile(package_root, 'model', 'motion_ctrl'));
addpath(fullfile(package_root, 'model', 'particle_dynamics'));
addpath(fullfile(package_root, 'test_script', 'utils'));

% -------------------------------------------------------------------------
% 1.1 Controller Selection
% -------------------------------------------------------------------------
% Select which controller to use:
%   'model_base_ctrl' - Model Based Control (discrete-time, feedforward + DOB + PI)
%   'pi_controller' - PI Controller (classic proportional-integral)
controller_type = 'model_base_ctrl';   % 'model_base_ctrl' or 'pi_controller'

% -------------------------------------------------------------------------
% 1.1.1 Model Based Control Feedforward Settings (only when controller_type = 'model_base_ctrl')
% -------------------------------------------------------------------------
% ff_enable: Enable/disable feedforward filter
%   true  - Feedforward enabled (ZPETC tracking)
%   false - Feedforward disabled (closed-loop only)
%
% ff_preview: Preview steps for ZPETC (only effective when ff_enable = true)
%   0 - No preview (ZPETC d=0, phase lag = -2*theta)
%   2 - 2-step preview (ZPETC d=2, zero phase error)
%
ff_enable = true;                        % Enable feedforward filter
ff_preview = 0;                         % Preview steps: 0 or 2

% -------------------------------------------------------------------------
% 1.1.1a LPF Configuration
% -------------------------------------------------------------------------
% lpf_enable: Enable LPF mode (3rd-order controller)
%   false = Use original 2nd-order controller, Simulink LPF bypassed
%   true  = Use new 3rd-order controller, Simulink LPF enabled
%
% f_low: LPF cutoff frequency [Hz] (only used when lpf_enable=true)
%   Typical values: 5000, 10000, 20000 Hz
%
lpf_enable = false;                     % Default: disabled for backward compatibility
f_low = 10000;                          % Default: 10 kHz

% -------------------------------------------------------------------------
% 1.1.2 Theory Curves Configuration (for Bode plot)
% -------------------------------------------------------------------------
% Select which theory curves to display
% Options:
%   'zpetc_d0' - ZPETC with preview=0 (phase = φ_rate - 2θ_inner)
%   'zpetc_d2' - ZPETC with preview=2 (phase = φ_rate only)
%   'no_ff'    - No feedforward (closed-loop inner + rate transition)
%   'pi'       - PI controller closed-loop
THEORY_CURVES = {};  % Auto-populated based on controller settings
SHOW_COMPARISON_CURVES = false;  % Only show theory curve for current controller mode

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
QUICK_TEST = false;              % Set to false for full frequency sweep

if QUICK_TEST
    % Quick test: 6 points (10 Hz ~ 500 Hz)
    frequencies = [1, 10, 50, 100, 200, 400, 500, 625, 800, 1000, 2000, 3200, 4000];  % Hz
else
    % Full sweep: 11 points (1 Hz ~ 500 Hz)
    % Note: Max freq limited by pos_update_rate (Nyquist = 800 Hz)
    frequencies = [1, 10, 20, 50, 100, ...
                   125, 200, 250];  % Hz
end

% -------------------------------------------------------------------------
% 1.6 Signal Generation Mode
% -------------------------------------------------------------------------
% Select signal generation mode:
%   'hardware' - Simulate hardware constraint (1600 Hz generation + interpolation)
%   'ideal'    - Direct 100 kHz generation (no interpolation delay)
generation_mode = 'hardware';      % 'hardware' or 'ideal'

% -------------------------------------------------------------------------
% 1.7 Hardware Constraint (Position Update Rate)
% -------------------------------------------------------------------------
% Only used when generation_mode = 'hardware'
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
% 1.8 Model Based Control Bandwidth
% -------------------------------------------------------------------------
fB_f = 3000;                    % Feedforward bandwidth [Hz]
fB_c = 3200;                    % Controller bandwidth [Hz]
fB_e = 16000;                    % Estimator bandwidth [Hz]

% -------------------------------------------------------------------------
% 1.9 PI Controller Parameters
% -------------------------------------------------------------------------
Kp_value = 2;                   % Proportional gain
zc = 2206;                      % Zero location [rad/s]
Ki_value = Kp_value * zc;       % Integral gain (Ki = Kp * zc = 4412)

% -------------------------------------------------------------------------
% 1.10 Simulation Cycles
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
% 1.11 Quality Check Parameters
% -------------------------------------------------------------------------
steady_state_threshold = 0.02;  % Steady-state threshold (2% of amplitude)
dc_tolerance = 0.01;            % DC tolerance (1% of amplitude)

% -------------------------------------------------------------------------
% 1.12 Output Control
% -------------------------------------------------------------------------
SAVE_PNG = true;
SAVE_MAT = true;
output_dir = fullfile(package_root, 'test_results', 'force_generation', 'force_bode');

% -------------------------------------------------------------------------
% 1.13 Plot Style
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

% -------------------------------------------------------------------------
% 1.14 Control Output Low-Pass Filter (in ZOH Plant)
% -------------------------------------------------------------------------
% u_lpf_enable: Simulink LPF switch - automatically linked to lpf_enable
%   When lpf_enable=false: u_lpf_enable=0 (Simulink LPF bypassed)
%   When lpf_enable=true:  u_lpf_enable=1 (Simulink LPF enabled)
%
% f_low: LPF cutoff frequency [Hz] (shared with controller)
%   Transfer function: H(s) = w_low / (s + w_low), where w_low = 2*pi*f_low
%
u_lpf_enable = double(lpf_enable);  % Link to controller lpf_enable
% f_low is already set in LPF Configuration section (1.1.1a)


%% ========================================================================
%                         SECTION 2: Initialization
%% ========================================================================

fprintf('[Configuration]\n');
fprintf('------------------------\n');

% System constants
Ts = 1e-5;                      % Sampling time [s] (100 kHz)

% Validate controller type
controller_type = lower(controller_type);
if ~ismember(controller_type, {'model_base_ctrl', 'pi_controller'})
    error('Invalid controller_type: %s. Use ''model_base_ctrl'' or ''pi_controller''.', controller_type);
end

% Set ControllerType for Simulink (1=Model Based Control, 2=PI-Controller)
if strcmpi(controller_type, 'model_base_ctrl')
    ControllerType = 1;
    % Generate controller label with feedforward mode
    if ~ff_enable
        controller_label = 'Model Based Control (No FF)';
    elseif ff_preview == 0
        controller_label = 'Model Based Control (ZPETC d=0)';
    else
        controller_label = 'Model Based Control (ZPETC d=2)';
    end
else
    ControllerType = 2;
    controller_label = 'PI-Controller';
end

% Auto-select theory curves based on controller configuration
if ControllerType == 1  % Model Based Control
    if lpf_enable
        % LPF mode: use 3rd-order ZPETC theory curve
        THEORY_CURVES = {'zpetc_lpf_d0'};
        if SHOW_COMPARISON_CURVES
            THEORY_CURVES = [THEORY_CURVES, {'zpetc_d0', 'no_ff'}];
        end
    elseif ~ff_enable
        THEORY_CURVES = {'no_ff'};
        if SHOW_COMPARISON_CURVES
            THEORY_CURVES = [THEORY_CURVES, {'zpetc_d0', 'pi'}];
        end
    elseif ff_preview == 0
        THEORY_CURVES = {'zpetc_d0'};
        if SHOW_COMPARISON_CURVES
            THEORY_CURVES = [THEORY_CURVES, {'zpetc_d2', 'no_ff'}];
        end
    else  % ff_preview == 2
        THEORY_CURVES = {'zpetc_d2'};
        if SHOW_COMPARISON_CURVES
            THEORY_CURVES = [THEORY_CURVES, {'zpetc_d0', 'no_ff'}];
        end
    end
else  % PI Controller
    THEORY_CURVES = {'pi'};
    if SHOW_COMPARISON_CURVES
        THEORY_CURVES = [THEORY_CURVES, {'zpetc_d0'}];
    end
end

% Load system parameters (for inverse_model / force_model)
% Use offline mode (no Simulink) for manual inverse_model calls
inv_params = force_model_allocation_params();

% For Simulink integration with Force mode
alloc_params_sim = force_model_allocation_params('Simulink', true, ...
    'pos_m', bead_position, 'SampleRateMode', 2);  % Linear interpolation

% Load Model Based Control parameters (with ff_enable and lpf_enable settings)
model_base_ctrl_params_local = model_base_ctrl_params(fB_c, fB_e, fB_f, ...
    'ff_enable', ff_enable, ...
    'lpf_enable', lpf_enable, ...
    'f_low', f_low);

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
    fprintf('  Model Based Control: fB_f=%d, fB_c=%d, fB_e=%d Hz\n', fB_f, fB_c, fB_e);
    fprintf('  Feedforward: ff_enable=%d, ff_preview=%d\n', ff_enable, ff_preview);
else
    fprintf('  PI-Controller: Kp=%.1f, Ki=%.1f (zc=%d)\n', Kp_value, Ki_value, zc);
end
fprintf('  Theory curves: {%s}\n', strjoin(THEORY_CURVES, ', '));
fprintf('  Generation mode: %s\n', upper(generation_mode));
if strcmpi(generation_mode, 'hardware')
    fprintf('  Hardware constraint: %d Hz position update\n', pos_update_rate);
    if USE_REALTIME_INTERP
        interp_mode_str = 'real-time';
    else
        interp_mode_str = 'ideal';
    end
    fprintf('  Interpolation: %s (%s)\n', interp_method, interp_mode_str);
end
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
% Generate short controller name for folder
if ControllerType == 1
    if ~ff_enable
        ctrl_short_name = 'mbc_noff';
    elseif ff_preview == 0
        ctrl_short_name = 'mbc_d0';
    else
        ctrl_short_name = 'mbc_d2';
    end
else
    ctrl_short_name = 'pi_ctrl';
end
test_folder_name = sprintf('%s_%s_axis_%s', ctrl_short_name, force_axis, test_timestamp);
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
    % 3.1 Define Time Axis
    % ---------------------------------------------------------------------
    % Generate time axis at 100 kHz (controller rate)
    N = round(sim_time / Ts) + 1;
    t = (0:N-1)' * Ts;

    % Hardware constraint parameters for Simulink (set via alloc_params.sample_rate_mode)
    % sample_rate_mode: 1=ZOH (1600 Hz), 2=Linear (1600 Hz), 3=Direct (100 kHz)
    if strcmpi(generation_mode, 'ideal')
        sample_rate_mode = 3;  % Direct (100 kHz)
    elseif strcmpi(interp_method, 'previous')
        sample_rate_mode = 1;  % ZOH
    else
        sample_rate_mode = 2;  % Linear
    end

    % ---------------------------------------------------------------------
    % 3.2 Generate f_d at 100 kHz
    % ---------------------------------------------------------------------
    % f_d: N x 3 matrix [Fx, Fy, Fz] at 100 kHz
    % This is the reference signal for Bode analysis (command input)
    f_d = force_amplitude * sin(2*pi*freq*t) .* force_direction';

    % Note: The hardware constraint (rate transition) is handled internally
    % by inverse_model_function in Simulink via alloc_params.sample_rate_mode.
    % The FFT analysis compares f_d (command) vs f_m (response) to measure
    % the full system frequency response including rate transition effects.

    % ---------------------------------------------------------------------
    % 3.3 Simulink Simulation (Controller) - Using New Force Mode Architecture
    % ---------------------------------------------------------------------
    % Clear workspace variables that shadow functions (see CLAUDE.md naming convention)
    % Note: Use direct clear (not evalin) since script runs in base workspace
    clear vd_signal_params alloc_params pi_ctrl_params motion_control_law_params trajectory_generator_params particle_dynamics_params thermal_force_params

    % Create f_d timeseries for Simulink From Workspace block
    % f_d is at 100 kHz; rate transition is handled by inverse_model_function
    f_d_timeseries = timeseries(f_d, t);
    f_d_timeseries.Name = 'f_d_external';
    assignin('base', 'f_d_timeseries', f_d_timeseries);

    % Set signal_type = 2 (Force mode) - new architecture
    sim_signal_type = 2;  % Force mode
    assignin('base', 'signal_type', sim_signal_type);

    % vd_signal_params (required by Signal mode, but not used in Force mode)
    vd_sig_params = vd_signal_params('Mode', 1, 'Channel', 1, 'Amplitude', 0, ...
        'Frequency', freq, 'Ts', Ts, 'ff_preview', ff_preview);
    assignin('base', 'vd_signal_params', vd_sig_params);

    % alloc_params with correct sample_rate_mode for hardware constraint
    alloc_params_sim.Value.sample_rate_mode = sample_rate_mode;
    assignin('base', 'alloc_params', alloc_params_sim);

    % Controller parameters (both Model Based Control and PI are needed)
    assignin('base', 'model_base_ctrl_params', model_base_ctrl_params_local);

    % PI Controller parameters
    pi_ctrl_params_local = pi_ctrl_params(Kp_value, Ki_value, 'Ts', Ts);
    assignin('base', 'pi_ctrl_params', pi_ctrl_params_local);

    % Set controller type and legacy parameters
    assignin('base', 'ControllerType', ControllerType);
    assignin('base', 'Kp_value', Kp_value);
    assignin('base', 'Ki_value', Ki_value);

    % Control output LPF parameters (for ZOH Plant)
    w_low = 2*pi*f_low;
    assignin('base', 'u_lpf_enable', u_lpf_enable);
    assignin('base', 'f_low', f_low);
    assignin('base', 'w_low', w_low);

    % Motion Control parameters (required by Simulink model even in Force mode)
    motion_ctrl_params = motion_control_law_params('Enable', 0);
    traj_params = trajectory_generator_params();
    particle_params = particle_dynamics_params();
    thermal_params = thermal_force_params('Enable', 0);
    p0 = [0; 0; 5];

    assignin('base', 'motion_control_law_params', motion_ctrl_params);
    assignin('base', 'trajectory_generator_params', traj_params);
    assignin('base', 'particle_dynamics_params', particle_params);
    assignin('base', 'thermal_force_params', thermal_params);
    assignin('base', 'p0', p0);
    assignin('base', 'delay_steps', 0);  % No delay for force bode test

    % pos_m_static timeseries (required by From Workspace blocks)
    pos_m_static = timeseries(zeros(2, 3), [0; sim_time]);
    assignin('base', 'pos_m_static', pos_m_static);

    % Set simulation parameters
    set_param(model_name, 'StopTime', num2str(sim_time));
    % Use ode45 (variable-step) instead of ode5 (fixed-step) to handle
    % Motion Control blocks that run at 1600 Hz (which is not an integer
    % multiple of 100 kHz fixed-step size)
    set_param(model_name, 'Solver', 'ode45');
    set_param(model_name, 'MaxStep', num2str(Ts));

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

    % Extract vm and f_m from simulation output
    % New architecture outputs Fm directly from Force_Model block
    vm_sim = out.Vm;
    N_vm = size(vm_sim, 1);
    t_vm = (0:N_vm-1)' * Ts;

    % Resample vm if needed
    if N_vm ~= N
        vm = zeros(N, 6);
        for ch = 1:6
            vm(:, ch) = interp1(t_vm, vm_sim(:, ch), t, 'linear', 'extrap');
        end
    else
        vm = vm_sim;
    end

    % ---------------------------------------------------------------------
    % 3.4 Force Model: Extract f_m from Simulink output (new architecture)
    % ---------------------------------------------------------------------
    % Try to get Fm directly from simulation output (new model)
    if isfield(out, 'Fm') || isprop(out, 'Fm')
        Fm_ts = out.Fm;
        if isa(Fm_ts, 'timeseries')
            f_m_sim = Fm_ts.Data;
        else
            f_m_sim = Fm_ts;
        end

        % Resample f_m if needed
        N_fm = size(f_m_sim, 1);
        if N_fm ~= N
            f_m = zeros(N, 3);
            t_fm = (0:N_fm-1)' * (sim_time / (N_fm-1));
            for ax = 1:3
                f_m(:, ax) = interp1(t_fm, f_m_sim(:, ax), t, 'linear', 'extrap');
            end
        else
            f_m = f_m_sim;
        end
        fprintf('  Extracted Fm from Simulink output\n');
    else
        % Fallback: compute force_model offline (legacy mode)
        fprintf('  Computing force_model offline... ');
        tic;
        f_m = zeros(N, 3);
        for i = 1:N
            f_m(i, :) = force_model(vm(i, :)', bead_position, inv_params)';
        end
        fprintf('Done (%.2f s)\n', toc);
    end

    % ---------------------------------------------------------------------
    % 3.5 Steady-State Data Selection
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
    % 3.6 Quality Check: Steady-State Detection
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
    % 3.7 FFT Analysis
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
    % 3.8 Cross-Axis Analysis (Crosstalk)
    % ---------------------------------------------------------------------
    for ax = 1:3
        Fm_ax_fft = fft(f_m_steady(:, ax));
        Fm_ax_mag = abs(Fm_ax_fft(freq_bin)) * 2 / N_fft;
        crosstalk_ratio(freq_idx, ax) = Fm_ax_mag / Fd_mag;
    end

    % ---------------------------------------------------------------------
    % 3.9 DC Error Check
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
%                      SECTION 3.5: Theory Curves Computation
%% ========================================================================

fprintf('[Theory Curves Computation]\n');
fprintf('------------------------\n');

% Dense frequency points for smooth theoretical curves
freq_theory = logspace(log10(max(frequencies(1), 0.1)), ...
                       log10(frequencies(end)*1.5), 500);

% Rate transition delay
if strcmpi(generation_mode, 'ideal')
    tau_rate = 0;  % No rate transition delay in ideal mode
elseif USE_REALTIME_INTERP && strcmp(interp_method, 'linear')
    tau_rate = 1 / pos_update_rate;
elseif USE_REALTIME_INTERP && strcmp(interp_method, 'previous')
    tau_rate = 0.5 / pos_update_rate;
else
    tau_rate = 0;
end

% Inner loop parameters from Model Based Control
b_value = model_base_ctrl_params_local.Value.b;
lambda_c = model_base_ctrl_params_local.Value.lambda_c;
kc = model_base_ctrl_params_local.Value.kc;
kf = 1 / (1 + b_value)^2;

% Pre-compute rate transition magnitude response for all frequencies
% Based on PDF derivation: ZOH = sinc(f/fs), Linear = sinc²(f/fs)
H_rate_mag = ones(size(freq_theory));
if strcmpi(generation_mode, 'hardware')
    for i = 1:length(freq_theory)
        x = freq_theory(i) / pos_update_rate;  % Normalized frequency
        if x < 1e-10
            sinc_val = 1;  % lim sinc(x) as x->0 = 1
        else
            sinc_val = sin(pi * x) / (pi * x);
        end

        if strcmpi(interp_method, 'linear')
            % Linear interpolation: sinc²(f/fs)
            H_rate_mag(i) = sinc_val^2;
        else
            % ZOH (previous): sinc(f/fs)
            H_rate_mag(i) = abs(sinc_val);
        end
    end
end

% Theory curves computation loop
theory_curves_data = struct([]);
theory_line_styles = {'-', '--', ':', '-.'};

for curve_idx = 1:length(THEORY_CURVES)
    curve_type = THEORY_CURVES{curve_idx};
    A_curve = zeros(size(freq_theory));
    phi_curve = zeros(size(freq_theory));

    switch curve_type
        case 'zpetc_d0'
            % ZPETC with preview=0: phase = φ_rate - 2*θ_inner
            for i = 1:length(freq_theory)
                f = freq_theory(i);
                theta = 2 * pi * f * Ts;
                phi_rate = -2 * pi * f * tau_rate;
                A_ctrl = kf * (1 + 2*b_value*cos(theta) + b_value^2);
                A_curve(i) = H_rate_mag(i) * A_ctrl;  % Include rate transition
                phi_curve(i) = phi_rate + (-2 * theta);
            end
            label = 'Theory (ZPETC d=0)';

        case 'zpetc_d2'
            % ZPETC with preview=2: phase = φ_rate only
            for i = 1:length(freq_theory)
                f = freq_theory(i);
                theta = 2 * pi * f * Ts;
                phi_rate = -2 * pi * f * tau_rate;
                A_ctrl = kf * (1 + 2*b_value*cos(theta) + b_value^2);
                A_curve(i) = H_rate_mag(i) * A_ctrl;  % Include rate transition
                phi_curve(i) = phi_rate;
            end
            label = 'Theory (ZPETC d=2)';

        case 'no_ff'
            % No feedforward: closed-loop inner + rate transition
            % Hcl = z^-1 * kc * (1+b*z^-1) / (1-λc*z^-1)
            for i = 1:length(freq_theory)
                f = freq_theory(i);
                theta = 2 * pi * f * Ts;
                z_inv = exp(-1j * theta);
                Hcl = (z_inv * kc * (1 + b_value * z_inv)) / (1 - lambda_c * z_inv);
                phi_rate = -2 * pi * f * tau_rate;
                A_curve(i) = H_rate_mag(i) * abs(Hcl);  % Include rate transition
                phi_curve(i) = phi_rate + angle(Hcl);
            end
            label = 'Theory (No FF)';

        case 'pi'
            % PI controller closed-loop
            for i = 1:length(freq_theory)
                f = freq_theory(i);
                w = 2 * pi * f;
                jw = 1j * w;
                C_jw = Kp_value * (jw + zc) / jw;
                P_jw = exp(-1j * w * tau_rate);
                L_jw = C_jw * P_jw;
                H_jw = L_jw / (1 + L_jw);
                A_curve(i) = H_rate_mag(i) * abs(H_jw);  % Include rate transition
                phi_curve(i) = angle(H_jw);
            end
            label = 'Theory (PI)';

        otherwise
            warning('Unknown theory curve type: %s', curve_type);
            continue;
    end

    idx = length(theory_curves_data) + 1;
    theory_curves_data(idx).type = curve_type;
    theory_curves_data(idx).label = label;
    theory_curves_data(idx).A = A_curve;
    theory_curves_data(idx).phi_deg = unwrap(phi_curve) * (180/pi);
    theory_curves_data(idx).line_style = theory_line_styles{min(curve_idx, 4)};
end

fprintf('  Computed %d theory curves\n', length(theory_curves_data));
fprintf('    tau_rate = %.4f s (%.2f µs)\n', tau_rate, tau_rate*1e6);
if strcmpi(generation_mode, 'hardware')
    if strcmpi(interp_method, 'linear')
        fprintf('    Magnitude response: sinc²(f/fs) [Linear Interp]\n');
    else
        fprintf('    Magnitude response: sinc(f/fs) [ZOH]\n');
    end
    % Show attenuation at key frequencies
    test_f = [100, 200, 250];
    for tf = test_f
        [~, idx] = min(abs(freq_theory - tf));
        fprintf('    H_rate_mag @ %d Hz: %.4f (%.1f%%)\n', tf, H_rate_mag(idx), H_rate_mag(idx)*100);
    end
else
    fprintf('    Magnitude response: 1 (no attenuation) [Ideal]\n');
end
fprintf('\n');


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

% Plot all theory curves first (bottom layer)
h_theory_mag = gobjects(length(theory_curves_data), 1);
for curve_idx = 1:length(theory_curves_data)
    h_theory_mag(curve_idx) = semilogx(freq_theory, theory_curves_data(curve_idx).A, ...
        theory_curves_data(curve_idx).line_style, ...
        'LineWidth', unified_linewidth, ...
        'Color', theory_color, ...
        'DisplayName', theory_curves_data(curve_idx).label);
end

% Plot all 3 axes responses (Fm_x/Fd, Fm_y/Fd, Fm_z/Fd)
% Use markers only (no connecting lines)
% Plot order: large non-excited -> excited -> small non-excited (top layer)
% Small markers drawn last so they won't be hidden by large ones
plot_handles_mag = gobjects(3, 1);

% Determine plot order: large markers first, small markers last (top layer)
non_excited_axes = setdiff([1, 2, 3], axis_idx);
% Plot order: second non-excited (large) -> excited (large) -> first non-excited (small, top)
plot_order = [non_excited_axes(2), axis_idx, non_excited_axes(1)];

for plot_idx = 1:3
    ax = plot_order(plot_idx);
    mag = crosstalk_ratio(:, ax);  % This contains |Fm_ax / Fd_excited|

    if ax == non_excited_axes(1)
        % First non-excited axis: smaller markers (80%), drawn last (top layer)
        marker_size = unified_markersize * 0.8;
        line_width = unified_linewidth * 0.8;
    else
        % Excited axis and second non-excited: full size markers
        marker_size = unified_markersize;
        line_width = unified_linewidth;
    end

    plot_handles_mag(ax) = semilogx(frequencies, mag, force_markers{ax}, ...
        'LineStyle', 'none', ...
        'Color', force_colors(ax, :), ...
        'MarkerFaceColor', 'none', ...
        'MarkerEdgeColor', force_colors(ax, :), ...
        'MarkerSize', marker_size, ...
        'LineWidth', line_width, ...
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
% Combine axis labels and theory curve labels
theory_labels = {theory_curves_data.label};
lgd = legend([plot_handles_mag; h_theory_mag], ...
    [axis_labels, theory_labels], ...
    'Location', 'northoutside', ...
    'NumColumns', 3 + length(theory_curves_data), ...
    'FontSize', 13, 'FontWeight', 'bold', 'Orientation', 'horizontal');
lgd.EdgeColor = [0 0 0];  % Black border
lgd.LineWidth = 2.0;      % Thick border

% -------------------------------------------------------------------------
% Lower plot: Phase (only excited axis)
% -------------------------------------------------------------------------
subplot('Position', [0.1, 0.1, 0.85, 0.35]);
hold on; grid off;

% Plot all theory phase curves first (bottom layer)
for curve_idx = 1:length(theory_curves_data)
    semilogx(freq_theory, theory_curves_data(curve_idx).phi_deg, ...
        theory_curves_data(curve_idx).line_style, ...
        'LineWidth', unified_linewidth, ...
        'Color', theory_color, ...
        'DisplayName', theory_curves_data(curve_idx).label);
end

% Plot phase of excited axis only (markers only, no connecting lines)
semilogx(frequencies, phase_lag_deg, force_markers{axis_idx}, ...
    'LineStyle', 'none', ...
    'Color', force_colors(axis_idx, :), ...
    'MarkerFaceColor', 'none', ...
    'MarkerEdgeColor', force_colors(axis_idx, :), ...
    'MarkerSize', unified_markersize, ...
    'LineWidth', unified_linewidth, ...
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
ylim([-75, 0]);  % Fixed Y-axis range for phase plot

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
        results.config.generation_mode = generation_mode;
        results.config.pos_update_rate = pos_update_rate;
        results.config.interp_method = interp_method;
        results.config.USE_REALTIME_INTERP = USE_REALTIME_INTERP;
        results.config.total_cycles = total_cycles;
        results.config.skip_cycles = skip_cycles;
        results.config.fft_cycles = fft_cycles;
        results.config.Ts = Ts;
        results.config.ff_enable = ff_enable;
        results.config.ff_preview = ff_preview;

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

        % Theory data (unified structure for all controller modes)
        results.theory.freq = freq_theory;
        results.theory.tau_rate = tau_rate;
        results.theory.curves = theory_curves_data;
        results.theory.b_value = b_value;
        results.theory.THEORY_CURVES = THEORY_CURVES;

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
