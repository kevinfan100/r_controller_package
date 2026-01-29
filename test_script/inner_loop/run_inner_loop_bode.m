% run_inner_loop_bode.m
% Inner loop controller frequency response test (Bode plot analysis)
%
% This script performs frequency sweep analysis (10 Hz ~ 4 kHz) and generates
% Bode plots with quality checks for each frequency point.
%
% Features:
%   - FFT analysis compares Vm vs true reference r(t) = sin(2*pi*f*t)
%   - Multiple preview values support (d=0, d=2) with formula-based phase calc
%   - Multiple theory curves: ZPETC (d=0/d=2), No FF, PI
%   - Preview comparison plots (log and linear frequency axis)
%
% Configuration:
%   - ControllerType: 1 = Model Based Control, 2 = PI Controller
%   - ff_enable: true = feedforward enabled (ZPETC)
%   - ff_preview: 0 (no preview) or 2 (ZPETC zero-phase tracking)
%   - fft_preview_values: Preview values to display in plots [0, 2]
%   - THEORY_CURVES: Theory curves to display {'zpetc_d0', 'no_ff', 'zpetc_d2', 'pi'}
%   - SHOW_PREVIEW_COMPARISON: Show preview comparison plots (ZPETC only)
%
% For single-frequency analysis, use run_inner_loop_test.m instead.
%
% See also: run_inner_loop_test, model_base_ctrl_params, CLAUDE.md

clear; clc; close all;

%% Path Setup

script_dir = fileparts(mfilename('fullpath'));
package_root = fullfile(script_dir, '..', '..');
addpath(fullfile(package_root, 'model'));
addpath(fullfile(package_root, 'model', 'inner_loop_ctrl'));
addpath(fullfile(package_root, 'model', 'flux_allocation'));
addpath(fullfile(package_root, 'model', 'motion_ctrl'));
addpath(fullfile(package_root, 'model', 'particle_dynamics'));
addpath(fullfile(package_root, 'test_script', 'utils'));

%% Configuration

% Load default configuration and styles
config = test_config('Type', 'inner_loop');
styles = plot_styles();

% Frequency sweep points (all produce integer samples_per_cycle)
frequencies = config.inner_loop_frequencies;

% Signal parameters
channel = 5;
amplitude = 2;

% -------------------------------------------------------------------------
% Controller Configuration
% -------------------------------------------------------------------------
% ControllerType: 1 = Model Based Control, 2 = PI Controller
% ff_enable: true = feedforward enabled (ZPETC), false = feedforward bypassed
ControllerType = 1;  % 1 = Model Based Control, 2 = PI Controller
ff_enable = true;    % Enable feedforward filter (only for Model Based Control)

% -------------------------------------------------------------------------
% Feedforward Preview Configuration (only for Model Based Control)
% -------------------------------------------------------------------------
% ff_preview: Preview steps for Vd generator (0 or 2)
%   0 = No preview (Vm lags behind r(t) by ~2*theta)
%   2 = ZPETC preview (Vm tracks r(t) with zero phase)
ff_preview = 0;  % Actual preview used in simulation

% FFT Preview Analysis Configuration
% Only run simulation with ff_preview=0, use formula for ff_preview=2 phase
% Formula: phase(preview=N) = phase(preview=0) + N*theta, where theta = 2*pi*f*Ts
fft_preview_values = [0, 2];  % Preview values to display in plots
use_formula_for_preview = true;  % Use formula for preview>0 phase calculation

% -------------------------------------------------------------------------
% Theory Curves Configuration
% -------------------------------------------------------------------------
% Theory curves are auto-selected based on controller settings.
% Available options: 'zpetc_d0', 'zpetc_d2', 'no_ff', 'pi'
%   'zpetc_d0' - ZPETC with preview=0 (phase = -2*theta)
%   'zpetc_d2' - ZPETC with preview=2 (phase = 0, true zero-phase)
%   'no_ff'    - No feedforward (closed-loop only)
%   'pi'       - PI controller closed-loop
THEORY_CURVES = {};  % Auto-populated based on controller settings

% Show preview comparison plots (ZPETC only)
SHOW_PREVIEW_COMPARISON = true;

% Model Based Control bandwidths
fB_f = 1000;
fB_c = 3200;
fB_e = 16000;

% PI controller parameters
Kp_value = config.Kp_default;
Ki_value = config.Ki_default;
Ts = config.Ts;

% Compute controller parameters (both are needed by Simulink model)
ctrl_params_model_base = model_base_ctrl_params(fB_c, fB_e, fB_f, 'ff_enable', ff_enable);
ctrl_params_pi = pi_ctrl_params(Kp_value, Ki_value, 'Ts', Ts);

% Derive USE_PI_CONTROLLER for backward compatibility
USE_PI_CONTROLLER = (ControllerType == 2);

% Extract values for theory curve calculation
b_value = ctrl_params_model_base.Value.b;
lambda_f = ctrl_params_model_base.Value.lambda_f;

% Output settings
SAVE_RESULTS = true;

% -------------------------------------------------------------------------
% Control Output Low-Pass Filter (in ZOH Plant)
% -------------------------------------------------------------------------
% u_lpf_enable: Enable continuous LPF on control output (after DAC)
%   0 = Bypass (no filtering, default)
%   1 = Apply LPF (s-domain first-order)
%
% f_low: LPF cutoff frequency [Hz]
%   Transfer function: H(s) = w_low / (s + w_low), where w_low = 2*pi*f_low
%
u_lpf_enable = 0;                % 0=bypass (default), 1=enable LPF
f_low = 10000;                   % Cutoff frequency [Hz]

%% Initialization

% Auto-select theory curves and adjust settings based on controller type
if USE_PI_CONTROLLER
    % PI Controller
    fft_preview_values = [0];  % PI: only preview=0
    use_formula_for_preview = false;
    SHOW_PREVIEW_COMPARISON = false;
    THEORY_CURVES = {'pi'};
else
    % Model Based Control: select based on ff_enable and ff_preview
    if ~ff_enable
        THEORY_CURVES = {'no_ff'};
    elseif ff_preview == 0
        THEORY_CURVES = {'zpetc_d0'};
    else  % ff_preview == 2
        THEORY_CURVES = {'zpetc_d2'};
    end
end
num_fft_previews = length(fft_preview_values);

fprintf('\n');
fprintf('================================================================\n');
if USE_PI_CONTROLLER
    fprintf('         PI Controller Frequency Response Test (Bode Plot)\n');
else
    fprintf('     Model Based Controller Frequency Response Test (Bode Plot)\n');
end
fprintf('================================================================\n\n');

fprintf('[Configuration]\n');
fprintf('------------------------\n');
if USE_PI_CONTROLLER
    fprintf('  Controller: PI (Kp=%.1f, Ki=%.1f)\n', Kp_value, Ki_value);
else
    if ff_enable
        fprintf('  Controller: Model Based Control (ZPETC, ff_preview=%d)\n', ff_preview);
    else
        fprintf('  Controller: Model Based Control (No FF)\n');
    end
    fprintf('  Bandwidths: fB_f=%d, fB_c=%d, fB_e=%d Hz\n', fB_f, fB_c, fB_e);
    fprintf('  FFT preview values: [%s] (use_formula=%s)\n', ...
            num2str(fft_preview_values), mat2str(use_formula_for_preview));
    fprintf('  Theory curves: {%s}\n', strjoin(THEORY_CURVES, ', '));
end
fprintf('  Frequency range: %.0f Hz ~ %.0f Hz (%d points)\n', ...
        frequencies(1), frequencies(end), length(frequencies));
fprintf('  Excited channel: P%d\n', channel);
fprintf('  Amplitude: %.2f V\n', amplitude);
fprintf('  Total cycles: %d (skip %d, analyze %d)\n', ...
        config.total_cycles, config.skip_cycles, config.fft_cycles);
fprintf('  Model parameter b: %.4f\n\n', b_value);

% Load Simulink model
model_name = config.model_name;
model_path = fullfile(package_root, 'model', [model_name '.slx']);

if ~exist(model_path, 'file')
    error('Model file not found: %s', model_path);
end

if ~bdIsLoaded(model_name)
    load_system(model_path);
end
fprintf('  Model loaded: %s\n', model_name);

% Create output directory
if SAVE_RESULTS
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    folder_name = sprintf('ff%d_ch%d_%s', ff_preview, channel, timestamp);
    output_dir = fullfile(package_root, 'test_results', 'inner_loop', ...
                         'frequency_response', folder_name);
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    diagnostic_dir = fullfile(output_dir, 'diagnostics');
    if ~exist(diagnostic_dir, 'dir')
        mkdir(diagnostic_dir);
    end
    fprintf('  Output: %s\n', output_dir);
end
fprintf('\n');

%% Frequency Sweep

num_freq = length(frequencies);

% Initialize result arrays (3D: freq x channel x fft_preview)
magnitude_ratio_all = zeros(num_freq, 6, num_fft_previews);
phase_lag_all = zeros(num_freq, 6, num_fft_previews);
sim_times = zeros(num_freq, 1);

% Quality check results
quality_results = cell(num_freq, 1);

fprintf('================================================================\n');
fprintf('  Starting frequency sweep (ff_preview = %d)\n', ff_preview);
fprintf('================================================================\n\n');

for freq_idx = 1:num_freq
    freq = frequencies(freq_idx);
    period = 1 / freq;

    % Compute simulation time
    sim_time = config.total_cycles * period;
    sim_time = max(config.min_sim_time, min(config.max_sim_time, sim_time));
    sim_times(freq_idx) = sim_time;

    fprintf('--------------------------------------------------------\n');
    fprintf('[%2d/%2d] Frequency: %8.2f Hz (Period: %.4f s, Sim: %.2f s)\n', ...
            freq_idx, num_freq, freq, period, sim_time);
    fprintf('--------------------------------------------------------\n');

    % Clear workspace variables that shadow functions (see CLAUDE.md naming convention)
    % These variables are created by assignin() in the loop and shadow the
    % parameter functions on subsequent iterations.
    % Note: Use direct clear (not evalin) since script runs in base workspace
    clear vd_signal_params alloc_params motion_control_law_params trajectory_generator_params particle_dynamics_params thermal_force_params

    % Create vd_signal_params for this frequency
    vd_sig_params = vd_signal_params( ...
        'Mode', 1, ...  % Sine mode
        'Channel', channel, ...
        'Amplitude', amplitude, ...
        'Frequency', freq, ...
        'Phase', 0, ...
        'StepTime', 0, ...
        'Ts', Ts, ...
        'ff_preview', ff_preview);

    % Create alloc_params (required by Force_Model block)
    alloc_params_sim = force_model_allocation_params('Simulink', true, ...
        'pos_m', [0; 0; 0], 'SampleRateMode', 2);

    % Assign workspace variables for Simulink
    % Controller parameters (both are needed)
    assignin('base', 'model_base_ctrl_params', ctrl_params_model_base);
    assignin('base', 'pi_ctrl_params', ctrl_params_pi);

    % Select controller type
    if USE_PI_CONTROLLER
        assignin('base', 'ControllerType', config.controller_type_enum.PI_CTRL);
    else
        assignin('base', 'ControllerType', config.controller_type_enum.MODEL_BASE_CTRL);
    end

    % signal_type: 1=Signal mode (inner loop test)
    assignin('base', 'signal_type', 1);
    assignin('base', 'vd_signal_params', vd_sig_params);
    assignin('base', 'alloc_params', alloc_params_sim);

    % f_d timeseries (required by Force mode, not used here)
    f_d_timeseries = timeseries(zeros(2, 3), [0; sim_time]);
    assignin('base', 'f_d_timeseries', f_d_timeseries);

    % Motion Control parameters (required by Simulink model even in Signal mode)
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
    assignin('base', 'delay_steps', 0);  % No delay for inner loop test

    % pos_m_static timeseries (required by From Workspace blocks)
    pos_m_static = timeseries(zeros(2, 3), [0; sim_time]);
    assignin('base', 'pos_m_static', pos_m_static);

    % Control output LPF parameters (for ZOH Plant)
    w_low = 2*pi*f_low;              % Convert to rad/s
    assignin('base', 'u_lpf_enable', u_lpf_enable);
    assignin('base', 'f_low', f_low);
    assignin('base', 'w_low', w_low);

    % Configure Simulink
    % Use ode45 (variable-step) instead of ode5 (fixed-step) to handle
    % Motion Control blocks that run at 1600 Hz (which is not an integer
    % multiple of 100 kHz fixed-step size)
    set_param(model_name, 'StopTime', num2str(sim_time));
    set_param(model_name, 'Solver', 'ode45');
    set_param(model_name, 'MaxStep', num2str(config.Ts));

    % Run simulation
    fprintf('  Running simulation...\n');
    tic;
    try
        out = sim(model_name);
        elapsed = toc;
        fprintf('  Completed in %.2f s\n', elapsed);
    catch ME
        fprintf('  Simulation failed: %s\n', ME.message);
        continue;
    end

    % Extract data
    v_d = out.Vd;
    v_m = out.Vm;
    N = size(v_d, 1);
    t = (0:N-1)' * config.Ts;

    % Adjust cycle counts if sim_time was limited
    actual_total_cycles = t(end) / period;
    if actual_total_cycles < config.total_cycles
        adj_skip_cycles = floor(actual_total_cycles * 0.6);
        adj_fft_cycles = max(2, floor(actual_total_cycles * 0.35));
        fprintf('  Adjusted cycles: skip=%d, fft=%d\n', adj_skip_cycles, adj_fft_cycles);
    else
        adj_skip_cycles = config.skip_cycles;
        adj_fft_cycles = config.fft_cycles;
    end

    % Select steady-state data
    t_start = adj_skip_cycles * period;
    t_end = min(t_start + adj_fft_cycles * period, t(end));
    idx_steady = (t >= t_start) & (t <= t_end);

    if sum(idx_steady) < 100
        fprintf('  Insufficient steady-state data (%d points), skipping\n\n', sum(idx_steady));
        continue;
    end

    v_d_steady = v_d(idx_steady, :);
    v_m_steady = v_m(idx_steady, :);

    fprintf('  Steady-state: %.2f ~ %.2f s (%d points)\n', ...
            t_start, t_end, sum(idx_steady));

    % Quality check
    qc = quality_check(v_m_steady, config.fs, amplitude, ...
        'Frequency', freq, ...
        'SteadyThreshold', config.steady_threshold, ...
        'THDThreshold', config.thd_threshold, ...
        'DCTolerance', config.dc_tolerance);
    quality_results{freq_idx} = qc;

    % Display quality check results
    display_quality_results(qc, channel);

    % Save diagnostic plots for failed steady-state checks
    if SAVE_RESULTS
        save_diagnostic_plots(qc, v_m_steady, freq, period, config.Ts, diagnostic_dir);
    end

    % FFT analysis
    % IMPORTANT: Compare Vm vs true reference r(t) = sin(2*pi*f*t), NOT vs Vd
    % This correctly shows the preview effect:
    %   - ff_preview=0: Vm lags r(t) by ~2*theta (tracking error)
    %   - ff_preview=2: Vm tracks r(t) with zero phase (ZPETC)
    t_steady = (0:size(v_d_steady, 1)-1)' * config.Ts;
    r_t = amplitude * sin(2 * pi * freq * t_steady);  % True reference signal

    % Use r(t) as reference instead of v_d
    fft_results = fft_analysis(r_t, v_m_steady, config.fs, freq);

    fprintf('\n  FFT: Target %.2f Hz, Bin %.2f Hz (error: %.3f%%)\n', ...
            freq, fft_results.actual_freq, fft_results.freq_error_pct);

    if fft_results.freq_error_pct > config.freq_error_threshold
        fprintf('  Warning: Frequency error exceeds %.3f%%\n', config.freq_error_threshold);
    end

    % Store results for preview=0 (first index)
    magnitude_ratio_all(freq_idx, :, 1) = fft_results.magnitude_ratio;
    phase_lag_all(freq_idx, :, 1) = fft_results.phase_lag_deg;

    % Calculate phase for other preview values using formula
    % Formula: phase(preview=N) = phase(preview=0) + N*theta
    % where theta = 2*pi*f*Ts (phase shift per sample in degrees)
    if use_formula_for_preview && num_fft_previews > 1
        theta_deg = 2 * pi * freq * config.Ts * (180 / pi);  % Phase per sample (deg)

        for preview_idx = 2:num_fft_previews
            fft_preview_val = fft_preview_values(preview_idx);
            phase_shift = fft_preview_val * theta_deg;  % N samples phase shift

            for ch = 1:6
                % Magnitude unchanged by preview
                magnitude_ratio_all(freq_idx, ch, preview_idx) = magnitude_ratio_all(freq_idx, ch, 1);

                % Phase = preview=0 phase + preview*theta
                phase_raw = phase_lag_all(freq_idx, ch, 1) + phase_shift;

                % Normalize to [-180, 180]
                while phase_raw > 180
                    phase_raw = phase_raw - 360;
                end
                while phase_raw < -180
                    phase_raw = phase_raw + 360;
                end
                phase_lag_all(freq_idx, ch, preview_idx) = phase_raw;
            end
        end
    end

    fprintf('  P%d gain: %.2f%%, phase: %.2f deg (Vm vs r(t))\n\n', ...
            channel, fft_results.magnitude_ratio(channel)*100, ...
            fft_results.phase_lag_deg(channel));
end

%% Compute Theory Curves

freq_theory = logspace(0, log10(5000), 500);

% Initialize theory curve storage
theory_curves_data = struct();

% System parameters for theory calculation
lambda_c = exp(-fB_c * config.Ts * 2 * pi);
kc = (1 - lambda_c) / (1 + b_value);

if ~USE_PI_CONTROLLER
    % ─────────────────────────────────────────────────────────────────────
    % Model Based Control: Multiple theory curves based on THEORY_CURVES
    % ─────────────────────────────────────────────────────────────────────
    lambda_f = 0;
    kf = (1 - lambda_f) / (1 + b_value)^2;

    for curve_idx = 1:length(THEORY_CURVES)
        curve_type = THEORY_CURVES{curve_idx};
        A_curve = zeros(size(freq_theory));
        phi_curve = zeros(size(freq_theory));

        switch curve_type
            case 'zpetc_d0'
                % ZPETC with preview=0: phase = -2*theta
                for i = 1:length(freq_theory)
                    theta = 2 * pi * freq_theory(i) * config.Ts;
                    A_curve(i) = kf * (1 + 2*b_value*cos(theta) + b_value^2);
                    phi_curve(i) = -2 * theta;  % -2*theta (deg)
                end
                label = 'Theory (preview=0)';

            case 'zpetc_d2'
                % ZPETC with preview=2: phase = 0 (true zero-phase)
                for i = 1:length(freq_theory)
                    theta = 2 * pi * freq_theory(i) * config.Ts;
                    A_curve(i) = kf * (1 + 2*b_value*cos(theta) + b_value^2);
                    phi_curve(i) = 0;  % Zero phase
                end
                label = 'Theory (preview=2)';

            case 'no_ff'
                % No feedforward: closed-loop only
                % Hcl = z^-1 * kc * (1+b*z^-1) / (1-λc*z^-1)
                for i = 1:length(freq_theory)
                    theta = 2 * pi * freq_theory(i) * config.Ts;
                    z_inv = exp(-1j * theta);

                    Hcl_num = z_inv * kc * (1 + b_value * z_inv);
                    Hcl_den = 1 - lambda_c * z_inv;
                    Hcl = Hcl_num / Hcl_den;

                    A_curve(i) = abs(Hcl);
                    phi_curve(i) = angle(Hcl);
                end
                label = 'Theory (No FF)';

            case 'pi'
                % PI controller closed-loop
                k_o = ctrl_params_model_base.Value.k_o;
                a1 = ctrl_params_model_base.Value.a1;
                a2 = ctrl_params_model_base.Value.a2;

                for i = 1:length(freq_theory)
                    w = 2 * pi * freq_theory(i);
                    z = exp(1j * w * Ts);

                    H_plant = k_o * (1 + b_value / z) / (1 - a1 / z - a2 / z^2);
                    C_pi = Kp_value + Ki_value * Ts / (1 - 1/z);
                    L = C_pi * H_plant;
                    T_closed = L / (1 + L);

                    A_curve(i) = abs(T_closed);
                    phi_curve(i) = angle(T_closed);
                end
                label = 'Theory (PI)';

            otherwise
                warning('Unknown theory curve type: %s', curve_type);
                continue;
        end

        theory_curves_data(curve_idx).type = curve_type;
        theory_curves_data(curve_idx).label = label;
        theory_curves_data(curve_idx).A = A_curve;
        theory_curves_data(curve_idx).phi_deg = phi_curve * (180/pi);
    end

    % Primary theory for error analysis (use first curve)
    A_theory = theory_curves_data(1).A;
    phi_theory_deg = theory_curves_data(1).phi_deg;
    theory_label = theory_curves_data(1).label;

else
    % ─────────────────────────────────────────────────────────────────────
    % PI Controller: single theory curve
    % ─────────────────────────────────────────────────────────────────────
    k_o = ctrl_params_model_base.Value.k_o;
    a1 = ctrl_params_model_base.Value.a1;
    a2 = ctrl_params_model_base.Value.a2;

    A_theory = zeros(size(freq_theory));
    phi_theory = zeros(size(freq_theory));

    for i = 1:length(freq_theory)
        w = 2 * pi * freq_theory(i);
        z = exp(1j * w * Ts);

        H_plant = k_o * (1 + b_value / z) / (1 - a1 / z - a2 / z^2);
        C_pi = Kp_value + Ki_value * Ts / (1 - 1/z);
        L = C_pi * H_plant;
        T_closed = L / (1 + L);

        A_theory(i) = abs(T_closed);
        phi_theory(i) = angle(T_closed);
    end

    phi_theory_deg = phi_theory * (180/pi);
    theory_label = 'Theory (PI)';
    lambda_f = NaN;
    kf = NaN;

    % Store single curve for PI
    theory_curves_data(1).type = 'pi';
    theory_curves_data(1).label = theory_label;
    theory_curves_data(1).A = A_theory;
    theory_curves_data(1).phi_deg = phi_theory_deg;
end

%% Build Results Structure

results = struct();
results.ff_preview = ff_preview;
results.fft_preview_values = fft_preview_values;
results.frequencies = frequencies;
results.magnitude_ratio = magnitude_ratio_all;  % 3D: freq x ch x fft_preview
results.phase_lag = phase_lag_all;              % 3D: freq x ch x fft_preview
results.magnitude_dB = 20 * log10(magnitude_ratio_all);
results.sim_times = sim_times;
results.channel = channel;
results.USE_PI_CONTROLLER = USE_PI_CONTROLLER;

if USE_PI_CONTROLLER
    results.controller_type = 'PI';
    results.Kp = Kp_value;
    results.Ki = Ki_value;
else
    results.controller_type = 'Model Based Control';
    results.ff_enable = ff_enable;
    results.fB_c = fB_c;
    results.fB_e = fB_e;
    results.fB_f = fB_f;
end

% Theory curves data
results.theory.b_value = b_value;
results.theory.freq_theory = freq_theory;
results.theory.curves = theory_curves_data;
results.theory.primary_label = theory_label;

% Error analysis (compare experiment with primary theory at experiment frequencies)
H_theory_exp = interp1(freq_theory, A_theory, frequencies, 'linear', 'extrap');

% Calculate error for each fft_preview value
for preview_idx = 1:num_fft_previews
    mag_ch = magnitude_ratio_all(:, channel, preview_idx);
    error_pct = abs(mag_ch - H_theory_exp') ./ H_theory_exp' * 100;
    results.theory.error_percent(:, preview_idx) = error_pct;
    results.theory.max_error_percent(preview_idx) = max(error_pct);
    results.theory.mean_error_percent(preview_idx) = mean(error_pct);
    results.theory.rms_error_percent(preview_idx) = sqrt(mean(error_pct.^2));
end
results.theory.H_magnitude = H_theory_exp;

% Quality check results
results.quality = quality_results;

%% Generate Tabbed Figure

fprintf('================================================================\n');
fprintf('  Generating Bode Plot (Tabbed Interface)\n');
fprintf('================================================================\n\n');

% Use preview=0 data for main Bode plot (first index)
mag_ratio_main = magnitude_ratio_all(:, :, 1);
phase_lag_main = phase_lag_all(:, :, 1);

% Create main uifigure with tabs
if USE_PI_CONTROLLER
    fig_title = sprintf('PI Controller Bode (Kp=%.1f, Ki=%.1f) - P%d', Kp_value, Ki_value, channel);
else
    fig_title = sprintf('Model Based Control Bode (fB_f=%d Hz) - P%d', fB_f, channel);
end

fig_main = uifigure('Name', fig_title, 'Position', [50 50 1400 900]);
tabgroup = uitabgroup(fig_main);
tabgroup.Units = 'normalized';
tabgroup.Position = [0 0 1 1];

% Tab 1: Main Bode Plot
create_bode_tab(tabgroup, frequencies, mag_ratio_main, phase_lag_main, ...
                freq_theory, theory_curves_data, channel, styles);
fprintf('  Tab 1: Bode Plot created\n');

% Preview comparison tabs (ZPETC only)
if SHOW_PREVIEW_COMPARISON && ~USE_PI_CONTROLLER && num_fft_previews > 1
    % Tab 2: Preview Comparison (Log scale)
    create_preview_comparison_tab(tabgroup, frequencies, magnitude_ratio_all, phase_lag_all, ...
                                  freq_theory, theory_curves_data, fft_preview_values, ...
                                  channel, styles, config.Ts, 'log');
    fprintf('  Tab 2: Preview Comparison (Log) created\n');

    % Tab 3: Preview Comparison (Linear scale)
    create_preview_comparison_tab(tabgroup, frequencies, magnitude_ratio_all, phase_lag_all, ...
                                  freq_theory, theory_curves_data, fft_preview_values, ...
                                  channel, styles, config.Ts, 'linear');
    fprintf('  Tab 3: Preview Comparison (Linear) created\n');
end

fprintf('  All tabs created\n');

%% Display Analysis Results

fprintf('\n[Frequency Response Analysis]\n');
fprintf('------------------------\n');

% Display for each fft_preview value
for preview_idx = 1:num_fft_previews
    fft_preview_val = fft_preview_values(preview_idx);
    fprintf('\n  [FFT preview = %d samples]\n', fft_preview_val);

    mag_dB = results.magnitude_dB(:, channel, preview_idx);

    % Find -3dB bandwidth using interpolation
    idx_below_3dB = find(mag_dB < -3, 1, 'first');
    if ~isempty(idx_below_3dB) && idx_below_3dB > 1
        idx_above = idx_below_3dB - 1;
        f1 = frequencies(idx_above);
        f2 = frequencies(idx_below_3dB);
        mag_dB1 = mag_dB(idx_above);
        mag_dB2 = mag_dB(idx_below_3dB);
        f_3dB = f1 + (f2 - f1) * (-3 - mag_dB1) / (mag_dB2 - mag_dB1);
        fprintf('    -3dB bandwidth: %.2f Hz (interpolated)\n', f_3dB);
    elseif ~isempty(idx_below_3dB)
        fprintf('    -3dB bandwidth: < %.2f Hz\n', frequencies(1));
    else
        fprintf('    -3dB bandwidth: > %.2f Hz\n', frequencies(end));
    end

    % DC and HF gains
    fprintf('    Low-freq gain (%.0f Hz): %.2f dB (%.2f%%)\n', ...
            frequencies(1), mag_dB(1), 10^(mag_dB(1)/20)*100);
    fprintf('    High-freq gain (%.0f Hz): %.2f dB (%.2f%%)\n', ...
            frequencies(end), mag_dB(end), 10^(mag_dB(end)/20)*100);

    % Phase statistics
    phase_ch = results.phase_lag(:, channel, preview_idx);
    fprintf('    Phase range: %.2f to %.2f deg\n', min(phase_ch), max(phase_ch));
    fprintf('    Mean phase: %.2f deg\n', mean(phase_ch));

    % Theory comparison
    if USE_PI_CONTROLLER
        fprintf('\n    [Theory Comparison (PI: Kp=%.1f, Ki=%.1f)]\n', Kp_value, Ki_value);
    else
        fprintf('\n    [Theory Comparison (Model Based Ctrl: b = %.4f)]\n', b_value);
    end
    fprintf('    Max error: %.2f%%\n', results.theory.max_error_percent(preview_idx));
    fprintf('    Mean error: %.2f%%\n', results.theory.mean_error_percent(preview_idx));
    fprintf('    RMS error: %.2f%%\n', results.theory.rms_error_percent(preview_idx));
end
fprintf('\n');

%% Quality Statistics

fprintf('[Quality Check Statistics]\n');
fprintf('------------------------\n');

for ch = 1:6
    [steady_rate, thd_rate, dc_rate, overall_rate] = compute_quality_stats(quality_results, ch, num_freq);

    fprintf('\n  P%d', ch);
    if ch == channel
        fprintf(' (excited)');
    end
    fprintf(':\n');
    fprintf('    Steady-state: %.0f%%\n', steady_rate);
    fprintf('    THD: %.0f%%\n', thd_rate);
    fprintf('    DC: %.0f%%\n', dc_rate);
    fprintf('    Overall: %.0f%%\n', overall_rate);
end

fprintf('\n');

%% Save Results

if SAVE_RESULTS
    fprintf('[Saving Results]\n');
    fprintf('------------------------\n');

    % Save data
    save(fullfile(output_dir, 'freq_sweep_data.mat'), 'results', '-v7.3');
    fprintf('  Data saved: freq_sweep_data.mat\n');

    % Export each tab as PNG
    tabs = tabgroup.Children;
    for i = 1:length(tabs)
        tab_title = matlab.lang.makeValidName(tabs(i).Title);
        png_filename = sprintf('%s.png', tab_title);
        exportgraphics(tabs(i), fullfile(output_dir, png_filename), ...
                       'Resolution', styles.export_resolution);
        fprintf('  Tab %d saved: %s\n', i, png_filename);
    end

    fprintf('  Output: %s\n\n', output_dir);
end

%% Summary

fprintf('================================================================\n');
fprintf('                     Test Completed\n');
fprintf('================================================================\n\n');

fprintf('[Summary]\n');
if USE_PI_CONTROLLER
    fprintf('  Controller: PI (Kp=%.1f, Ki=%.1f)\n', Kp_value, Ki_value);
else
    if ff_enable
        fprintf('  Controller: Model Based Control (ZPETC, ff_preview=%d)\n', ff_preview);
    else
        fprintf('  Controller: Model Based Control (No FF)\n');
    end
    fprintf('  Bandwidths: fB_c=%d Hz, fB_e=%d Hz, fB_f=%d Hz\n', fB_c, fB_e, fB_f);
    fprintf('  FFT preview values: [%s]\n', num2str(fft_preview_values));
    fprintf('  Theory curves: {%s}\n', strjoin(THEORY_CURVES, ', '));
end
fprintf('  Excited channel: P%d\n', channel);
fprintf('  Frequency range: %.0f ~ %.0f Hz (%d points)\n', ...
        frequencies(1), frequencies(end), num_freq);
fprintf('  Total simulation time: %.2f min\n\n', sum(sim_times)/60);


% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function display_quality_results(qc, channel)
% Display quality check results in table format
    fprintf('\n  Quality Check:\n');
    fprintf('    Ch | Steady | THD          | DC Error | Status\n');
    fprintf('    ---+--------+--------------+----------+-------\n');

    for ch = 1:6
        steady_mark = status_mark(qc.steady_state_pass(ch));
        thd_mark = status_mark(qc.thd_pass(ch));
        dc_mark = status_mark(qc.dc_pass(ch));

        if ch == channel
            status = status_string(qc.overall_pass(ch), 'PASS', 'WARN');
        else
            status = status_string(qc.overall_pass(ch), 'OK', 'FAIL');
        end

        thd_str = format_thd_value(qc.thd_values(ch));

        fprintf('    P%d |   %s    | %s %s | %.4fV %s | %s\n', ...
                ch, steady_mark, thd_str, thd_mark, ...
                qc.dc_error(ch), dc_mark, status);
    end
end

function mark = status_mark(condition)
% Return status mark based on condition
    if condition
        mark = 'v';
    else
        mark = 'x';
    end
end

function str = status_string(condition, pass_str, fail_str)
% Return status string based on condition
    if condition
        str = pass_str;
    else
        str = fail_str;
    end
end

function str = format_thd_value(thd_val)
% Format THD value for display
    if isnan(thd_val)
        str = '    N/A    ';
    elseif thd_val < 0.01
        str = sprintf('%10.2e%%', thd_val);
    else
        str = sprintf('%10.4f%%', thd_val);
    end
end

function save_diagnostic_plots(qc, v_m, freq, period, Ts, diagnostic_dir)
% Save diagnostic plots for failed steady-state checks
    samples_per_cycle = round(period / Ts);
    num_cycles = floor(size(v_m, 1) / samples_per_cycle);

    for ch = 1:6
        if ~qc.steady_state_pass(ch)
            fig_diag = figure('Visible', 'off', 'Position', [100, 100, 800, 600]);
            hold on; grid on;

            for k = 1:min(num_cycles, 10)
                idx_start = (k-1) * samples_per_cycle + 1;
                idx_end = k * samples_per_cycle;

                if idx_end <= size(v_m, 1)
                    cycle_data = v_m(idx_start:idx_end, ch);
                    t_cycle = (0:length(cycle_data)-1)' * Ts * 1000;

                    color_val = (k-1) / max(1, min(num_cycles, 10)-1);
                    plot(t_cycle, cycle_data, 'LineWidth', 1.5, ...
                         'Color', [color_val, 0, 1-color_val]);
                end
            end

            xlabel('Time within Cycle [ms]', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Vm [V]', 'FontSize', 12, 'FontWeight', 'bold');
            title(sprintf('Cycle Overlay - %.1f Hz, P%d (NOT STEADY)', freq, ch), ...
                  'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');

            diag_filename = sprintf('steady_fail_%.1fHz_P%d.png', freq, ch);
            exportgraphics(fig_diag, fullfile(diagnostic_dir, diag_filename), 'Resolution', 300);
            close(fig_diag);
        end
    end
end

function [steady_rate, thd_rate, dc_rate, overall_rate] = compute_quality_stats(qc_results, ch, num_freq)
% Compute quality check pass rates for a channel
    steady_count = 0;
    thd_count = 0;
    dc_count = 0;
    overall_count = 0;

    for i = 1:num_freq
        if ~isempty(qc_results{i})
            if qc_results{i}.steady_state_pass(ch)
                steady_count = steady_count + 1;
            end
            if qc_results{i}.thd_pass(ch)
                thd_count = thd_count + 1;
            end
            if qc_results{i}.dc_pass(ch)
                dc_count = dc_count + 1;
            end
            if qc_results{i}.overall_pass(ch)
                overall_count = overall_count + 1;
            end
        end
    end

    steady_rate = steady_count / num_freq * 100;
    thd_rate = thd_count / num_freq * 100;
    dc_rate = dc_count / num_freq * 100;
    overall_rate = overall_count / num_freq * 100;
end

function create_bode_tab(tabgroup, frequencies, mag_ratio, phase_lag, ...
                         freq_theory, theory_curves_data, channel, styles)
% Create Bode plot tab with magnitude and phase subplots
    tab = uitab(tabgroup, 'Title', 'Bode Plot');

    colors = styles.channel_colors;
    markers = styles.channel_markers;
    lw = styles.bode_linewidth;
    ms = styles.bode_marker_size;
    theory_color = styles.theory_color;
    theory_line_styles = {'-', '--', ':', '-.'};

    % Use tiledlayout for better control
    tl = tiledlayout(tab, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    % === Upper: Magnitude ===
    ax1 = nexttile(tl);
    hold(ax1, 'on');

    % Theory curves
    num_theory = length(theory_curves_data);
    h_theory = gobjects(num_theory, 1);
    theory_labels = cell(num_theory, 1);

    for curve_idx = 1:num_theory
        line_style = theory_line_styles{min(curve_idx, length(theory_line_styles))};
        h_theory(curve_idx) = semilogx(ax1, freq_theory, theory_curves_data(curve_idx).A, line_style, ...
                        'LineWidth', lw, 'Color', theory_color, ...
                        'DisplayName', theory_curves_data(curve_idx).label);
        theory_labels{curve_idx} = theory_curves_data(curve_idx).label;
    end

    % Experiment data (dashed lines with markers)
    h_ch = gobjects(6, 1);
    for ch = 1:6
        if ch == channel
            label = sprintf('P%d (Excited)', ch);
        else
            label = sprintf('P%d', ch);
        end
        h_ch(ch) = semilogx(ax1, frequencies, mag_ratio(:, ch), ['--' markers{ch}], ...
                            'LineWidth', lw, 'Color', colors(ch, :), ...
                            'MarkerFaceColor', 'none', ...
                            'MarkerEdgeColor', colors(ch, :), ...
                            'MarkerSize', ms, ...
                            'DisplayName', label);
    end

    ylabel(ax1, 'Magnitude', 'FontSize', styles.bode_label_fontsize, 'FontWeight', 'bold');
    xlim(ax1, [frequencies(1), frequencies(end)]);
    ylim(ax1, [0, 1.25]);
    ax1.YTick = [0, 0.25, 0.5, 0.75, 1.0, 1.25];
    ax1.XScale = 'log';
    ax1.XTick = [1, 10, 100, 1000, 10000];
    ax1.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
    ax1.FontSize = styles.bode_tick_fontsize;
    ax1.FontWeight = 'bold';
    ax1.LineWidth = 2.5;
    ax1.Box = 'on';

    % Legend
    legend(ax1, [h_ch; h_theory], [styles.channel_labels, theory_labels'], ...
           'Location', 'northoutside', 'NumColumns', 6 + num_theory, ...
           'FontSize', styles.bode_legend_fontsize, 'FontWeight', 'bold', ...
           'Orientation', 'horizontal');

    % === Lower: Phase (excited channel only) ===
    ax2 = nexttile(tl);
    hold(ax2, 'on');

    % Theory phase curves
    for curve_idx = 1:num_theory
        line_style = theory_line_styles{min(curve_idx, length(theory_line_styles))};
        semilogx(ax2, freq_theory, theory_curves_data(curve_idx).phi_deg, line_style, ...
                 'LineWidth', lw, 'Color', theory_color, ...
                 'DisplayName', theory_curves_data(curve_idx).label);
    end

    % Experiment data (excited channel)
    semilogx(ax2, frequencies, phase_lag(:, channel), ['--' markers{channel}], ...
             'LineWidth', lw, 'Color', colors(channel, :), ...
             'MarkerFaceColor', 'none', ...
             'MarkerEdgeColor', colors(channel, :), ...
             'MarkerSize', ms, ...
             'DisplayName', sprintf('P%d (Excited)', channel));

    xlabel(ax2, 'Frequency (Hz)', 'FontSize', styles.bode_label_fontsize, 'FontWeight', 'bold');
    ylabel(ax2, 'Phase (deg)', 'FontSize', styles.bode_label_fontsize, 'FontWeight', 'bold');
    xlim(ax2, [frequencies(1), frequencies(end)]);
    ax2.XScale = 'log';
    ax2.XTick = [1, 10, 100, 1000, 10000];
    ax2.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
    ax2.FontSize = styles.bode_tick_fontsize;
    ax2.FontWeight = 'bold';
    ax2.LineWidth = 2.5;
    ax2.Box = 'on';
end

function create_preview_comparison_tab(tabgroup, frequencies, mag_ratio_all, phase_lag_all, ...
                                       freq_theory, theory_curves_data, fft_preview_values, ...
                                       channel, styles, Ts, scale_type)
% Create preview comparison tab (log or linear frequency scale)
    if strcmp(scale_type, 'log')
        tab_title = 'Preview Comparison (Log)';
    else
        tab_title = 'Preview Comparison (Linear)';
    end
    tab = uitab(tabgroup, 'Title', tab_title);

    % Preview colors and markers
    preview_colors = [
        0.0000, 0.4470, 0.7410;  % preview=0: blue
        0.8500, 0.3250, 0.0980;  % preview=2: orange
        0.4660, 0.6740, 0.1880;  % other: green
        0.4940, 0.1840, 0.5560;  % purple
    ];
    preview_markers = {'o', 's', '^', 'd'};
    theory_color = styles.theory_color;
    lw = styles.bode_linewidth;
    ms = styles.bode_marker_size;
    num_fft_previews = length(fft_preview_values);

    % Use tiledlayout
    tl = tiledlayout(tab, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    % === Upper: Magnitude ===
    ax1 = nexttile(tl);
    hold(ax1, 'on');

    % Theory curves (look for zpetc_d0 and zpetc_d2)
    for curve_idx = 1:length(theory_curves_data)
        curve_type = theory_curves_data(curve_idx).type;
        if strcmp(curve_type, 'zpetc_d0')
            if strcmp(scale_type, 'log')
                semilogx(ax1, freq_theory, theory_curves_data(curve_idx).A, '-', ...
                         'LineWidth', lw, 'Color', theory_color, ...
                         'DisplayName', 'Theory (preview=0)');
            else
                plot(ax1, freq_theory, theory_curves_data(curve_idx).A, '-', ...
                     'LineWidth', lw, 'Color', theory_color, ...
                     'DisplayName', 'Theory (preview=0)');
            end
        end
    end

    % Experiment data for each preview value
    for preview_idx = 1:num_fft_previews
        fft_preview_val = fft_preview_values(preview_idx);
        mag_ch = mag_ratio_all(:, channel, preview_idx);
        color_idx = min(preview_idx, size(preview_colors, 1));

        if strcmp(scale_type, 'log')
            semilogx(ax1, frequencies, mag_ch, ['--' preview_markers{color_idx}], ...
                     'LineWidth', 3.0, 'Color', preview_colors(color_idx, :), ...
                     'MarkerFaceColor', 'none', ...
                     'MarkerEdgeColor', preview_colors(color_idx, :), ...
                     'MarkerSize', ms, ...
                     'DisplayName', sprintf('preview=%d', fft_preview_val));
        else
            plot(ax1, frequencies, mag_ch, ['--' preview_markers{color_idx}], ...
                 'LineWidth', 3.0, 'Color', preview_colors(color_idx, :), ...
                 'MarkerFaceColor', 'none', ...
                 'MarkerEdgeColor', preview_colors(color_idx, :), ...
                 'MarkerSize', ms, ...
                 'DisplayName', sprintf('preview=%d', fft_preview_val));
        end
    end

    ylabel(ax1, 'Magnitude', 'FontSize', styles.bode_label_fontsize, 'FontWeight', 'bold');
    if strcmp(scale_type, 'log')
        xlim(ax1, [frequencies(1), frequencies(end)]);
        ax1.XScale = 'log';
        ax1.XTick = [1, 10, 100, 1000, 10000];
        ax1.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
    else
        xlim(ax1, [0, frequencies(end)]);
    end
    ylim(ax1, [0, 1.25]);
    ax1.YTick = [0, 0.25, 0.5, 0.75, 1.0, 1.25];
    ax1.FontSize = styles.bode_tick_fontsize;
    ax1.FontWeight = 'bold';
    ax1.LineWidth = 2.5;
    ax1.Box = 'on';

    legend(ax1, 'Location', 'northoutside', 'NumColumns', num_fft_previews + 1, ...
           'FontSize', styles.bode_legend_fontsize, 'FontWeight', 'bold', ...
           'Orientation', 'horizontal');

    % === Lower: Phase ===
    ax2 = nexttile(tl);
    hold(ax2, 'on');

    % Theory phase curves - compute for appropriate frequency axis
    if strcmp(scale_type, 'log')
        % For log scale, compute phase at freq_theory points
        phi_theory_d0_log = -2 * (2 * pi * freq_theory * Ts) * (180/pi);
        phi_theory_d2_log = zeros(size(freq_theory));

        semilogx(ax2, freq_theory, phi_theory_d0_log, '-', ...
                 'LineWidth', lw, 'Color', theory_color, ...
                 'DisplayName', 'Theory (preview=0)');
        semilogx(ax2, freq_theory, phi_theory_d2_log, '--', ...
                 'LineWidth', lw, 'Color', theory_color, ...
                 'DisplayName', 'Theory (preview=2)');
    else
        % For linear scale, use linear frequency axis
        freq_linear = linspace(0, frequencies(end), 500);
        phi_theory_d0_lin = -2 * (2 * pi * freq_linear * Ts) * (180/pi);
        phi_theory_d2_lin = zeros(size(freq_linear));

        plot(ax2, freq_linear, phi_theory_d0_lin, '-', ...
             'LineWidth', lw, 'Color', theory_color, ...
             'DisplayName', 'Theory (preview=0)');
        plot(ax2, freq_linear, phi_theory_d2_lin, '--', ...
             'LineWidth', lw, 'Color', theory_color, ...
             'DisplayName', 'Theory (preview=2)');
    end

    % Experiment phase for each preview value
    for preview_idx = 1:num_fft_previews
        fft_preview_val = fft_preview_values(preview_idx);
        phase_ch = phase_lag_all(:, channel, preview_idx);
        color_idx = min(preview_idx, size(preview_colors, 1));

        if strcmp(scale_type, 'log')
            semilogx(ax2, frequencies, phase_ch, ['--' preview_markers{color_idx}], ...
                     'LineWidth', 3.0, 'Color', preview_colors(color_idx, :), ...
                     'MarkerFaceColor', 'none', ...
                     'MarkerEdgeColor', preview_colors(color_idx, :), ...
                     'MarkerSize', ms, ...
                     'DisplayName', sprintf('preview=%d', fft_preview_val));
        else
            plot(ax2, frequencies, phase_ch, ['--' preview_markers{color_idx}], ...
                 'LineWidth', 3.0, 'Color', preview_colors(color_idx, :), ...
                 'MarkerFaceColor', 'none', ...
                 'MarkerEdgeColor', preview_colors(color_idx, :), ...
                 'MarkerSize', ms, ...
                 'DisplayName', sprintf('preview=%d', fft_preview_val));
        end
    end

    xlabel(ax2, 'Frequency (Hz)', 'FontSize', styles.bode_label_fontsize, 'FontWeight', 'bold');
    ylabel(ax2, 'Phase (deg)', 'FontSize', styles.bode_label_fontsize, 'FontWeight', 'bold');
    if strcmp(scale_type, 'log')
        xlim(ax2, [frequencies(1), frequencies(end)]);
        ax2.XScale = 'log';
        ax2.XTick = [1, 10, 100, 1000, 10000];
        ax2.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
    else
        xlim(ax2, [0, frequencies(end)]);
    end
    ax2.FontSize = styles.bode_tick_fontsize;
    ax2.FontWeight = 'bold';
    ax2.LineWidth = 2.5;
    ax2.Box = 'on';
end
