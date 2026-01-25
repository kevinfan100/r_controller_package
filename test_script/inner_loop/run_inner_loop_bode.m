% run_inner_loop_bode.m
% Inner loop controller frequency response test (Bode plot analysis)
%
% This script performs frequency sweep analysis (10 Hz ~ 2 kHz) and generates
% Bode plots with quality checks for each frequency point.
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
d = 0;
channel = 2;
amplitude = 2;

% Controller selection: 1 = R-Controller, 2 = PI Controller
USE_PI_CONTROLLER = false;  % Set to true for PI controller test

% R-Controller bandwidths
fB_f = 1000;
fB_c = 3200;
fB_e = 16000;

% PI controller parameters
Kp_value = config.Kp_default;
Ki_value = config.Ki_default;
Ts = config.Ts;

% Compute controller parameters (both are needed by Simulink model)
ctrl_params_model_base = model_base_ctrl_params(fB_c, fB_e, fB_f);
ctrl_params_pi = pi_ctrl_params(Kp_value, Ki_value, 'Ts', Ts);

% Extract values for theory curve calculation
b_value = ctrl_params_model_base.Value.b;
lambda_f = ctrl_params_model_base.Value.lambda_f;

% Output settings
SAVE_RESULTS = true;

%% Initialization

fprintf('\n');
fprintf('================================================================\n');
if USE_PI_CONTROLLER
    fprintf('         PI Controller Frequency Response Test (Bode Plot)\n');
else
    fprintf('          R Controller Frequency Response Test (Bode Plot)\n');
end
fprintf('================================================================\n\n');

fprintf('[Configuration]\n');
fprintf('------------------------\n');
if USE_PI_CONTROLLER
    fprintf('  Controller: PI (Kp=%.1f, Ki=%.1f)\n', Kp_value, Ki_value);
else
    fprintf('  Controller: R-Controller (fB: f=%d, c=%d, e=%d Hz)\n', fB_f, fB_c, fB_e);
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
    folder_name = sprintf('d%d_ch%d_%s', d, channel, timestamp);
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

% Initialize result arrays
magnitude_ratio_all = zeros(num_freq, 6);
phase_lag_all = zeros(num_freq, 6);
sim_times = zeros(num_freq, 1);

% Quality check results
quality_results = cell(num_freq, 1);

fprintf('================================================================\n');
fprintf('  Starting frequency sweep (d = %d)\n', d);
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
        'd', d);

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
    fft_results = fft_analysis(v_d_steady(:, channel), v_m_steady, config.fs, freq);

    fprintf('\n  FFT: Target %.2f Hz, Bin %.2f Hz (error: %.3f%%)\n', ...
            freq, fft_results.actual_freq, fft_results.freq_error_pct);

    if fft_results.freq_error_pct > config.freq_error_threshold
        fprintf('  Warning: Frequency error exceeds %.3f%%\n', config.freq_error_threshold);
    end

    % Store results
    magnitude_ratio_all(freq_idx, :) = fft_results.magnitude_ratio;
    phase_lag_all(freq_idx, :) = fft_results.phase_lag_deg;

    fprintf('  P%d gain: %.2f%%, phase: %.2f deg\n\n', ...
            channel, fft_results.magnitude_ratio(channel)*100, ...
            fft_results.phase_lag_deg(channel));
end

%% Compute Theory Curves

freq_theory = logspace(0, log10(4000), 500);
A_theory = zeros(size(freq_theory));
phi_theory = zeros(size(freq_theory));

% 系統實際延遲（純模擬中無延遲 = 0，preview 是用來補償延遲的）
system_delay = 0;

if ~USE_PI_CONTROLLER
    % ─────────────────────────────────────────────────────────────────────
    % R-Controller: Zero Phase Feedforward 理論曲線
    % ─────────────────────────────────────────────────────────────────────
    lambda_f = 0;
    kf = (1 - lambda_f) / (1 + b_value)^2;

    for i = 1:length(freq_theory)
        theta = 2 * pi * freq_theory(i) * config.Ts;
        % Zero Phase Feedforward: A(θ) = kf * (1 + 2b*cos(θ) + b²)
        A_theory(i) = kf * (1 + 2*b_value*cos(theta) + b_value^2);
        % Zero Phase Feedforward: φ(θ) = -(2 + d) * θ
        phi_theory(i) = -(2 + system_delay) * theta;
    end
    theory_label = 'Theory (R-Ctrl)';
else
    % ─────────────────────────────────────────────────────────────────────
    % PI Controller: 閉環傳遞函數理論曲線
    % Plant: H(z) = k_o * (1 + b*z^-1) / (1 - a1*z^-1 - a2*z^-2)
    % PI: C(z) = Kp + Ki*Ts*z^-1/(1-z^-1)
    % Closed-loop: T(z) = C(z)*H(z) / (1 + C(z)*H(z))
    % ─────────────────────────────────────────────────────────────────────
    k_o = ctrl_params_model_base.Value.k_o;
    a1 = ctrl_params_model_base.Value.a1;
    a2 = ctrl_params_model_base.Value.a2;

    for i = 1:length(freq_theory)
        w = 2 * pi * freq_theory(i);
        z = exp(1j * w * Ts);

        % Plant transfer function H(z)
        H_plant = k_o * (1 + b_value / z) / (1 - a1 / z - a2 / z^2);

        % PI controller: C(z) = Kp + Ki*Ts/(1 - z^-1)
        C_pi = Kp_value + Ki_value * Ts / (1 - 1/z);

        % Closed-loop transfer function
        L = C_pi * H_plant;
        T_closed = L / (1 + L);

        A_theory(i) = abs(T_closed);
        phi_theory(i) = angle(T_closed);
    end
    theory_label = 'Theory (PI)';
    lambda_f = NaN;  % Not applicable for PI
    kf = NaN;
end

phi_theory_deg = phi_theory * (180/pi);

%% Build Results Structure

results = struct();
results.d_value = d;
results.frequencies = frequencies;
results.magnitude_ratio = magnitude_ratio_all;
results.phase_lag = phase_lag_all;
results.magnitude_dB = 20 * log10(magnitude_ratio_all);
results.sim_times = sim_times;
results.channel = channel;
results.USE_PI_CONTROLLER = USE_PI_CONTROLLER;

if USE_PI_CONTROLLER
    results.controller_type = 'PI';
    results.Kp = Kp_value;
    results.Ki = Ki_value;
else
    results.controller_type = 'R-Controller';
    results.fB_c = fB_c;
    results.fB_e = fB_e;
    results.fB_f = fB_f;
end

% Theory comparison
results.theory.b_value = b_value;
results.theory.freq_theory = freq_theory;
results.theory.A_theory = A_theory;
results.theory.phi_theory_deg = phi_theory_deg;
results.theory.label = theory_label;

% Error analysis (compare experiment with theory at experiment frequencies)
H_theory_exp = interp1(freq_theory, A_theory, frequencies, 'linear', 'extrap');
error_percent = abs(magnitude_ratio_all(:, channel) - H_theory_exp') ./ H_theory_exp' * 100;
results.theory.H_magnitude = H_theory_exp;
results.theory.error_percent = error_percent;
results.theory.max_error_percent = max(error_percent);
results.theory.mean_error_percent = mean(error_percent);
results.theory.rms_error_percent = sqrt(mean(error_percent.^2));

% Quality check results
results.quality = quality_results;

%% Generate Bode Plot

fprintf('================================================================\n');
fprintf('  Generating Bode Plot\n');
fprintf('================================================================\n\n');

fig_bode = create_bode_plot(frequencies, magnitude_ratio_all, phase_lag_all, ...
                            freq_theory, A_theory, phi_theory_deg, ...
                            channel, styles, theory_label, USE_PI_CONTROLLER, ...
                            fB_f, Kp_value, Ki_value);
fprintf('  Bode plot created\n');

%% Display Analysis Results

fprintf('\n[Frequency Response Analysis]\n');
fprintf('------------------------\n');

mag_dB = results.magnitude_dB(:, channel);

% Find -3dB bandwidth using interpolation
idx_below_3dB = find(mag_dB < -3, 1, 'first');
if ~isempty(idx_below_3dB) && idx_below_3dB > 1
    idx_above = idx_below_3dB - 1;
    f1 = frequencies(idx_above);
    f2 = frequencies(idx_below_3dB);
    mag_dB1 = mag_dB(idx_above);
    mag_dB2 = mag_dB(idx_below_3dB);
    f_3dB = f1 + (f2 - f1) * (-3 - mag_dB1) / (mag_dB2 - mag_dB1);
    fprintf('  -3dB bandwidth: %.2f Hz (interpolated)\n', f_3dB);
elseif ~isempty(idx_below_3dB)
    fprintf('  -3dB bandwidth: < %.2f Hz\n', frequencies(1));
else
    fprintf('  -3dB bandwidth: > %.2f Hz\n', frequencies(end));
end

% DC and HF gains
fprintf('  Low-freq gain (%.0f Hz): %.2f dB (%.2f%%)\n', ...
        frequencies(1), mag_dB(1), 10^(mag_dB(1)/20)*100);
fprintf('  High-freq gain (%.0f Hz): %.2f dB (%.2f%%)\n', ...
        frequencies(end), mag_dB(end), 10^(mag_dB(end)/20)*100);

% Phase statistics
phase_ch = results.phase_lag(:, channel);
fprintf('  Phase range: %.2f to %.2f deg\n', min(phase_ch), max(phase_ch));

% Theory comparison
if USE_PI_CONTROLLER
    fprintf('\n[Theory Comparison (PI: Kp=%.1f, Ki=%.1f)]\n', Kp_value, Ki_value);
else
    fprintf('\n[Theory Comparison (R-Ctrl: b = %.4f)]\n', b_value);
end
fprintf('  Max error: %.2f%%\n', results.theory.max_error_percent);
fprintf('  Mean error: %.2f%%\n', results.theory.mean_error_percent);
fprintf('  RMS error: %.2f%%\n\n', results.theory.rms_error_percent);

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

    % Save Bode plot
    exportgraphics(fig_bode, fullfile(output_dir, 'bode_plot.png'), ...
                   'Resolution', styles.export_resolution);
    fprintf('  Bode plot saved: bode_plot.png (%d DPI)\n', styles.export_resolution);

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
    fprintf('  Controller: R-Controller (d=%d)\n', d);
    fprintf('  Bandwidths: fB_c=%d Hz, fB_e=%d Hz, fB_f=%d Hz\n', fB_c, fB_e, fB_f);
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

function fig = create_bode_plot(frequencies, mag_ratio, phase_lag, ...
                                freq_theory, A_theory, phi_theory_deg, ...
                                channel, styles, theory_label, USE_PI, ...
                                fB_f, Kp, Ki)
% Create Bode plot figure
    if USE_PI
        fig_title = sprintf('PI Controller (Kp=%.1f, Ki=%.1f) - P%d', Kp, Ki, channel);
    else
        fig_title = sprintf('R-Controller (fB_f=%.0fHz) - P%d', fB_f, channel);
    end
    fig = figure('Name', fig_title, 'Position', [100, 100, 1200, 800]);

    colors = styles.channel_colors;
    markers = styles.channel_markers;
    lw = styles.bode_linewidth;
    ms = styles.bode_marker_size;
    theory_color = styles.theory_color;

    % Magnitude plot (upper)
    subplot('Position', [0.1, 0.55, 0.85, 0.35]);
    hold on;

    % Theory curve (solid line)
    h_theory = semilogx(freq_theory, A_theory, '-', ...
                        'LineWidth', lw, 'Color', theory_color, ...
                        'DisplayName', theory_label);

    % Experiment data (dashed lines with markers)
    h_ch = gobjects(6, 1);
    for ch = 1:6
        if ch == channel
            label = sprintf('P%d (Excited)', ch);
        else
            label = sprintf('P%d', ch);
        end
        h_ch(ch) = semilogx(frequencies, mag_ratio(:, ch), ['--' markers{ch}], ...
                            'LineWidth', lw, 'Color', colors(ch, :), ...
                            'MarkerFaceColor', 'none', ...
                            'MarkerEdgeColor', colors(ch, :), ...
                            'MarkerSize', ms, ...
                            'DisplayName', label);
    end

    ylabel('Magnitude', 'FontSize', styles.bode_label_fontsize, 'FontWeight', 'bold');
    xlim([frequencies(1), frequencies(end)]);
    ylim([0, 1.25]);
    yticks([0, 0.25, 0.5, 0.75, 1.0, 1.25]);

    ax1 = gca;
    ax1.XScale = 'log';
    ax1.XTick = [1, 10, 100, 1000, 10000];
    ax1.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
    ax1.FontSize = styles.bode_tick_fontsize;
    ax1.FontWeight = 'bold';
    ax1.LineWidth = 2.5;
    ax1.Box = 'on';

    % Legend
    legend([h_ch; h_theory], [styles.channel_labels, {theory_label}], ...
           'Location', 'northoutside', 'NumColumns', 7, ...
           'FontSize', styles.bode_legend_fontsize, 'FontWeight', 'bold', ...
           'Orientation', 'horizontal');

    % Phase plot (lower, excited channel only)
    subplot('Position', [0.1, 0.1, 0.85, 0.35]);
    hold on;

    % Theory curve
    semilogx(freq_theory, phi_theory_deg, '-', ...
             'LineWidth', lw, 'Color', theory_color, ...
             'DisplayName', theory_label);

    % Experiment data (excited channel)
    semilogx(frequencies, phase_lag(:, channel), ['--' markers{channel}], ...
             'LineWidth', lw, 'Color', colors(channel, :), ...
             'MarkerFaceColor', 'none', ...
             'MarkerEdgeColor', colors(channel, :), ...
             'MarkerSize', ms, ...
             'DisplayName', sprintf('P%d (Excited)', channel));

    xlabel('Frequency (Hz)', 'FontSize', styles.bode_label_fontsize, 'FontWeight', 'bold');
    ylabel('Phase (deg)', 'FontSize', styles.bode_label_fontsize, 'FontWeight', 'bold');
    xlim([frequencies(1), frequencies(end)]);

    ax2 = gca;
    ax2.XScale = 'log';
    ax2.XTick = [1, 10, 100, 1000, 10000];
    ax2.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
    ax2.FontSize = styles.bode_tick_fontsize;
    ax2.FontWeight = 'bold';
    ax2.LineWidth = 2.5;
    ax2.Box = 'on';
end
