% run_inner_loop_test.m
% Inner loop controller single-frequency test (sine/step mode)
%
% This script tests the R-Controller performance at a single frequency.
% For frequency sweep analysis, use run_inner_loop_bode.m instead.
%
% See also: run_inner_loop_bode, model_base_ctrl_params, CLAUDE.md

clear; clc; close all;

%% Path Setup

script_dir = fileparts(mfilename('fullpath'));
package_root = fullfile(script_dir, '..', '..');
addpath(fullfile(package_root, 'model'));
addpath(fullfile(package_root, 'model', 'inner_loop_ctrl'));
addpath(fullfile(package_root, 'model', 'flux_allocation'));
addpath(fullfile(package_root, 'test_script', 'utils'));

%% Configuration

test_name = 'test';
signal_type_name = 'sine';          % 'step' or 'sine'

% Signal parameters
d = 0;                              % Preview steps
channel = 1;                        % Excitation channel (1-6)
amplitude = 1;                      % Amplitude [V]
frequency = 100;                    % Sine frequency [Hz]
phase = 0;                          % Sine phase [deg]
step_time = 0;                      % Step time [s]

% Simulation parameters
step_sim_time = 0.5;                % Step mode simulation time [s]

% Load default configuration and styles
config = test_config('Type', 'inner_loop');
styles = plot_styles();

% Override controller bandwidths if needed
fB_f = 1000;
fB_c = 300;
fB_e = 500;

% Compute lambda values for display
Ts = config.Ts;
lambda_f = exp(-fB_f * Ts * 2 * pi);
lambda_c = exp(-fB_c * Ts * 2 * pi);
lambda_e = exp(-fB_e * Ts * 2 * pi);
beta = sqrt(lambda_e * lambda_c);

% Compute controller parameters
ctrl_params = model_base_ctrl_params(fB_c, fB_e, fB_f);

% PI controller parameters (alternative)
Kp_value = config.Kp_default;
Ki_value = config.Ki_default;

% Output control
ENABLE_PLOT = true;
SAVE_PNG = true;
SAVE_MAT = true;

%% Initialization and Validation

fprintf('\n');
fprintf('================================================================\n');
fprintf('           R Controller Inner Loop Test\n');
fprintf('================================================================\n\n');

% Convert signal type to numeric for Simulink
if strcmpi(signal_type_name, 'sine')
    SignalType = 1;
else
    SignalType = 2;
end

% Validation
if ~ismember(lower(signal_type_name), {'step', 'sine'})
    error('signal_type_name must be ''step'' or ''sine''');
end

if channel < 1 || channel > 6
    error('channel must be between 1 and 6');
end

% For Simulink (uses PascalCase)
Channel = channel;
Amplitude = amplitude;
Frequency = frequency;
Phase = phase;
StepTime = step_time;

% Display configuration
fprintf('[Workspace Variables]\n');
fprintf('------------------------\n');
fprintf('  SignalType: %d (%s)\n', SignalType, signal_type_name);
fprintf('  Channel: %d\n', channel);
fprintf('  Amplitude: %.3f V\n', amplitude);

if strcmpi(signal_type_name, 'sine')
    fprintf('  Frequency: %.1f Hz\n', frequency);
    fprintf('  Phase: %.1f deg\n', phase);
else
    fprintf('  StepTime: %.3f s\n', step_time);
end

fprintf('  d (preview): %d\n', d);
fprintf('  fB_f: %d Hz, fB_c: %d Hz, fB_e: %d Hz\n', fB_f, fB_c, fB_e);
fprintf('  lambda_f: %.6f, lambda_c: %.6f, lambda_e: %.6f\n', ...
        lambda_f, lambda_c, lambda_e);
fprintf('  beta: %.6f\n\n', beta);

% Create output directory
if strcmpi(signal_type_name, 'sine')
    output_subdir = fullfile('test_results', 'inner_loop', 'sine_wave');
else
    output_subdir = fullfile('test_results', 'inner_loop', 'step_response');
end

if SAVE_PNG || SAVE_MAT
    output_dir = fullfile(package_root, output_subdir);
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    test_dir = fullfile(output_dir, sprintf('%s_%s', test_name, timestamp));
    mkdir(test_dir);
    fprintf('Output directory: %s\n\n', test_dir);
end

%% Compute Simulation Time

fprintf('[Simulation Time]\n');
fprintf('------------------------\n');

if strcmpi(signal_type_name, 'sine')
    period = 1 / frequency;
    sim_time = config.total_cycles * period;
    sim_time = max(config.min_sim_time, min(config.max_sim_time, sim_time));

    fprintf('  Frequency: %.1f Hz, Period: %.6f s\n', frequency, period);
    fprintf('  Total cycles: %d (skip %d, analyze %d)\n', ...
            config.total_cycles, config.skip_cycles, config.fft_cycles);
    fprintf('  Simulation time: %.4f s\n\n', sim_time);
else
    sim_time = step_sim_time;
    fprintf('  Step time: %.3f s\n', step_time);
    fprintf('  Simulation time: %.3f s\n\n', sim_time);
end

%% Configure Simulink Model

fprintf('[Simulink Model Configuration]\n');
fprintf('------------------------\n');

model_name = config.model_name;
model_path = fullfile(package_root, 'model', [model_name '.slx']);

if ~exist(model_path, 'file')
    error('Model file not found: %s', model_path);
end

if ~bdIsLoaded(model_name)
    load_system(model_path);
end
fprintf('  Model loaded: %s\n', model_name);

% Set simulation parameters
set_param(model_name, 'StopTime', num2str(sim_time));
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'MaxStep', num2str(Ts/10));
fprintf('  StopTime: %.4f s, Solver: ode45, MaxStep: %.2e s\n', sim_time, Ts/10);

% Assign variables to base workspace
assignin('base', 'params', ctrl_params);
assignin('base', 'Kp_value', Kp_value);
assignin('base', 'Ki_value', Ki_value);
assignin('base', 'ControllerType', 1);  % 1=R-Controller

% External vd timeseries (not used for inner loop test)
vd_timeseries = timeseries(zeros(2, 6), [0; sim_time]);
assignin('base', 'vd_timeseries', vd_timeseries);

fprintf('  Parameters loaded to workspace\n\n');

%% Run Simulation

fprintf('[Simulation]\n');
fprintf('------------------------\n');
fprintf('  Sampling rate: %.0f kHz\n', 1/Ts/1000);
fprintf('  Running simulation...\n');

tic;
try
    out = sim(model_name);
    elapsed_time = toc;
    fprintf('  Completed in %.2f seconds\n\n', elapsed_time);
catch ME
    fprintf('  Simulation failed: %s\n', ME.message);
    rethrow(ME);
end

%% Extract Data

fprintf('[Data Extraction]\n');
fprintf('------------------------\n');

v_d = out.Vd;
v_m = out.Vm;
u = out.u;
u_w1 = out.u_w1;

N = size(v_d, 1);
t = (0:N-1)' * Ts;

fprintf('  Data points: %d (%.3f s)\n', N, t(end));
fprintf('  v_d: [%d x %d], v_m: [%d x %d]\n', ...
        size(v_d, 1), size(v_d, 2), size(v_m, 1), size(v_m, 2));
fprintf('  u: [%d x %d], u_w1: [%d x %d]\n\n', ...
        size(u, 1), size(u, 2), size(u_w1, 1), size(u_w1, 2));

%% Steady-State Analysis (Sine Mode)

if strcmpi(signal_type_name, 'sine')
    fprintf('[Steady-State Analysis]\n');
    fprintf('------------------------\n');

    % Select steady-state data for FFT
    period = 1 / frequency;
    t_start = config.skip_cycles * period;
    t_end = min(t_start + config.fft_cycles * period, t(end));
    idx_steady = (t >= t_start) & (t <= t_end);

    if sum(idx_steady) < 100
        fprintf('  Insufficient steady-state data points (%d)\n\n', sum(idx_steady));
        fft_results = [];
        qc_results = [];
    else
        v_d_steady = v_d(idx_steady, :);
        v_m_steady = v_m(idx_steady, :);
        actual_cycles = (t_end - t_start) / period;

        fprintf('  Steady-state window: %.2f ~ %.2f s (%.1f cycles, %d points)\n', ...
                t_start, t_end, actual_cycles, sum(idx_steady));

        % Quality check using utility function
        qc_results = quality_check(v_m_steady, config.fs, amplitude, ...
            'Frequency', frequency, ...
            'SteadyThreshold', config.steady_threshold, ...
            'THDThreshold', config.thd_threshold, ...
            'DCTolerance', config.dc_tolerance);

        % Display quality check results
        fprintf('\n  Quality Check Results:\n');
        fprintf('    Channel | Steady | THD          | DC Error | Status\n');
        fprintf('    --------+--------+--------------+----------+-------\n');

        for ch = 1:6
            steady_mark = conditional_mark(qc_results.steady_state_pass(ch));
            thd_mark = conditional_mark(qc_results.thd_pass(ch));
            dc_mark = conditional_mark(qc_results.dc_pass(ch));

            if ch == channel
                status = conditional_status(qc_results.overall_pass(ch), 'PASS', 'WARN');
            else
                status = conditional_status(qc_results.overall_pass(ch), 'OK', 'FAIL');
            end

            thd_str = format_thd(qc_results.thd_values(ch));

            fprintf('     P%d     |   %s    | %s %s | %.4fV %s | %s\n', ...
                    ch, steady_mark, thd_str, thd_mark, ...
                    qc_results.dc_error(ch), dc_mark, status);
        end
        fprintf('\n');

        % FFT analysis using utility function
        fprintf('[FFT Frequency Response]\n');
        fprintf('------------------------\n');

        fft_results = fft_analysis(v_d_steady(:, channel), v_m_steady, ...
                                   config.fs, frequency);

        fprintf('  Target: %.2f Hz, Actual bin: %.2f Hz (error: %.3f%%)\n', ...
                frequency, fft_results.actual_freq, fft_results.freq_error_pct);

        if fft_results.freq_error_pct > config.freq_error_threshold
            fprintf('  Warning: Frequency error %.3f%% > %.3f%%\n', ...
                    fft_results.freq_error_pct, config.freq_error_threshold);
        end

        fprintf('\n  Channel |  Magnitude  |   Phase    | Notes\n');
        fprintf('  --------+-------------+------------+------\n');

        for ch = 1:6
            notes = '';
            if ch == channel
                notes = '<- excited';
            end

            fprintf('    P%d    |  %6.2f%%   |  %+7.2f  | %s\n', ...
                    ch, fft_results.magnitude_ratio(ch)*100, ...
                    fft_results.phase_lag_deg(ch), notes);
        end
        fprintf('\n');
    end

    % Prepare display data (last N cycles)
    t_display_start = max(0, t(end) - config.display_cycles * period);
    idx_display = t >= t_display_start;
    t_display = t(idx_display);
    v_d_display = v_d(idx_display, :);
    v_m_display = v_m(idx_display, :);
end

%% Performance Metrics (Step Mode)

if strcmpi(signal_type_name, 'step')
    fprintf('[Performance Metrics]\n');
    fprintf('------------------------\n');
    fprintf('  Excited channel: P%d, Target: %.4f V\n\n', channel, amplitude);

    v_m_ch = v_m(:, channel);
    v_d_ch = v_d(:, channel);

    % Final value (average of last 100ms)
    settling_window = 0.1;
    n_window = round(settling_window / Ts);
    final_value = mean(v_m_ch(end-n_window:end));

    % Steady-state error
    v_e_steady = v_d_ch(end-n_window:end) - v_m_ch(end-n_window:end);
    sse = mean(abs(v_e_steady));
    sse_percent = (sse / abs(amplitude)) * 100;

    % Rise time (10% to 90%)
    level_10 = final_value * 0.1;
    level_90 = final_value * 0.9;
    idx_10 = find(v_m_ch >= level_10, 1, 'first');
    idx_90 = find(v_m_ch >= level_90, 1, 'first');

    if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
        rise_time = t(idx_90) - t(idx_10);
    else
        rise_time = NaN;
    end

    % Settling time (2% band)
    settling_band = 0.02;
    upper_bound = final_value * (1 + settling_band);
    lower_bound = final_value * (1 - settling_band);
    outside_band = (v_m_ch > upper_bound) | (v_m_ch < lower_bound);
    last_violation = find(outside_band, 1, 'last');

    if isempty(last_violation)
        settling_time = 0;
    else
        settling_time = t(last_violation);
    end

    % Overshoot
    idx_after_step = t >= step_time;
    v_m_after = v_m_ch(idx_after_step);
    [peak_value, ~] = max(v_m_after);

    if final_value ~= 0
        overshoot_pct = max(0, (peak_value - final_value) / abs(final_value) * 100);
    else
        overshoot_pct = 0;
    end

    % Display results
    fprintf('  Time-Domain Response:\n');
    fprintf('    Final value:          %.6f V\n', final_value);
    fprintf('    Rise time (10-90%%):    %.4f s (%.2f ms)\n', rise_time, rise_time*1000);
    fprintf('    Settling time (2%%):    %.4f s (%.2f ms)\n', settling_time, settling_time*1000);
    fprintf('    Overshoot:            %.2f%% (peak: %.6f V)\n', overshoot_pct, peak_value);
    fprintf('    Steady-state error:   %.6f V (%.4f%%)\n\n', sse, sse_percent);

    % Store performance metrics
    performance = struct();
    performance.channel = channel;
    performance.target_value = amplitude;
    performance.final_value = final_value;
    performance.rise_time = rise_time;
    performance.settling_time = settling_time;
    performance.overshoot_pct = overshoot_pct;
    performance.peak_value = peak_value;
    performance.sse = sse;
    performance.sse_percent = sse_percent;
end

%% Generate Plots

if ENABLE_PLOT
    fprintf('[Generating Plots]\n');
    fprintf('------------------------\n');

    % Create tab figure
    fig_main = uifigure('Name', sprintf('R-Controller Test: %s', test_name), ...
                        'Position', [50 50 1400 900]);
    tabgroup = uitabgroup(fig_main);
    tabgroup.Units = 'normalized';
    tabgroup.Position = [0 0 1 1];

    if strcmpi(signal_type_name, 'sine')
        % Sine mode tabs
        tab1 = create_vm_vd_plot(tabgroup, v_d_display, v_m_display, ...
                                 channel, amplitude, styles, fft_results);
        fprintf('  Tab 1: Vm vs Vd\n');

        tab2 = create_6ch_time_plot(tabgroup, t_display, v_d_display, v_m_display, ...
                                    channel, frequency, config.display_cycles, styles, ...
                                    '6ch Time Response');
        fprintf('  Tab 2: 6ch Time Response\n');

        % Detail window for u and u_w1
        detail_cycles = 10;
        t_start_detail = max(0, t(end) - detail_cycles * period);
        idx_detail = t >= t_start_detail;
        t_detail = t(idx_detail);
        u_detail = u(idx_detail, :);
        u_w1_detail = u_w1(idx_detail, :);

        tab3 = create_control_input_plot(tabgroup, t_detail, u_detail, ...
                                         fB_f, fB_c, fB_e, detail_cycles, styles, ...
                                         'Control Input (u)');
        fprintf('  Tab 3: Control Input (u)\n');

        tab4 = create_control_input_plot(tabgroup, t_detail, u_w1_detail, ...
                                         fB_f, fB_c, fB_e, detail_cycles, styles, ...
                                         'u_w1 Estimation');
        fprintf('  Tab 4: u_w1 Estimation\n');

        tab5 = create_6ch_time_plot(tabgroup, t, v_d, v_m, ...
                                    channel, frequency, -1, styles, ...
                                    'Vm Full Time');
        fprintf('  Tab 5: Vm Full Time\n');

        tab6 = create_control_input_plot(tabgroup, t, u, ...
                                         fB_f, fB_c, fB_e, -1, styles, ...
                                         'Control Effort Full');
        fprintf('  Tab 6: Control Effort Full\n');
    else
        % Step mode tabs
        zoom_time = 0.01;
        idx_zoom = t <= zoom_time;
        t_zoom = t(idx_zoom);
        v_m_zoom = v_m(idx_zoom, :);
        v_d_zoom = v_d(idx_zoom, :);

        tab1 = create_step_response_plot(tabgroup, t_zoom, v_d_zoom, v_m_zoom, ...
                                         channel, amplitude, styles);
        fprintf('  Tab 1: Step Response\n');

        tab2 = create_6ch_time_plot(tabgroup, t, v_d, v_m, ...
                                    channel, 0, -1, styles, ...
                                    '6ch Time Response');
        fprintf('  Tab 2: 6ch Time Response\n');

        % Steady state data for u and u_w1
        steady_time = 0.1;
        idx_steady_step = t >= (t(end) - steady_time);
        t_steady_step = t(idx_steady_step);
        u_steady = u(idx_steady_step, :);
        u_w1_steady = u_w1(idx_steady_step, :);

        tab3 = create_control_input_plot(tabgroup, t_steady_step - t_steady_step(1), ...
                                         u_steady, fB_f, fB_c, fB_e, -1, styles, ...
                                         'Control Input (u)');
        fprintf('  Tab 3: Control Input (u)\n');

        tab4 = create_control_input_plot(tabgroup, t_steady_step - t_steady_step(1), ...
                                         u_w1_steady, fB_f, fB_c, fB_e, -1, styles, ...
                                         'u_w1 Estimation');
        fprintf('  Tab 4: u_w1 Estimation\n');

        tab5 = create_6ch_time_plot(tabgroup, t, v_d, v_m, ...
                                    channel, 0, -1, styles, ...
                                    'Vm Full Time');
        fprintf('  Tab 5: Vm Full Time\n');

        tab6 = create_control_input_plot(tabgroup, t, u, ...
                                         fB_f, fB_c, fB_e, -1, styles, ...
                                         'Control Effort Full');
        fprintf('  Tab 6: Control Effort Full\n');
    end
    fprintf('\n');
end

%% Save Results

if SAVE_PNG || SAVE_MAT
    fprintf('[Saving Results]\n');
    fprintf('------------------------\n');

    if SAVE_PNG && ENABLE_PLOT
        export_res = styles.export_resolution;

        exportgraphics(tab1, fullfile(test_dir, 'tab1.png'), 'Resolution', export_res);
        exportgraphics(tab2, fullfile(test_dir, 'tab2.png'), 'Resolution', export_res);
        exportgraphics(tab3, fullfile(test_dir, 'tab3.png'), 'Resolution', export_res);
        exportgraphics(tab4, fullfile(test_dir, 'tab4.png'), 'Resolution', export_res);
        exportgraphics(tab5, fullfile(test_dir, 'tab5.png'), 'Resolution', export_res);
        exportgraphics(tab6, fullfile(test_dir, 'tab6.png'), 'Resolution', export_res);

        fprintf('  Figures saved (PNG, %d DPI)\n', export_res);
    end

    if SAVE_MAT
        result = build_result_struct(test_name, signal_type_name, channel, amplitude, ...
                                     d, fB_f, fB_c, fB_e, lambda_f, lambda_c, lambda_e, ...
                                     beta, sim_time, Ts, t, v_d, v_m, u, u_w1, elapsed_time);

        if strcmpi(signal_type_name, 'sine')
            result.config.frequency = frequency;
            result.config.phase = phase;
            result.display.t = t_display;
            result.display.v_d = v_d_display;
            result.display.v_m = v_m_display;

            if ~isempty(fft_results)
                result.analysis = fft_results;
            end
            if ~isempty(qc_results)
                result.quality = qc_results;
            end
        else
            result.config.step_time = step_time;
            result.performance = performance;
        end

        save(fullfile(test_dir, 'result.mat'), 'result', '-v7.3');
        fprintf('  Data saved (MAT)\n');
    end

    fprintf('  Output: %s\n\n', test_dir);
end

%% Summary

fprintf('================================================================\n');
fprintf('                     Test Completed\n');
fprintf('================================================================\n\n');

fprintf('[Summary]\n');
fprintf('  Name: %s\n', test_name);
fprintf('  Signal: %s, P%d, %.3f V\n', signal_type_name, channel, amplitude);

if strcmpi(signal_type_name, 'sine')
    fprintf('  Frequency: %.1f Hz\n', frequency);
end

fprintf('  R-Controller: d=%d, fB_c=%d Hz, fB_e=%d Hz\n', d, fB_c, fB_e);
fprintf('  Elapsed time: %.2f s\n\n', elapsed_time);


% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function mark = conditional_mark(condition)
% Return check/cross mark based on condition
    if condition
        mark = 'v';
    else
        mark = 'x';
    end
end

function status = conditional_status(condition, pass_str, fail_str)
% Return status string based on condition
    if condition
        status = pass_str;
    else
        status = fail_str;
    end
end

function str = format_thd(thd_val)
% Format THD value for display
    if isnan(thd_val)
        str = '    N/A    ';
    elseif thd_val < 0.01
        str = sprintf('%10.2e%%', thd_val);
    else
        str = sprintf('%10.4f%%', thd_val);
    end
end

function tab = create_vm_vd_plot(tabgroup, v_d, v_m, channel, amplitude, styles, fft_results)
% Create Vm vs Vd phase plot
    tab = uitab(tabgroup, 'Title', 'Vm vs Vd');
    ax = uiaxes(tab);
    ax.Position = [80 80 700 700];
    hold(ax, 'on');

    colors = styles.channel_colors;
    excited_lw = styles.measurement_linewidth * 1.5;
    base_lw = styles.measurement_linewidth * 0.5;
    plot_handles = gobjects(6, 1);

    % Plot non-excited channels first
    draw_count = 0;
    for ch = 1:6
        if ch ~= channel
            draw_count = draw_count + 1;
            lw = excited_lw - (excited_lw - base_lw) * (draw_count - 1) / 4;
            plot_handles(ch) = plot(ax, v_d(:, channel), v_m(:, ch), ...
                 'Color', colors(ch, :), 'LineWidth', lw);
        end
    end

    % Plot excited channel on top
    plot_handles(channel) = plot(ax, v_d(:, channel), v_m(:, channel), ...
         'Color', colors(channel, :), 'LineWidth', excited_lw);

    xlabel(ax, 'Vd (V)', 'FontSize', styles.xlabel_fontsize+6, 'FontWeight', 'bold');
    ylabel(ax, 'Vm (V)', 'FontSize', styles.ylabel_fontsize+6, 'FontWeight', 'bold');

    ax.LineWidth = 3.0;
    ax.FontSize = styles.tick_fontsize+6;
    ax.FontWeight = 'bold';
    ax.Box = 'on';

    % Unified axis limits
    max_val = max([max(abs(v_d(:))), max(abs(v_m(:)))]);
    axis_lim = [-max_val*1.1, max_val*1.1];
    xlim(ax, axis_lim);
    ylim(ax, axis_lim);
    axis(ax, 'square');

    tick_step = amplitude / 2;
    num_ticks = floor(max_val / tick_step);
    tick_values = (-num_ticks:num_ticks) * tick_step;
    tick_values = tick_values(tick_values >= axis_lim(1) & tick_values <= axis_lim(2));
    ax.XTick = tick_values;
    ax.YTick = tick_values;

    legend(ax, plot_handles, styles.channel_labels, ...
           'Location', 'southeast', 'FontSize', styles.legend_fontsize+2);

    % Title with FFT info
    if ~isempty(fft_results)
        title_str = sprintf('P%d Mag: %.2f%% Phase: %+.2f deg', ...
                           channel, fft_results.magnitude_ratio(channel)*100, ...
                           fft_results.phase_lag_deg(channel));
    else
        title_str = sprintf('P%d', channel);
    end
    title(ax, title_str, 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');
end

function tab = create_6ch_time_plot(tabgroup, t, v_d, v_m, channel, frequency, ...
                                    display_cycles, styles, tab_title)
% Create 6-channel time response plot
    tab = uitab(tabgroup, 'Title', tab_title);
    tl = tiledlayout(tab, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

    if frequency > 0 && display_cycles > 0
        main_title = sprintf('%s - P%d, %.1f Hz (Last %d cycles)', ...
                            tab_title, channel, frequency, display_cycles);
    elseif frequency > 0
        main_title = sprintf('%s - P%d, %.1f Hz', tab_title, channel, frequency);
    else
        main_title = sprintf('%s - P%d', tab_title, channel);
    end
    title(tl, main_title, 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    colors = styles.channel_colors;

    for ch = 1:6
        ax = nexttile(tl);
        plot(ax, t*1000, v_m(:, ch), '-', ...
             'Color', colors(ch, :), 'LineWidth', styles.measurement_linewidth);
        hold(ax, 'on');
        plot(ax, t*1000, v_d(:, ch), '--', ...
             'Color', styles.theory_color, 'LineWidth', styles.reference_linewidth);

        xlabel(ax, 'Time (ms)', 'FontSize', styles.xlabel_fontsize);
        ylabel(ax, 'Voltage (V)', 'FontSize', styles.ylabel_fontsize);
        title(ax, sprintf('P%d', ch), 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');
        ax.LineWidth = styles.axis_linewidth;
        ax.FontSize = styles.tick_fontsize;
        ax.Box = 'off';

        if ch == 1
            legend(ax, {'Vm', 'Vd'}, 'Location', 'northeast', ...
                   'FontSize', styles.legend_fontsize-2);
        end
    end
end

function tab = create_control_input_plot(tabgroup, t, u_data, fB_f, fB_c, fB_e, ...
                                         detail_cycles, styles, tab_title)
% Create control input plot (u or u_w1)
    tab = uitab(tabgroup, 'Title', tab_title);
    tl = tiledlayout(tab, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

    if detail_cycles > 0
        main_title = sprintf('%s - fB: f=%d, c=%d, e=%d Hz (Last %d cycles)', ...
                            tab_title, fB_f, fB_c, fB_e, detail_cycles);
    else
        main_title = sprintf('%s - fB: f=%d, c=%d, e=%d Hz', ...
                            tab_title, fB_f, fB_c, fB_e);
    end
    title(tl, main_title, 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    colors = styles.channel_colors;

    for ch = 1:6
        ax = nexttile(tl);
        plot(ax, t*1000, u_data(:, ch), '-', ...
             'Color', colors(ch, :), 'LineWidth', styles.measurement_linewidth);
        grid(ax, 'on');

        xlabel(ax, 'Time (ms)', 'FontSize', styles.xlabel_fontsize);
        ylabel(ax, 'u (V)', 'FontSize', styles.ylabel_fontsize);
        title(ax, sprintf('P%d', ch), 'FontSize', styles.title_fontsize-2, 'FontWeight', 'bold');
        ax.LineWidth = styles.axis_linewidth;
        ax.FontSize = styles.tick_fontsize;
    end
end

function tab = create_step_response_plot(tabgroup, t, v_d, v_m, channel, amplitude, styles)
% Create step response plot (0-10ms)
    tab = uitab(tabgroup, 'Title', 'Step Response');
    tl = tiledlayout(tab, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

    title(tl, sprintf('Step Response - P%d, Amp: %.2f V (0-10ms)', channel, amplitude), ...
          'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    colors = styles.channel_colors;

    for ch = 1:6
        ax = nexttile(tl);
        plot(ax, t*1000, v_m(:, ch), '-', ...
             'Color', colors(ch, :), 'LineWidth', styles.measurement_linewidth);
        hold(ax, 'on');
        plot(ax, t*1000, v_d(:, ch), '--', ...
             'Color', styles.reference_color, 'LineWidth', styles.reference_linewidth);

        xlabel(ax, 'Time (ms)', 'FontSize', styles.xlabel_fontsize);
        ylabel(ax, 'Voltage (V)', 'FontSize', styles.ylabel_fontsize);
        title(ax, sprintf('P%d', ch), 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');
        ax.LineWidth = styles.axis_linewidth;
        ax.FontSize = styles.tick_fontsize;
        ax.Box = 'off';

        if ch == 1
            legend(ax, {'Vm', 'Vd'}, 'Location', 'best', ...
                   'FontSize', styles.legend_fontsize-2);
        end
    end
end

function result = build_result_struct(test_name, signal_type_name, channel, amplitude, ...
                                      d, fB_f, fB_c, fB_e, lambda_f, lambda_c, lambda_e, ...
                                      beta, sim_time, Ts, t, v_d, v_m, u, u_w1, elapsed_time)
% Build result structure for saving
    result = struct();
    result.config.test_name = test_name;
    result.config.signal_type_name = signal_type_name;
    result.config.channel = channel;
    result.config.amplitude = amplitude;
    result.config.d = d;
    result.config.fB_f = fB_f;
    result.config.fB_c = fB_c;
    result.config.fB_e = fB_e;
    result.config.lambda_f = lambda_f;
    result.config.lambda_c = lambda_c;
    result.config.lambda_e = lambda_e;
    result.config.beta = beta;
    result.config.sim_time = sim_time;
    result.config.Ts = Ts;

    result.data.t = t;
    result.data.v_d = v_d;
    result.data.v_m = v_m;
    result.data.u = u;
    result.data.u_w1 = u_w1;

    result.meta.timestamp = datestr(now);
    result.meta.elapsed_time = elapsed_time;
end
