% run_force_generation_test.m
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
package_root = fullfile(script_dir, '..', '..');
addpath(fullfile(package_root, 'model'));
addpath(fullfile(package_root, 'model', 'inner_loop_ctrl'));
addpath(fullfile(package_root, 'model', 'flux_allocation'));


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
force_frequency = 50;           % Force frequency [Hz]
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
% 1.5.1 Signal Generation Mode
% ─────────────────────────────────────────────────────────────────────────
% Select signal generation mode:
%   'hardware' - Simulate hardware constraint (1600 Hz generation + interpolation)
%   'ideal'    - Direct 100 kHz generation (no interpolation delay)
generation_mode = 'ideal';   % 'hardware' or 'ideal'

% Hardware mode parameters (only used when generation_mode = 'hardware')
pos_update_rate = 1600;         % Position update frequency [Hz]
interp_method = 'linear';       % Interpolation method: 'linear' or 'previous' (ZOH)
USE_REALTIME_INTERP = true;     % true = Real-time (1-period delay), false = Ideal (non-causal)

% ─────────────────────────────────────────────────────────────────────────
% 1.6 R-Controller Parameters
% ─────────────────────────────────────────────────────────────────────────
fB_f = 3000;                    % Feedforward bandwidth [Hz]
fB_c = 3200;                    % Controller bandwidth [Hz]
fB_e = 16000;                    % Estimator bandwidth [Hz]

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
output_dir = fullfile(package_root, 'test_results', 'force_generation', 'force_control');

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

% Load system parameters with Simulink Bus support
% SampleRateMode: 1=ZOH, 2=Linear (default), 3=Direct
if strcmpi(generation_mode, 'hardware')
    if strcmpi(interp_method, 'previous')
        sample_rate_mode = 1;  % ZOH
    else
        sample_rate_mode = 2;  % Linear
    end
else
    sample_rate_mode = 3;  % Direct (100 kHz)
end

alloc_params_sim = force_model_allocation_params('Simulink', true, ...
    'pos_m', bead_position, 'SampleRateMode', sample_rate_mode);

% For offline analysis (not used by Simulink, but kept for compatibility)
inv_params = force_model_allocation_params();

% Load R-Controller parameters
model_base_ctrl_params_local = model_base_ctrl_params(fB_c, fB_e, fB_f);

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
fprintf('  Generation mode: %s (SampleRateMode=%d)\n', upper(generation_mode), sample_rate_mode);
if strcmpi(generation_mode, 'hardware')
    fprintf('    - Handled by inverse_model_function in Simulink\n');
end
if ControllerType == 1
    fprintf('  R-Controller: fB_f=%d, fB_c=%d, fB_e=%d Hz\n', fB_f, fB_c, fB_e);
else
    fprintf('  PI-Controller: Kp=%.1f, Ki=%.1f (zc=%d)\n', Kp_value, Ki_value, zc);
end
fprintf('\n');


%%                        SECTION 3: Generate f_d Timeseries


fprintf('【Generate f_d Timeseries】\n');
fprintf('────────────────────────\n');

% The new Simulink model handles inverse_model internally via inverse_model_function.
% Rate transition (ZOH/Linear/Direct) is controlled by alloc_params.sample_rate_mode.
% We only need to generate the f_d timeseries here.

% Define time axis at 100 kHz
N = round(sim_time / Ts) + 1;
t = (0:N-1)' * Ts;

fprintf('  Simulation rate: %.0f kHz (%d points)\n', 1/Ts/1000, N);
sample_rate_labels = {'ZOH (1600 Hz)', 'Linear (1600 Hz)', 'Direct (100 kHz)'};
fprintf('  SampleRateMode: %d (%s)\n', sample_rate_mode, sample_rate_labels{sample_rate_mode});

% Generate f_d at 100 kHz
f_d = zeros(N, 3);
if strcmpi(signal_type, 'sine')
    envelope = force_amplitude * sin(2*pi*force_frequency*t + deg2rad(force_phase));
    f_d = envelope .* force_direction';
    fprintf('  f_d: Sine wave, %.1f Hz, %.2f pN amplitude\n', force_frequency, force_amplitude);
else
    idx_step = t >= step_time;
    f_d(idx_step, :) = repmat(force_amplitude * force_direction', sum(idx_step), 1);
    fprintf('  f_d: Step at %.3f s, %.2f pN amplitude\n', step_time, force_amplitude);
end

fprintf('  f_d range: [%.4f, %.4f] pN\n', min(f_d(:)), max(f_d(:)));

% Create timeseries for Simulink From Workspace block
f_d_timeseries = timeseries(f_d, t);
f_d_timeseries.Name = 'f_d_external';

fprintf('\n');


%%                        SECTION 4: Simulink Simulation or Ideal Tracking


if USE_SIMULINK
    fprintf('【Simulink Simulation】\n');
    fprintf('────────────────────────\n');

    % ─────────────────────────────────────────────────────────────────────
    % New Vd Generator parameters (Bus-based architecture)
    % ─────────────────────────────────────────────────────────────────────
    % signal_type = 2 (Force mode): Simulink uses inverse_model_function
    % Rate transition (ZOH/Linear/Direct) is controlled by alloc_params.sample_rate_mode
    sim_signal_type = 2;  % Force mode
    assignin('base', 'signal_type', sim_signal_type);

    % f_d timeseries for Force mode
    assignin('base', 'f_d_timeseries', f_d_timeseries);

    % alloc_params (contains pos_m, sample_rate_mode, LUT, etc.)
    assignin('base', 'alloc_params', alloc_params_sim);

    % vd_signal_params (required by Signal mode, but not used in Force mode)
    % Create a dummy one to avoid Simulink errors
    vd_sig_params = vd_signal_params('Mode', 1, 'Channel', 1, 'Amplitude', 0, ...
        'Frequency', 100, 'Ts', Ts, 'd', 0);
    assignin('base', 'vd_signal_params', vd_sig_params);

    % ─────────────────────────────────────────────────────────────────────
    % Controller parameters
    % ─────────────────────────────────────────────────────────────────────
    assignin('base', 'ControllerType', ControllerType);

    % R-Controller parameters
    assignin('base', 'model_base_ctrl_params', model_base_ctrl_params_local);

    % PI-Controller parameters
    pi_ctrl_params_local = pi_ctrl_params(Kp_value, Ki_value, 'Ts', Ts);
    assignin('base', 'pi_ctrl_params', pi_ctrl_params_local);

    % Legacy parameters (for backward compatibility)
    assignin('base', 'Kp_value', Kp_value);
    assignin('base', 'Ki_value', Ki_value);

    % ─────────────────────────────────────────────────────────────────────
    % Load and run Simulink model
    % ─────────────────────────────────────────────────────────────────────
    model_name = 'main_system';
    model_path = fullfile(package_root, 'model', [model_name '.slx']);

    if ~bdIsLoaded(model_name)
        load_system(model_path);
    end

    % Set simulation parameters
    set_param(model_name, 'StopTime', num2str(sim_time));

    % Run simulation
    fprintf('  Running Simulink... ');
    tic;
    simOut = sim(model_name);
    fprintf('Done (%.2f sec)\n', toc);

    % ─────────────────────────────────────────────────────────────────────
    % Extract outputs from simulation
    % ─────────────────────────────────────────────────────────────────────
    % Vm: measured Hall voltage (6x1) from R-Controller output
    Vm_ts = simOut.get('Vm');
    if isa(Vm_ts, 'timeseries')
        vm_sim = Vm_ts.Data;
        t_vm = Vm_ts.Time;
    else
        vm_sim = Vm_ts;
        t_vm = (0:size(vm_sim,1)-1)' * Ts;
    end

    % Fm: estimated force (3x1) from Force_Model block
    Fm_ts = simOut.get('Fm');
    if isa(Fm_ts, 'timeseries')
        fm_sim = Fm_ts.Data;
    else
        fm_sim = Fm_ts;
    end

    % Resample to match our time axis if needed
    N_vm = size(vm_sim, 1);
    if N_vm ~= N
        fprintf('  Resampling vm (%d -> %d points)...\n', N_vm, N);
        vm = zeros(N, 6);
        for ch = 1:6
            vm(:, ch) = interp1(t_vm, vm_sim(:, ch), t, 'linear', 'extrap');
        end
        f_m = zeros(N, 3);
        for ax = 1:3
            f_m(:, ax) = interp1(t_vm, fm_sim(:, ax), t, 'linear', 'extrap');
        end
    else
        vm = vm_sim;
        f_m = fm_sim;
    end

    fprintf('  vm range: [%.4f, %.4f] V\n', min(vm(:)), max(vm(:)));
    fprintf('  f_m range: [%.4f, %.4f] pN\n', min(f_m(:)), max(f_m(:)));

    % ─────────────────────────────────────────────────────────────────────
    % Compute vd offline for plotting (optional, for Tab 2 display)
    % Note: This is a reference calculation, not the actual vd used in Simulink
    % ─────────────────────────────────────────────────────────────────────
    fprintf('  Computing offline vd for reference plots...');
    tic;
    vd = zeros(N, 6);
    for i = 1:N
        vd(i, :) = inverse_model(f_d(i, :)', bead_position, inv_params)';
    end
    fprintf(' Done (%.2f sec)\n', toc);

else
    fprintf('【Ideal Tracking Mode】\n');
    fprintf('────────────────────────\n');

    % Compute vd offline using inverse_model
    fprintf('  Computing vd from inverse_model...');
    tic;
    vd = zeros(N, 6);
    for i = 1:N
        vd(i, :) = inverse_model(f_d(i, :)', bead_position, inv_params)';
    end
    fprintf(' Done (%.2f sec)\n', toc);

    fprintf('  Assuming perfect tracking: vm = vd\n');
    vm = vd;

    % Compute f_m offline using force_model
    fprintf('  Computing f_m from force_model...');
    tic;
    f_m = zeros(N, 3);
    for i = 1:N
        f_m(i, :) = force_model(vm(i, :)', bead_position, inv_params)';
    end
    fprintf(' Done (%.2f sec)\n', toc);
end

fprintf('\n');


%%                        SECTION 5: Force Model Results
%
% Note: f_m is now computed in Section 4:
%   - USE_SIMULINK=true:  f_m comes directly from Force_Model block output (Fm)
%   - USE_SIMULINK=false: f_m computed offline using force_model()

fprintf('【Force Model Results】\n');
fprintf('────────────────────────\n');

if USE_SIMULINK
    fprintf('  f_m obtained from Simulink Force_Model block (force_model_function)\n');
else
    fprintf('  f_m computed offline using force_model()\n');
end
fprintf('  f_m range: [%.4f, %.4f] pN\n', min(f_m(:)), max(f_m(:)));


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
    % Tab 3: Error Analysis - 3 axes (error time series only)
    % ═══════════════════════════════════════════════════════════════════════
    tab3 = uitab(tabgroup, 'Title', 'Error Analysis');
    tab_handles.error_analysis = tab3;

    tl3 = tiledlayout(tab3, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl3, sprintf('Force Estimation Error Analysis (Total RMS: %.2f%%)', relative_error), ...
        'FontWeight', 'bold', 'FontSize', title_fontsize);

    % Error time series for each axis
    for ax_idx = 1:3
        ax = nexttile(tl3);
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
    fprintf('  Tab 3: Error Analysis (3-axis)\n');

    % ═══════════════════════════════════════════════════════════════════════
    % Tab 4-5: Simulink Controller Tabs (only when USE_SIMULINK = true)
    % ═══════════════════════════════════════════════════════════════════════
    if USE_SIMULINK
        % Calculate voltage tracking error
        voltage_error = vd - vm;
        voltage_error_rms = rms(voltage_error(idx_start:end, :));
        voltage_error_max = max(abs(voltage_error(idx_start:end, :)));

        % ───────────────────────────────────────────────────────────────────
        % Tab 4: Voltage Tracking (v_d vs v_m) + Error - 6 channels
        %        Left column: vd/vm overlay, Right column: tracking error
        % ───────────────────────────────────────────────────────────────────
        tab4 = uitab(tabgroup, 'Title', 'Voltage Tracking');
        tab_handles.voltage_tracking = tab4;

        tl4 = tiledlayout(tab4, 3, 4, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl4, 'Voltage Tracking: v_d vs v_m (left) | Error (right)', ...
            'FontWeight', 'bold', 'FontSize', title_fontsize);

        for ch = 1:6
            % Calculate row and column positions for left-right layout
            row = ceil(ch / 2);  % 1,1,2,2,3,3
            col_offset = mod(ch-1, 2) * 2;  % 0,2,0,2,0,2

            % Left subplot: vd vs vm overlay
            ax_left = nexttile(tl4, (row-1)*4 + col_offset + 1);
            plot(ax_left, t(idx_display:end)*1000, vd(idx_display:end, ch), 'b-', ...
                'LineWidth', measurement_linewidth);
            hold(ax_left, 'on');
            plot(ax_left, t(idx_display:end)*1000, vm(idx_display:end, ch), 'r--', ...
                'LineWidth', reference_linewidth);
            xlabel(ax_left, 'Time [ms]', 'FontSize', xlabel_fontsize-2);
            ylabel(ax_left, 'V [V]', 'FontSize', ylabel_fontsize-2);
            title(ax_left, sprintf('P%d: vd vs vm', ch), ...
                'FontSize', title_fontsize-3, 'FontWeight', 'bold');
            if ch == 1
                legend(ax_left, 'v_d', 'v_m', 'Location', 'best', 'FontSize', legend_fontsize-3);
            end
            ax_left.FontSize = tick_fontsize-1;
            ax_left.LineWidth = axis_linewidth;
            grid(ax_left, 'on');
            box(ax_left, 'on');

            % Right subplot: tracking error
            ax_right = nexttile(tl4, (row-1)*4 + col_offset + 2);
            plot(ax_right, t(idx_display:end)*1000, voltage_error(idx_display:end, ch)*1000, ...
                'Color', colors(ch,:), 'LineWidth', measurement_linewidth);
            hold(ax_right, 'on');
            yline(ax_right, 0, 'k--', 'LineWidth', 0.5);
            xlabel(ax_right, 'Time [ms]', 'FontSize', xlabel_fontsize-2);
            ylabel(ax_right, 'Error [mV]', 'FontSize', ylabel_fontsize-2);
            title(ax_right, sprintf('P%d Error (RMS: %.3f mV)', ch, voltage_error_rms(ch)*1000), ...
                'FontSize', title_fontsize-3, 'FontWeight', 'bold');
            ax_right.FontSize = tick_fontsize-1;
            ax_right.LineWidth = axis_linewidth;
            grid(ax_right, 'on');
            box(ax_right, 'on');
        end
        fprintf('  Tab 4: Voltage Tracking + Error (6 channels)\n');

        % ───────────────────────────────────────────────────────────────────
        % Tab 5: Control Input (u) - 6 channels
        % ───────────────────────────────────────────────────────────────────
        tab5 = uitab(tabgroup, 'Title', 'Control Input (u)');
        tab_handles.control_input = tab5;

        % Extract u from simulation output based on controller type
        if ControllerType == 1  % R-Controller
            u_ts = simOut.get('u');
        else  % PI-Controller
            u_ts = simOut.get('u_pi');
        end

        % Handle timeseries or array format
        if isa(u_ts, 'timeseries')
            u_data = u_ts.Data;
        else
            u_data = u_ts;
        end

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
        % Tab 6: Vm vs Vd Tracking (Lissajous-style plot)
        %        X: Vd, Y: Vm, 6 channels overlaid, 3 cycles
        %        Style: matching reference (analyze_invmodel_test.m Tab 8)
        % ───────────────────────────────────────────────────────────────────
        tab6 = uitab(tabgroup, 'Title', 'Vm vs Vd');
        tab_handles.vd_vs_vm = tab6;

        % Calculate 3-cycle window indices (from steady state)
        cycles_for_plot = 3;
        if strcmpi(signal_type, 'sine')
            T_period = 1 / force_frequency;
            % Start from skip_cycles, take 3 cycles
            idx_plot_start = round(skip_cycles * T_period / Ts) + 1;
            idx_plot_end = round((skip_cycles + cycles_for_plot) * T_period / Ts);
            idx_plot_end = min(idx_plot_end, N);  % Clamp to array size
        else
            % Step mode: use steady-state portion
            idx_plot_start = idx_display;
            idx_plot_end = N;
        end

        % Create single axes for overlaid plot
        ax6 = uiaxes(tab6);
        ax6.Position = [100 80 1200 750];
        hold(ax6, 'on');

        % Color scheme matching reference (6-channel colors)
        colors_6ch = [
            0.0000, 0.0000, 0.5000;  % P1: Dark blue
            0.0000, 0.0000, 1.0000;  % P2: Blue
            0.0000, 0.5000, 0.0000;  % P3: Green
            1.0000, 0.0000, 0.0000;  % P4: Red
            0.8000, 0.0000, 0.8000;  % P5: Purple
            0.0000, 0.7500, 0.7500;  % P6: Cyan
        ];

        % Plot 6 channels: X=Vd, Y=Vm (swapped from previous)
        channel_labels = {'Ch1', 'Ch2', 'Ch3', 'Ch4', 'Ch5', 'Ch6'};
        plot_handles = gobjects(1, 6);
        for ch = 1:6
            plot_handles(ch) = plot(ax6, ...
                vd(idx_plot_start:idx_plot_end, ch), ...
                vm(idx_plot_start:idx_plot_end, ch), ...
                'Color', colors_6ch(ch,:), 'LineWidth', measurement_linewidth);
        end

        % Add reference line (y = x) as black dashed
        all_voltage = [vd(idx_plot_start:idx_plot_end, :); vm(idx_plot_start:idx_plot_end, :)];
        lims_v = [min(all_voltage(:)), max(all_voltage(:))];
        plot(ax6, lims_v, lims_v, 'k--', 'LineWidth', 1.5);

        % Title with frequency
        title(ax6, sprintf('Vm vs Vd (Freq: %.1f Hz)', force_frequency), ...
            'FontSize', title_fontsize, 'FontWeight', 'bold');

        % Set axis labels (matching reference font sizes: +4)
        xlabel(ax6, 'Vd [V]', 'FontSize', xlabel_fontsize+4, 'FontWeight', 'bold');
        ylabel(ax6, 'Vm [V]', 'FontSize', ylabel_fontsize+4, 'FontWeight', 'bold');

        % Axis properties (matching reference)
        ax6.FontSize = tick_fontsize + 4;
        ax6.FontWeight = 'bold';
        ax6.LineWidth = axis_linewidth;
        ax6.Box = 'on';
        axis(ax6, 'equal');
        grid(ax6, 'on');

        % Legend at southeast (matching reference)
        legend(ax6, plot_handles, channel_labels, ...
            'Location', 'southeast', ...
            'FontSize', legend_fontsize + 2, 'FontWeight', 'bold');

        fprintf('  Tab 6: Vm vs Vd Tracking (%d cycles)\n', cycles_for_plot);

        % ───────────────────────────────────────────────────────────────────
        % Tab 7: fd vs fm Tracking (TEMPORARY)
        %        X: fd (excited direction only), Y: fm (all 3 directions)
        %        Style: matching reference image
        % ───────────────────────────────────────────────────────────────────
        tab7 = uitab(tabgroup, 'Title', 'fm vs fd (TEMP)');
        tab_handles.fd_vs_fm = tab7;

        % Find the excited direction index (the one with largest component in force_direction)
        [~, excited_axis] = max(abs(force_direction));
        force_labels_short = {'X', 'Y', 'Z'};
        force_colors = [0.0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880];

        % Use the same 3-cycle window as Tab 6 (100 kHz data)
        % idx_plot_start and idx_plot_end are already defined above

        % Extract data (100 kHz interpolated data)
        fd_excited = f_d(idx_plot_start:idx_plot_end, excited_axis);
        fm_all = f_m(idx_plot_start:idx_plot_end, :);  % All 3 directions

        % Create single axes for the plot
        ax7 = uiaxes(tab7);
        ax7.Position = [100 80 1200 750];
        hold(ax7, 'on');

        % Plot fm (all 3 directions) vs fd (excited) - X on fd axis, Y on fm axis
        plot_handles_7 = gobjects(1, 3);
        for i = 1:3
            plot_handles_7(i) = plot(ax7, fd_excited, fm_all(:, i), ...
                '-', 'Color', force_colors(i,:), 'LineWidth', 2.5);
        end

        % Add ideal reference line (y=x) as black dashed
        axis_limit = force_amplitude * 1.05;
        plot(ax7, [-axis_limit, axis_limit], [-axis_limit, axis_limit], ...
            'k--', 'LineWidth', 1.5);

        % Set axis limits
        xlim(ax7, [-axis_limit, axis_limit]);
        ylim(ax7, [-axis_limit, axis_limit]);

        % Title
        title(ax7, sprintf('fm vs fd (Freq: %.1f Hz)', force_frequency), ...
            'FontSize', 16, 'FontWeight', 'bold');

        % Set axis labels (bold, larger)
        xlabel(ax7, 'fd [pN]', 'FontSize', 18, 'FontWeight', 'bold');
        ylabel(ax7, 'fm [pN]', 'FontSize', 18, 'FontWeight', 'bold');

        % Axis properties
        ax7.FontSize = 14;
        ax7.FontWeight = 'bold';
        ax7.LineWidth = 2;  % Thicker frame
        ax7.XColor = 'k';
        ax7.YColor = 'k';
        ax7.GridColor = [0.8 0.8 0.8];
        ax7.GridAlpha = 1;
        grid(ax7, 'on');
        box(ax7, 'on');

        % Legend at bottom right
        legend(ax7, plot_handles_7, force_labels_short, ...
            'Location', 'southeast', ...
            'FontSize', 12, 'FontWeight', 'bold');

        fprintf('  Tab 7: fm vs fd Tracking (Freq: %.1f Hz) (TEMPORARY)\n', force_frequency);
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
                    % Tab 4: Voltage Tracking & Error
                    tabgroup.SelectedTab = tab4;
                    drawnow; pause(0.1);
                    exportgraphics(tab4, fullfile(test_dir, 'tab4_voltage_tracking.png'), 'Resolution', export_resolution);

                    % Tab 5: Control Input
                    tabgroup.SelectedTab = tab5;
                    drawnow; pause(0.1);
                    exportgraphics(tab5, fullfile(test_dir, 'tab5_control_input.png'), 'Resolution', export_resolution);

                    % Tab 6: vd vs vm Tracking
                    tabgroup.SelectedTab = tab6;
                    drawnow; pause(0.1);
                    exportgraphics(tab6, fullfile(test_dir, 'tab6_vd_vs_vm.png'), 'Resolution', export_resolution);

                    % Tab 7: fd vs fm Tracking (TEMPORARY)
                    tabgroup.SelectedTab = tab7;
                    drawnow; pause(0.1);
                    exportgraphics(tab7, fullfile(test_dir, 'tab7_fd_vs_fm_TEMP.png'), 'Resolution', export_resolution);

                    tab_count = 7;
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
        results.config.generation_mode = generation_mode;
        results.config.sample_rate_mode = sample_rate_mode;
        if strcmpi(generation_mode, 'hardware')
            results.config.pos_update_rate = pos_update_rate;
            results.config.interp_method = interp_method;
        end

        results.data.t = t;
        results.data.f_d = f_d;
        results.data.vd = vd;  % Offline reference (not actual Simulink vd)
        results.data.vm = vm;
        results.data.f_m = f_m;
        results.data.force_error = force_error;

        results.analysis.force_error_rms = force_error_rms;
        results.analysis.force_error_max = force_error_max;
        if exist('relative_error', 'var')
            results.analysis.relative_error = relative_error;
        end

        % Additional Simulink data
        if USE_SIMULINK
            results.data.u = u_data;
            results.analysis.voltage_error_rms = voltage_error_rms;
            results.analysis.voltage_error_max = voltage_error_max;
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
