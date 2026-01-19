% run_motion_control_test.m
% Motion Control comprehensive integration test
%
% Tests the complete Motion Control pipeline including:
%   - Trajectory Generator (z_sine, xy_circle, positioning)
%   - Motion Control Law (position tracking)
%   - Particle Dynamics (Wall Effect, Thermal Force)
%   - Force Allocation (inverse_model, force_model)
%   - Inner Loop Control (R-Controller)
%
% Supports multiple trajectory modes and configurable physical effects.
%
% See also: run_motion_control_simple_test, motion_control_law_function, CLAUDE.md

%% SECTION 1: Configuration
% =========================================================================
clear; clc; close all;

% === Test Name ===
test_name = 'motion_test';

% === Trajectory Mode ===
% Options: 'z_sine', 'xy_circle', 'positioning'
traj_type = 'z_sine';
h_init = 5;                     % Initial wall distance [um]
amplitude = 2.5;                % z_sine amplitude [um] (safe: h_init - amplitude > h_min)
frequency = 10;                  % z_sine frequency [Hz]
n_cycles = 3;                   % Number of cycles
radius = 5;                     % xy_circle radius [um]
period = 1;                     % xy_circle period [s]
hold_time = 1;                  % positioning hold time [s]

% === Wall Parameters ===
theta = 0;                      % Azimuth angle [rad]
phi = 0;                        % Polar angle [rad]
pz = 0;                         % Wall displacement [um]
h_min = 1.1 * 2.25;                  % Minimum safe distance [um]

% === Motion Control Parameters ===
ctrl_enable = true;             % true=closed-loop, false=open-loop
lambda_c = 0.7;                 % Closed-loop pole (0 < lambda < 1)
noise_filter_enable = false;    % Feedback low-pass filter
noise_filter_cutoff = 10;       % Filter cutoff frequency [Hz]

% === Physical Effects ===
thermal_enable = true;          % Thermal force (Brownian motion)
wall_effect_enable = true;      % Wall effect

% === Sample Rate and Interpolation ===
% 1=ZOH, 2=Linear (default), 3=Direct
sample_rate_mode = 2;
pos_update_rate = 1600;         % Position update rate [Hz]

% === Inner Loop Controller ===
% Options: 'model_base_ctrl' or 'pi_ctrl'
controller_type = 'pi_ctrl';
fB_f = 3000;                    % Feedforward bandwidth [Hz]
fB_c = 3200;                    % Controller bandwidth [Hz]
fB_e = 16000;                   % Estimator bandwidth [Hz]
Kp_value = 2;                   % PI proportional gain
Ki_value = 4412;                % PI integral gain

% === Analysis Parameters ===
cutoff_freq = 3;               % Deterministic/random separation cutoff [Hz]

% === Simulation Parameters ===
T_margin = 1/frequency;                 % Post-trajectory buffer time [s]

% === Output Control ===
ENABLE_PLOT = true;
SAVE_PNG = true;
SAVE_MAT = true;

%% SECTION 2: Setup Paths
% =========================================================================
script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
addpath(fullfile(project_root, 'model', 'motion_ctrl'));
addpath(fullfile(project_root, 'model', 'particle_dynamics'));
addpath(fullfile(project_root, 'model', 'flux_allocation'));
addpath(fullfile(project_root, 'model', 'inner_loop_ctrl'));
addpath(fullfile(project_root, 'test_script', 'utils'));

fprintf('\n');
fprintf('================================================================\n');
fprintf('       Motion Control Comprehensive Integration Test\n');
fprintf('================================================================\n\n');

% Load plot styles
styles = plot_styles();

%% SECTION 3: Calculate Simulation Time
% =========================================================================
fprintf('[Configuration]\n');
fprintf('------------------------\n');
fprintf('  Trajectory type: %s\n', traj_type);

switch lower(traj_type)
    case 'z_sine'
        T_traj = n_cycles / frequency;
        fprintf('  Amplitude: %.2f um, Frequency: %.1f Hz, Cycles: %d\n', ...
            amplitude, frequency, n_cycles);
    case 'xy_circle'
        T_traj = period * n_cycles;
        fprintf('  Radius: %.2f um, Period: %.1f s, Cycles: %d\n', ...
            radius, period, n_cycles);
    case 'positioning'
        T_traj = hold_time;
        fprintf('  Hold time: %.1f s\n', hold_time);
    otherwise
        error('Unknown trajectory type: %s', traj_type);
end

T_sim = T_traj + T_margin;
fprintf('  Trajectory time: %.2f s, Total sim time: %.2f s\n', T_traj, T_sim);
fprintf('  Control: %s (lambda_c=%.2f)\n', ...
    conditional_str(ctrl_enable, 'Closed-loop', 'Open-loop'), lambda_c);
fprintf('  Thermal: %s, Wall effect: %s\n', ...
    conditional_str(thermal_enable, 'ON', 'OFF'), ...
    conditional_str(wall_effect_enable, 'ON', 'OFF'));
fprintf('  Inner loop: %s\n\n', controller_type);

%% SECTION 4: Create Parameter Structures
% =========================================================================
fprintf('[Creating Parameters]\n');
fprintf('------------------------\n');

% Trajectory parameters
traj_type_num = get_traj_type_num(traj_type);
traj_params = trajectory_generator_params( ...
    'Type', traj_type_num, ...
    'Amplitude', amplitude, ...
    'Frequency', frequency, ...
    'Radius', radius, ...
    'Period', period, ...
    'NCycles', n_cycles, ...
    'HoldTime', hold_time);
fprintf('  trajectory_generator_params created\n');

% Particle dynamics parameters
particle_params = particle_dynamics_params( ...
    'Theta', theta, ...
    'Phi', phi, ...
    'Pz', pz, ...
    'HMin', h_min, ...
    'HInit', h_init, ...
    'WallEffectEnable', wall_effect_enable);
fprintf('  particle_dynamics_params created\n');

% Motion control parameters
motion_params = motion_control_law_params( ...
    'LambdaC', lambda_c, ...
    'Enable', ctrl_enable);
fprintf('  motion_control_law_params created\n');

% Thermal force parameters
thermal_params = thermal_force_params( ...
    'Enable', thermal_enable, ...
    'Seed', 42);
fprintf('  thermal_force_params created\n');

% Inner loop controller parameters
if strcmpi(controller_type, 'model_base_ctrl')
    ctrl_params = model_base_ctrl_params(fB_c, fB_e, fB_f);
    ControllerType = 1;
    fprintf('  model_base_ctrl_params created (fB: c=%d, e=%d, f=%d Hz)\n', fB_c, fB_e, fB_f);
else
    ctrl_params = pi_ctrl_params(Kp_value, Ki_value);
    ControllerType = 2;
    fprintf('  pi_ctrl_params created (Kp=%.2f, Ki=%.2f)\n', Kp_value, Ki_value);
end

% Force model allocation parameters
alloc_params = force_model_allocation_params( ...
    'Simulink', true, ...
    'SampleRateMode', sample_rate_mode, ...
    'PosMSource', 1);
fprintf('  force_model_allocation_params created\n');

% Calculate initial position
p0 = calc_initial_position_function(particle_params.Value);
fprintf('  Initial position: [%.2f, %.2f, %.2f] um\n\n', p0(1), p0(2), p0(3));

%% SECTION 5: Safety Check
% =========================================================================
fprintf('[Safety Check]\n');
fprintf('------------------------\n');

[is_safe, h_min_actual, t_critical] = check_trajectory_safety_function( ...
    p0, traj_params.Value, particle_params.Value, T_sim);

fprintf('  Minimum wall distance: %.3f um at t=%.3f s\n', h_min_actual, t_critical);
fprintf('  Safety threshold: %.3f um\n', h_min);

if is_safe
    fprintf('  Status: SAFE\n\n');
else
    fprintf('  Status: UNSAFE - trajectory may collide with wall!\n');
    fprintf('  Consider reducing amplitude or increasing h_init.\n\n');
    % Continue anyway but warn
end

%% SECTION 6: Prepare Simulink Workspace
% =========================================================================
fprintf('[Simulink Configuration]\n');
fprintf('------------------------\n');

model_name = 'main_system';
model_path = fullfile(project_root, 'model', [model_name '.slx']);

if ~exist(model_path, 'file')
    error('Model file not found: %s', model_path);
end

Ts = 1e-5;  % Inner loop sampling time
signal_type = 3;  % Motion Control mode

% Create timeseries for From Workspace blocks
t_vec = (0:Ts:T_sim)';
f_d_timeseries = timeseries(zeros(length(t_vec), 3), t_vec);
pos_m_static = timeseries(zeros(length(t_vec), 3) + p0', t_vec);

% Create vd_signal_params (required but not used in Motion Control mode)
vd_sig_params = vd_signal_params( ...
    'Mode', 1, ...
    'Channel', 1, ...
    'Amplitude', 0, ...
    'Frequency', 100, ...
    'Ts', Ts, ...
    'd', 0);

% Create pi_ctrl_params (always needed by model even if not used)
if strcmpi(controller_type, 'model_base_ctrl')
    pi_params = pi_ctrl_params(Kp_value, Ki_value);
else
    pi_params = ctrl_params;
    ctrl_params = model_base_ctrl_params(fB_c, fB_e, fB_f);
end

% Assign all to base workspace
assignin('base', 'trajectory_generator_params', traj_params);
assignin('base', 'particle_dynamics_params', particle_params);
assignin('base', 'motion_control_law_params', motion_params);
assignin('base', 'thermal_force_params', thermal_params);
assignin('base', 'model_base_ctrl_params', ctrl_params);
assignin('base', 'pi_ctrl_params', pi_params);
assignin('base', 'vd_signal_params', vd_sig_params);
assignin('base', 'alloc_params', alloc_params);
assignin('base', 'p0', p0);
assignin('base', 'signal_type', signal_type);
assignin('base', 'ControllerType', ControllerType);
assignin('base', 'f_d_timeseries', f_d_timeseries);
assignin('base', 'pos_m_static', pos_m_static);

fprintf('  Parameters assigned to base workspace\n');
fprintf('  signal_type = %d (Motion Control mode)\n\n', signal_type);

%% SECTION 7: Run Simulation
% =========================================================================
fprintf('[Simulation]\n');
fprintf('------------------------\n');
fprintf('  Loading model: %s\n', model_name);

if ~bdIsLoaded(model_name)
    load_system(model_path);
end

set_param(model_name, 'StopTime', num2str(T_sim));
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'MaxStep', num2str(Ts/10));

fprintf('  StopTime: %.2f s\n', T_sim);
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

%% SECTION 8: Extract Data
% =========================================================================
fprintf('[Data Extraction]\n');
fprintf('------------------------\n');

% Position data (from Motion Control)
p_d_out = out.p_d;
p_m_out = out.p_m;
f_d_out = out.f_m;

% Get thermal force if available
if isfield(out, 'F_th')
    F_th_out = out.F_th;
else
    F_th_out = zeros(size(f_d_out));
end

% Inner loop data (100 kHz rate)
v_d_out = out.Vd;
v_m_out = out.Vm;

% Control voltage output depends on selected controller
% ControllerType=1: model_base_ctrl (out.u), ControllerType=2: pi_ctrl (out.u_pi)
if ControllerType == 1
    u_out = out.u;      % Model-based controller output
else
    u_out = out.u_pi;   % PI controller output
end

% Print data info
fprintf('  Controller: %s\n', conditional_str(ControllerType == 1, 'model_base_ctrl', 'pi_ctrl'));

% Create time vectors
% Note: All Simulink To Workspace outputs are at 100 kHz (Ts = 1e-5)
% Motion Control internally updates at 1600 Hz, but logged at 100 kHz (shows ZOH steps)
N_motion = size(p_d_out, 1);
t_motion = (0:N_motion-1)' * Ts;  % 100 kHz time vector

N_inner = size(v_d_out, 1);
t_inner = (0:N_inner-1)' * Ts;    % 100 kHz time vector

fprintf('  Motion Control data: %d samples (%.2f s) @ 100 kHz\n', N_motion, t_motion(end));
fprintf('  Inner Loop data: %d samples (%.2f s) @ 100 kHz\n', N_inner, t_inner(end));
fprintf('  (Motion Control updates at %d Hz internally)\n', pos_update_rate);

% -------------------------------------------------------------------------
% Post-processing: Apply interpolation to p_d, f_d based on sample_rate_mode
% This makes the analysis consistent with vd interpolation behavior
% -------------------------------------------------------------------------
if sample_rate_mode == 2  % Linear interpolation mode
    fprintf('  Applying linear interpolation to p_d, f_d (post-processing)...\n');

    % Calculate 1600 Hz sample indices
    Ts_motion = 1/pos_update_rate;
    downsample_factor = round(Ts_motion / Ts);  % 100kHz/1600Hz ≈ 62-63
    idx_1600 = 1:downsample_factor:N_motion;

    % Ensure last point is included
    if idx_1600(end) ~= N_motion
        idx_1600 = [idx_1600, N_motion];
    end

    t_1600 = t_motion(idx_1600);

    % Linear interpolation for p_d
    p_d_1600 = p_d_out(idx_1600, :);
    for i = 1:3
        p_d_out(:, i) = interp1(t_1600, p_d_1600(:, i), t_motion, 'linear', 'extrap');
    end

    % Linear interpolation for f_d
    f_d_1600 = f_d_out(idx_1600, :);
    for i = 1:3
        f_d_out(:, i) = interp1(t_1600, f_d_1600(:, i), t_motion, 'linear', 'extrap');
    end

    fprintf('  Done (1600 Hz → 100 kHz linear interpolation)\n');

elseif sample_rate_mode == 1  % ZOH mode
    fprintf('  ZOH mode: p_d, f_d remain as 1600 Hz step data\n');

else  % Direct mode (mode=3)
    fprintf('  Direct mode: p_d, f_d at native rate (no interpolation needed)\n');
end

%% SECTION 9: Analysis
% =========================================================================
fprintf('\n[Analysis]\n');
fprintf('------------------------\n');

% Position tracking error
p_error = p_m_out - p_d_out;
rms_error_um = sqrt(mean(p_error.^2, 1));
rms_error_nm = rms_error_um * 1000;
max_error_um = max(abs(p_error), [], 1);
max_error_nm = max_error_um * 1000;

fprintf('  Position RMS Error:\n');
fprintf('    X: %.2f nm, Y: %.2f nm, Z: %.2f nm\n', rms_error_nm(1), rms_error_nm(2), rms_error_nm(3));
fprintf('  Position Max Error:\n');
fprintf('    X: %.2f nm, Y: %.2f nm, Z: %.2f nm\n', max_error_nm(1), max_error_nm(2), max_error_nm(3));

% Control force statistics
max_force = max(abs(f_d_out), [], 1);
fprintf('  Max Control Force:\n');
fprintf('    Fx: %.2f pN, Fy: %.2f pN, Fz: %.2f pN\n', max_force(1), max_force(2), max_force(3));

% Wall distance monitoring
w_hat = particle_params.Value.w_hat;
h_actual = p_m_out * w_hat - pz;
h_R = h_actual / particle_params.Value.R;
h_min_sim = min(h_actual);
fprintf('  Wall distance: min=%.3f um (h/R=%.2f)\n', h_min_sim, h_min_sim/particle_params.Value.R);

% Voltage tracking quality (if available)
v_error = v_m_out - v_d_out;
v_rms_error = sqrt(mean(v_error.^2, 1));
fprintf('  Voltage RMS Error: [');
fprintf('%.4f ', v_rms_error);
fprintf('] V\n\n');

%% SECTION 10: Create Output Directory
% =========================================================================
if SAVE_PNG || SAVE_MAT
    output_dir = fullfile(project_root, 'test_results', 'motion_control', 'full_test');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    test_dir = fullfile(output_dir, sprintf('%s_%s_%s', test_name, traj_type, timestamp));
    mkdir(test_dir);
    fprintf('[Output]\n');
    fprintf('------------------------\n');
    fprintf('  Directory: %s\n\n', test_dir);
end

%% SECTION 11: Generate Plots
% =========================================================================
if ENABLE_PLOT
    fprintf('[Generating Plots]\n');
    fprintf('------------------------\n');

    % Create tab figure
    fig_main = uifigure('Name', sprintf('Motion Control Test: %s - %s', test_name, traj_type), ...
                        'Position', [50 50 1400 900]);
    tabgroup = uitabgroup(fig_main);
    tabgroup.Units = 'normalized';
    tabgroup.Position = [0 0 1 1];

    % Tab 1: 3D Trajectory (pass particle_params for wall vectors)
    tab1 = create_3d_trajectory_tab(tabgroup, p_d_out, p_m_out, particle_params.Value, styles);
    fprintf('  Tab 1: 3D Trajectory\n');

    % Tab 2-4: XYZ Axis Analysis
    tab2 = create_axis_analysis_tab(tabgroup, t_motion, p_d_out, p_m_out, f_d_out, 1, 'X', styles);
    fprintf('  Tab 2: X-axis Analysis\n');

    tab3 = create_axis_analysis_tab(tabgroup, t_motion, p_d_out, p_m_out, f_d_out, 2, 'Y', styles);
    fprintf('  Tab 3: Y-axis Analysis\n');

    tab4 = create_axis_analysis_tab(tabgroup, t_motion, p_d_out, p_m_out, f_d_out, 3, 'Z', styles);
    fprintf('  Tab 4: Z-axis Analysis\n');

    % Tab 5: Position Overview
    tab5 = create_position_overview_tab(tabgroup, t_motion, p_d_out, p_m_out, styles);
    fprintf('  Tab 5: Position Overview\n');

    % Tab 6: Force Overview
    tab6 = create_force_overview_tab(tabgroup, t_motion, f_d_out, styles);
    fprintf('  Tab 6: Force Overview\n');

    % Tab 7: Voltage Tracking (6 channels)
    tab7 = create_voltage_tracking_tab(tabgroup, t_inner, v_d_out, v_m_out, styles);
    fprintf('  Tab 7: Voltage Tracking\n');

    % Tab 8: Control Output (6 channels)
    tab8 = create_control_output_tab(tabgroup, t_inner, u_out, styles);
    fprintf('  Tab 8: Control Output\n');

    % Tab 9: FFT Spectrum
    tab9 = create_fft_spectrum_tab(tabgroup, t_motion, p_m_out, pos_update_rate, cutoff_freq, styles);
    fprintf('  Tab 9: FFT Spectrum\n');

    % Tab 10: Deterministic/Random Separation (on p_m)
    tab10 = create_separation_tab(tabgroup, t_motion, p_m_out, pos_update_rate, cutoff_freq, styles);
    fprintf('  Tab 10: Deterministic/Random Separation\n');

    % Tab 11: Wall Distance Monitoring
    tab11 = create_wall_distance_tab(tabgroup, t_motion, h_actual, h_R, h_min, particle_params.Value.R, styles);
    fprintf('  Tab 11: Wall Distance Monitoring\n');

    fprintf('\n');
end

%% SECTION 12: Save Results
% =========================================================================
if SAVE_PNG || SAVE_MAT
    fprintf('[Saving Results]\n');
    fprintf('------------------------\n');

    if SAVE_PNG && ENABLE_PLOT
        export_res = styles.export_resolution;

        exportgraphics(tab1, fullfile(test_dir, 'tab01_3d_trajectory.png'), 'Resolution', export_res);
        exportgraphics(tab2, fullfile(test_dir, 'tab02_x_axis.png'), 'Resolution', export_res);
        exportgraphics(tab3, fullfile(test_dir, 'tab03_y_axis.png'), 'Resolution', export_res);
        exportgraphics(tab4, fullfile(test_dir, 'tab04_z_axis.png'), 'Resolution', export_res);
        exportgraphics(tab5, fullfile(test_dir, 'tab05_position.png'), 'Resolution', export_res);
        exportgraphics(tab6, fullfile(test_dir, 'tab06_force.png'), 'Resolution', export_res);
        exportgraphics(tab7, fullfile(test_dir, 'tab07_voltage.png'), 'Resolution', export_res);
        exportgraphics(tab8, fullfile(test_dir, 'tab08_current.png'), 'Resolution', export_res);
        exportgraphics(tab9, fullfile(test_dir, 'tab09_fft.png'), 'Resolution', export_res);
        exportgraphics(tab10, fullfile(test_dir, 'tab10_separation.png'), 'Resolution', export_res);
        exportgraphics(tab11, fullfile(test_dir, 'tab11_wall_distance.png'), 'Resolution', export_res);

        fprintf('  Figures saved (PNG, %d DPI)\n', export_res);
    end

    if SAVE_MAT
        result = struct();

        % Configuration
        result.config.test_name = test_name;
        result.config.traj_type = traj_type;
        result.config.h_init = h_init;
        result.config.amplitude = amplitude;
        result.config.frequency = frequency;
        result.config.n_cycles = n_cycles;
        result.config.radius = radius;
        result.config.period = period;
        result.config.hold_time = hold_time;
        result.config.lambda_c = lambda_c;
        result.config.ctrl_enable = ctrl_enable;
        result.config.thermal_enable = thermal_enable;
        result.config.wall_effect_enable = wall_effect_enable;
        result.config.controller_type = controller_type;
        result.config.T_sim = T_sim;

        % Motion Control data
        result.motion.t = t_motion;
        result.motion.p_d = p_d_out;
        result.motion.p_m = p_m_out;
        result.motion.f_d = f_d_out;
        result.motion.F_th = F_th_out;

        % Inner Loop data
        result.inner.t = t_inner;
        result.inner.v_d = v_d_out;
        result.inner.v_m = v_m_out;
        result.inner.u = u_out;

        % Analysis results
        result.analysis.rms_error_nm = rms_error_nm;
        result.analysis.max_error_nm = max_error_nm;
        result.analysis.max_force = max_force;
        result.analysis.h_min_sim = h_min_sim;
        result.analysis.v_rms_error = v_rms_error;

        % Metadata
        result.meta.timestamp = datestr(now);
        result.meta.elapsed_time = elapsed_time;

        save(fullfile(test_dir, 'result.mat'), 'result', '-v7.3');
        fprintf('  Data saved (MAT)\n');
    end

    fprintf('  Output: %s\n\n', test_dir);
end

%% SECTION 13: Summary
% =========================================================================
fprintf('================================================================\n');
fprintf('                     Test Completed\n');
fprintf('================================================================\n\n');

fprintf('[Summary]\n');
fprintf('  Test: %s (%s)\n', test_name, traj_type);
fprintf('  Position RMS Error: X=%.1f nm, Y=%.1f nm, Z=%.1f nm\n', ...
    rms_error_nm(1), rms_error_nm(2), rms_error_nm(3));
fprintf('  Max Force: Fx=%.1f pN, Fy=%.1f pN, Fz=%.1f pN\n', ...
    max_force(1), max_force(2), max_force(3));
fprintf('  Min Wall Distance: %.3f um (h/R=%.2f)\n', h_min_sim, h_min_sim/particle_params.Value.R);
fprintf('  Elapsed time: %.2f s\n\n', elapsed_time);

% Pass/Fail criteria
pass_criteria = true;
if strcmpi(traj_type, 'positioning')
    if rms_error_nm(3) > 50
        fprintf('  [WARN] Z-axis RMS error > 50 nm for positioning mode\n');
        pass_criteria = false;
    end
else
    if max(rms_error_nm) > 100
        fprintf('  [WARN] RMS error > 100 nm\n');
        pass_criteria = false;
    end
end

if h_min_sim < h_min
    fprintf('  [WARN] Wall distance below safety threshold\n');
    pass_criteria = false;
end

if pass_criteria
    fprintf('  [PASS] All criteria met\n\n');
else
    fprintf('  [FAIL] Some criteria not met\n\n');
end


% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function str = conditional_str(cond, true_str, false_str)
    if cond
        str = true_str;
    else
        str = false_str;
    end
end

function type_num = get_traj_type_num(traj_type)
    switch lower(traj_type)
        case 'z_sine'
            type_num = 0;
        case 'xy_circle'
            type_num = 1;
        case 'positioning'
            type_num = 2;
        otherwise
            error('Unknown trajectory type: %s', traj_type);
    end
end

function tab = create_3d_trajectory_tab(tabgroup, p_d, p_m, particle_params, styles)
    tab = uitab(tabgroup, 'Title', '3D Trajectory');
    ax = uiaxes(tab);
    ax.Position = [80 80 800 700];
    hold(ax, 'on');

    % Plot desired trajectory
    plot3(ax, p_d(:,1), p_d(:,2), p_d(:,3), 'b-', ...
        'LineWidth', styles.reference_linewidth, 'DisplayName', 'Desired');

    % Plot actual trajectory
    plot3(ax, p_m(:,1), p_m(:,2), p_m(:,3), 'r--', ...
        'LineWidth', styles.measurement_linewidth, 'DisplayName', 'Actual');

    % Mark start point
    plot3(ax, p_d(1,1), p_d(1,2), p_d(1,3), 'go', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');

    % Extract wall parameters
    w_hat = particle_params.w_hat;
    u_hat = particle_params.u_hat;
    v_hat = particle_params.v_hat;
    pz = particle_params.pz;

    % Calculate trajectory extent for wall size
    all_pts = [p_d; p_m];
    center = mean(all_pts, 1)';
    extent = max(abs(all_pts - center'), [], 1);
    wall_size = max(extent) * 1.5;  % 1.5x the trajectory extent
    wall_size = max(wall_size, 3);  % Minimum 3 um

    % Find wall center point (project trajectory center onto wall plane)
    % Wall equation: dot(p - pz*w_hat, w_hat) = 0
    dist_to_wall = dot(center, w_hat) - pz;
    wall_center = center - dist_to_wall * w_hat;

    % Create grid in wall coordinate system (u_hat, v_hat plane)
    [U, V] = meshgrid(linspace(-wall_size, wall_size, 15), ...
                     linspace(-wall_size, wall_size, 15));

    % Transform to 3D coordinates
    X = wall_center(1) + U * u_hat(1) + V * v_hat(1);
    Y = wall_center(2) + U * u_hat(2) + V * v_hat(2);
    Z = wall_center(3) + U * u_hat(3) + V * v_hat(3);

    % Draw wall surface
    surf(ax, X, Y, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
        'FaceColor', [0.5, 0.5, 0.5], 'DisplayName', 'Wall');

    xlabel(ax, 'X [um]', 'FontSize', styles.xlabel_fontsize, 'FontWeight', 'bold');
    ylabel(ax, 'Y [um]', 'FontSize', styles.ylabel_fontsize, 'FontWeight', 'bold');
    zlabel(ax, 'Z [um]', 'FontSize', styles.ylabel_fontsize, 'FontWeight', 'bold');
    title(ax, '3D Trajectory', 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');
    legend(ax, 'Location', 'best', 'FontSize', styles.legend_fontsize);
    grid(ax, 'on');
    view(ax, 45, 30);
    axis(ax, 'equal');  % Equal aspect ratio for better 3D visualization
    ax.LineWidth = styles.axis_linewidth;
end

function tab = create_axis_analysis_tab(tabgroup, t, p_d, p_m, f_d, axis_idx, axis_name, styles)
    tab = uitab(tabgroup, 'Title', sprintf('%s-axis', axis_name));
    tl = tiledlayout(tab, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, sprintf('%s-axis Analysis', axis_name), ...
        'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    colors = styles.force_colors;

    % Subplot 1: Position tracking
    ax1 = nexttile(tl);
    plot(ax1, t*1000, p_d(:,axis_idx), 'b-', 'LineWidth', styles.reference_linewidth);
    hold(ax1, 'on');
    plot(ax1, t*1000, p_m(:,axis_idx), 'r--', 'LineWidth', styles.measurement_linewidth);
    xlabel(ax1, 'Time [ms]', 'FontSize', styles.xlabel_fontsize);
    ylabel(ax1, 'Position [um]', 'FontSize', styles.ylabel_fontsize);
    title(ax1, 'Position Tracking', 'FontSize', styles.title_fontsize-2);
    legend(ax1, {'Desired', 'Actual'}, 'Location', 'best', 'FontSize', styles.legend_fontsize);
    grid(ax1, 'on');
    ax1.LineWidth = styles.axis_linewidth;

    % Subplot 2: Tracking error
    ax2 = nexttile(tl);
    p_error_nm = (p_m(:,axis_idx) - p_d(:,axis_idx)) * 1000;
    plot(ax2, t*1000, p_error_nm, 'Color', colors(axis_idx,:), ...
        'LineWidth', styles.measurement_linewidth);
    xlabel(ax2, 'Time [ms]', 'FontSize', styles.xlabel_fontsize);
    ylabel(ax2, 'Error [nm]', 'FontSize', styles.ylabel_fontsize);
    title(ax2, sprintf('Tracking Error (RMS: %.1f nm)', sqrt(mean(p_error_nm.^2))), ...
        'FontSize', styles.title_fontsize-2);
    grid(ax2, 'on');
    ax2.LineWidth = styles.axis_linewidth;

    % Subplot 3: Control force
    ax3 = nexttile(tl);
    plot(ax3, t*1000, f_d(:,axis_idx), 'Color', colors(axis_idx,:), ...
        'LineWidth', styles.measurement_linewidth);
    xlabel(ax3, 'Time [ms]', 'FontSize', styles.xlabel_fontsize);
    ylabel(ax3, 'Force [pN]', 'FontSize', styles.ylabel_fontsize);
    title(ax3, sprintf('Control Force (Max: %.1f pN)', max(abs(f_d(:,axis_idx)))), ...
        'FontSize', styles.title_fontsize-2);
    grid(ax3, 'on');
    ax3.LineWidth = styles.axis_linewidth;
end

function tab = create_position_overview_tab(tabgroup, t, p_d, p_m, styles)
    tab = uitab(tabgroup, 'Title', 'Position Overview');
    tl = tiledlayout(tab, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, 'Position Overview', 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    axis_names = {'X', 'Y', 'Z'};
    colors = styles.force_colors;

    for i = 1:3
        ax = nexttile(tl);
        plot(ax, t*1000, p_d(:,i), 'Color', colors(i,:), 'LineWidth', styles.reference_linewidth);
        hold(ax, 'on');
        plot(ax, t*1000, p_m(:,i), '--', 'Color', colors(i,:)*0.7, ...
            'LineWidth', styles.measurement_linewidth);
        xlabel(ax, 'Time [ms]', 'FontSize', styles.xlabel_fontsize);
        ylabel(ax, sprintf('%s [um]', axis_names{i}), 'FontSize', styles.ylabel_fontsize);
        if i == 1
            legend(ax, {'Desired', 'Actual'}, 'Location', 'best', 'FontSize', styles.legend_fontsize);
        end
        grid(ax, 'on');
        ax.LineWidth = styles.axis_linewidth;
    end
end

function tab = create_force_overview_tab(tabgroup, t, f_d, styles)
    tab = uitab(tabgroup, 'Title', 'Force Overview');
    tl = tiledlayout(tab, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, 'Control Force Overview', 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    axis_names = {'Fx', 'Fy', 'Fz'};
    colors = styles.force_colors;

    for i = 1:3
        ax = nexttile(tl);
        plot(ax, t*1000, f_d(:,i), 'Color', colors(i,:), 'LineWidth', styles.measurement_linewidth);
        xlabel(ax, 'Time [ms]', 'FontSize', styles.xlabel_fontsize);
        ylabel(ax, sprintf('%s [pN]', axis_names{i}), 'FontSize', styles.ylabel_fontsize);
        title(ax, sprintf('%s (Max: %.1f pN)', axis_names{i}, max(abs(f_d(:,i)))), ...
            'FontSize', styles.title_fontsize-2);
        grid(ax, 'on');
        ax.LineWidth = styles.axis_linewidth;
    end
end

function tab = create_voltage_tracking_tab(tabgroup, t, v_d, v_m, styles)
    tab = uitab(tabgroup, 'Title', 'Voltage Tracking');
    tl = tiledlayout(tab, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, 'Voltage Tracking (6 Channels)', ...
        'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    colors = styles.channel_colors;

    % Downsample for display (show last 100ms)
    t_display_start = max(0, t(end) - 0.1);
    idx = t >= t_display_start;
    t_disp = t(idx);
    v_d_disp = v_d(idx, :);
    v_m_disp = v_m(idx, :);

    for ch = 1:6
        ax = nexttile(tl);
        plot(ax, t_disp*1000, v_d_disp(:,ch), 'Color', styles.theory_color, ...
            'LineWidth', styles.reference_linewidth);
        hold(ax, 'on');
        plot(ax, t_disp*1000, v_m_disp(:,ch), 'Color', colors(ch,:), ...
            'LineWidth', styles.measurement_linewidth);
        xlabel(ax, 'Time [ms]', 'FontSize', styles.xlabel_fontsize-2);
        ylabel(ax, 'Voltage [V]', 'FontSize', styles.ylabel_fontsize-2);
        title(ax, sprintf('P%d', ch), 'FontSize', styles.title_fontsize-2);
        if ch == 1
            legend(ax, {'Vd', 'Vm'}, 'Location', 'best', 'FontSize', styles.legend_fontsize-2);
        end
        grid(ax, 'on');
        ax.LineWidth = styles.axis_linewidth;
        ax.FontSize = styles.tick_fontsize - 2;
    end
end

function tab = create_control_output_tab(tabgroup, t, u, styles)
    tab = uitab(tabgroup, 'Title', 'Control Output');

    n_channels = size(u, 2);

    if n_channels >= 6
        tl = tiledlayout(tab, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl, 'Control Output u (6 Channels)', ...
            'FontSize', styles.title_fontsize, 'FontWeight', 'bold');
        channels_to_plot = 6;
    else
        % Handle unexpected dimensions
        tl = tiledlayout(tab, 1, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl, sprintf('Control Output u (%d channel(s) - check Simulink model)', n_channels), ...
            'FontSize', styles.title_fontsize, 'FontWeight', 'bold');
        channels_to_plot = n_channels;
    end

    colors = styles.channel_colors;

    % Downsample for display (show last 100ms)
    t_display_start = max(0, t(end) - 0.1);
    idx = t >= t_display_start;
    t_disp = t(idx);
    u_disp = u(idx, :);

    for ch = 1:channels_to_plot
        ax = nexttile(tl);
        color_idx = min(ch, size(colors, 1));
        plot(ax, t_disp*1000, u_disp(:,ch), 'Color', colors(color_idx,:), ...
            'LineWidth', styles.measurement_linewidth);
        xlabel(ax, 'Time [ms]', 'FontSize', styles.xlabel_fontsize-2);
        ylabel(ax, 'u [V]', 'FontSize', styles.ylabel_fontsize-2);
        title(ax, sprintf('P%d', ch), 'FontSize', styles.title_fontsize-2);
        grid(ax, 'on');
        ax.LineWidth = styles.axis_linewidth;
        ax.FontSize = styles.tick_fontsize - 2;
    end
end

function tab = create_fft_spectrum_tab(tabgroup, t, p_m, fs, cutoff_freq, styles)
    tab = uitab(tabgroup, 'Title', 'FFT Spectrum');
    tl = tiledlayout(tab, 1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, 'Position FFT Spectrum', 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    axis_names = {'X', 'Y', 'Z'};
    colors = styles.force_colors;

    N = size(p_m, 1);
    N_half = floor(N/2);
    f = (0:N_half) * fs / N;

    for i = 1:3
        ax = nexttile(tl);

        % FFT
        Y = fft(p_m(:,i) - mean(p_m(:,i)));
        P = abs(Y(1:N_half+1)) / N * 2;
        P(1) = P(1) / 2;  % DC component

        loglog(ax, f, P, 'Color', colors(i,:), 'LineWidth', styles.measurement_linewidth);
        hold(ax, 'on');

        % Cutoff frequency line
        xline(ax, cutoff_freq, 'k--', 'LineWidth', 1.5);

        xlabel(ax, 'Frequency [Hz]', 'FontSize', styles.xlabel_fontsize);
        ylabel(ax, 'Amplitude [um]', 'FontSize', styles.ylabel_fontsize);
        title(ax, sprintf('%s-axis', axis_names{i}), 'FontSize', styles.title_fontsize-2);
        xlim(ax, [0.1, fs/2]);
        grid(ax, 'on');
        ax.LineWidth = styles.axis_linewidth;
    end
end

function tab = create_separation_tab(tabgroup, t, p_m, fs, cutoff_freq, styles)
    tab = uitab(tabgroup, 'Title', 'Det/Random');
    tl = tiledlayout(tab, 3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, sprintf('p_m Deterministic/Random Separation (cutoff: %.1f Hz)', cutoff_freq), ...
        'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    axis_names = {'X', 'Y', 'Z'};
    colors = styles.force_colors;

    % Design low-pass filter
    [b, a] = butter(4, cutoff_freq / (fs/2), 'low');

    for i = 1:3
        % Separate p_m into deterministic (low-freq) and random (high-freq)
        det_comp = filtfilt(b, a, p_m(:,i));
        rand_comp = p_m(:,i) - det_comp;

        % Left column: Deterministic (low-frequency component)
        ax1 = nexttile(tl, 2*(i-1)+1);
        plot(ax1, t*1000, det_comp, 'Color', colors(i,:), ...
            'LineWidth', styles.measurement_linewidth);
        xlabel(ax1, 'Time [ms]', 'FontSize', styles.xlabel_fontsize-2);
        ylabel(ax1, sprintf('%s [um]', axis_names{i}), 'FontSize', styles.ylabel_fontsize-2);
        if i == 1
            title(ax1, 'Deterministic (Low-freq)', 'FontSize', styles.title_fontsize-2);
        end
        grid(ax1, 'on');
        ax1.LineWidth = styles.axis_linewidth;

        % Right column: Random (high-frequency component)
        ax2 = nexttile(tl, 2*(i-1)+2);
        plot(ax2, t*1000, rand_comp*1000, 'Color', colors(i,:), ...
            'LineWidth', styles.measurement_linewidth);
        xlabel(ax2, 'Time [ms]', 'FontSize', styles.xlabel_fontsize-2);
        ylabel(ax2, sprintf('%s [nm]', axis_names{i}), 'FontSize', styles.ylabel_fontsize-2);
        if i == 1
            title(ax2, 'Random (High-freq)', 'FontSize', styles.title_fontsize-2);
        end
        grid(ax2, 'on');
        ax2.LineWidth = styles.axis_linewidth;
    end
end

function tab = create_wall_distance_tab(tabgroup, t, h, h_R, h_min, R, styles)
    tab = uitab(tabgroup, 'Title', 'Wall Distance');
    tl = tiledlayout(tab, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, 'Wall Distance Monitoring', 'FontSize', styles.title_fontsize, 'FontWeight', 'bold');

    % Top: h vs time
    ax1 = nexttile(tl);
    plot(ax1, t*1000, h, 'b-', 'LineWidth', styles.measurement_linewidth);
    hold(ax1, 'on');
    yline(ax1, h_min, 'r--', 'LineWidth', 2, 'Label', sprintf('h_{min}=%.2f um', h_min));
    xlabel(ax1, 'Time [ms]', 'FontSize', styles.xlabel_fontsize);
    ylabel(ax1, 'h [um]', 'FontSize', styles.ylabel_fontsize);
    title(ax1, 'Wall Distance', 'FontSize', styles.title_fontsize-2);
    grid(ax1, 'on');
    ax1.LineWidth = styles.axis_linewidth;

    % Bottom: h/R vs time
    ax2 = nexttile(tl);
    plot(ax2, t*1000, h_R, 'b-', 'LineWidth', styles.measurement_linewidth);
    hold(ax2, 'on');
    yline(ax2, h_min/R, 'r--', 'LineWidth', 2, 'Label', sprintf('h_{min}/R=%.2f', h_min/R));
    yline(ax2, 1.5, 'k:', 'LineWidth', 1.5, 'Label', 'h/R=1.5 (safety)');
    xlabel(ax2, 'Time [ms]', 'FontSize', styles.xlabel_fontsize);
    ylabel(ax2, 'h/R', 'FontSize', styles.ylabel_fontsize);
    title(ax2, 'Normalized Wall Distance', 'FontSize', styles.title_fontsize-2);
    grid(ax2, 'on');
    ax2.LineWidth = styles.axis_linewidth;
end
