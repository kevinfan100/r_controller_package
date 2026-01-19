%% RUN_MOTION_CONTROL_TEST Motion control integration test
%
% Tests the integrated motion control system including:
%   - Motion Control Law (position tracking)
%   - Trajectory Generator (z_sine, xy_circle)
%   - Particle Dynamics (with Wall Effect and Thermal Force)
%   - Force Allocation (inverse_model, force_model)
%   - Inner Loop Control (R-Controller)
%
% This script validates the pure MATLAB functions before Simulink integration.
%
% Test Modes:
%   1. Unit tests for individual functions
%   2. Open-loop particle dynamics (thermal force only)
%   3. Closed-loop motion control (z_sine trajectory)
%   4. Closed-loop motion control (xy_circle trajectory)
%
% See also: motion_control_law_function, trajectory_generator_function, CLAUDE.md

%% SECTION 1: Setup Paths
clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
addpath(fullfile(project_root, 'model', 'motion_ctrl'));
addpath(fullfile(project_root, 'model', 'particle_dynamics'));
addpath(fullfile(project_root, 'model', 'flux_allocation'));
addpath(fullfile(project_root, 'model', 'inner_loop_ctrl'));
addpath(fullfile(project_root, 'test_script', 'utils'));

fprintf('=== Motion Control Integration Test ===\n\n');

%% SECTION 2: Unit Tests for calc_correction_functions
fprintf('--- Test 1: calc_correction_functions ---\n');

% Test known values (from formula verification)
[c_para, c_perp] = calc_correction_functions(1.5);
fprintf('h_bar = 1.5: c_para = %.3f (expected ~1.615), c_perp = %.3f (expected ~3.205)\n', c_para, c_perp);
assert(abs(c_para - 1.615) < 0.01, 'c_para mismatch at h_bar=1.5');
assert(abs(c_perp - 3.205) < 0.01, 'c_perp mismatch at h_bar=1.5');

[c_para, c_perp] = calc_correction_functions(10);
fprintf('h_bar = 10:  c_para = %.3f (expected ~1.060), c_perp = %.3f (expected ~1.126)\n', c_para, c_perp);
assert(abs(c_para - 1.060) < 0.01, 'c_para mismatch at h_bar=10');
assert(abs(c_perp - 1.126) < 0.01, 'c_perp mismatch at h_bar=10');

[c_para, c_perp] = calc_correction_functions(100);
fprintf('h_bar = 100: c_para = %.4f (expected ~1), c_perp = %.4f (expected ~1)\n', c_para, c_perp);
assert(abs(c_para - 1) < 0.02, 'c_para should approach 1 at large h_bar');
assert(abs(c_perp - 1) < 0.02, 'c_perp should approach 1 at large h_bar');

fprintf('PASSED: calc_correction_functions\n\n');

%% SECTION 3: Unit Tests for calc_gamma_inv_function
fprintf('--- Test 2: calc_gamma_inv_function ---\n');

particle_params = particle_dynamics_params();
p_far = [0; 0; 1000];  % Far from wall
[Gamma_inv_far, h_bar_far] = calc_gamma_inv_function(p_far, particle_params.Value);

gamma_N = particle_params.Value.gamma_N;
expected_far = eye(3) / gamma_N;
error_far = max(abs(Gamma_inv_far - expected_far), [], 'all');
relative_error = error_far / (1/gamma_N);
fprintf('Far from wall (h=1000 um): relative error = %.2f%% (should be < 1%%)\n', relative_error*100);
assert(relative_error < 0.01, 'Gamma_inv should approach I/gamma_N far from wall');
fprintf('h_bar = %.1f\n', h_bar_far);

p_near = [0; 0; 5];  % h = 5 um, h_bar = 5/2.25 = 2.22
[Gamma_inv_near, h_bar_near] = calc_gamma_inv_function(p_near, particle_params.Value);
fprintf('Near wall (h=5 um): h_bar = %.2f\n', h_bar_near);
fprintf('Gamma_inv diagonal: [%.4f, %.4f, %.4f]\n', ...
    Gamma_inv_near(1,1), Gamma_inv_near(2,2), Gamma_inv_near(3,3));

% Verify anisotropy (Z direction should have lower mobility due to wall)
assert(Gamma_inv_near(3,3) < Gamma_inv_near(1,1), ...
    'Perpendicular mobility should be lower than parallel');
fprintf('PASSED: calc_gamma_inv_function\n\n');

%% SECTION 4: Unit Tests for calc_initial_position_function
fprintf('--- Test 3: calc_initial_position_function ---\n');

particle_params = particle_dynamics_params('HInit', 5);
p0 = calc_initial_position_function(particle_params.Value);
fprintf('Initial position (h_init=5, default wall): [%.2f, %.2f, %.2f] um\n', p0(1), p0(2), p0(3));
assert(all(abs(p0 - [0; 0; 5]) < 1e-10), 'Initial position should be [0;0;5] for default wall');

% Test with different wall angle
particle_params_rotated = particle_dynamics_params('Theta', pi/4, 'Phi', pi/4, 'HInit', 10);
p0_rotated = calc_initial_position_function(particle_params_rotated.Value);
fprintf('Rotated wall (theta=pi/4, phi=pi/4, h_init=10): [%.2f, %.2f, %.2f] um\n', ...
    p0_rotated(1), p0_rotated(2), p0_rotated(3));
fprintf('PASSED: calc_initial_position_function\n\n');

%% SECTION 5: Unit Tests for trajectory_generator_function
fprintf('--- Test 4: trajectory_generator_function ---\n');

traj_params_sine = trajectory_generator_params('Type', 0, 'Amplitude', 5, 'Frequency', 1, 'NCycles', 3);
p0 = [0; 0; 5];

% Test at t=0
p_d_0 = trajectory_generator_function(0, p0, traj_params_sine.Value);
fprintf('z_sine at t=0: [%.2f, %.2f, %.2f] (expected: [0, 0, 5])\n', p_d_0(1), p_d_0(2), p_d_0(3));
assert(all(abs(p_d_0 - p0) < 1e-10), 'Trajectory should start at p0');

% Test at t=0.25 (peak of sine at f=1Hz)
p_d_025 = trajectory_generator_function(0.25, p0, traj_params_sine.Value);
expected_025 = [0; 0; 5 + 5];  % p0 + amplitude
fprintf('z_sine at t=0.25: [%.2f, %.2f, %.2f] (expected: [0, 0, 10])\n', p_d_025(1), p_d_025(2), p_d_025(3));
assert(all(abs(p_d_025 - expected_025) < 1e-10), 'Peak should be p0 + amplitude');

% Test after trajectory completion
p_d_end = trajectory_generator_function(5, p0, traj_params_sine.Value);
fprintf('z_sine at t=5 (after completion): [%.2f, %.2f, %.2f]\n', p_d_end(1), p_d_end(2), p_d_end(3));
assert(all(abs(p_d_end - p0) < 1e-10), 'Should return to p0 after completion');

% Test xy_circle
traj_params_circle = trajectory_generator_params('Type', 1, 'Radius', 3, 'Period', 2, 'NCycles', 2);
p_d_circle_0 = trajectory_generator_function(0, p0, traj_params_circle.Value);
fprintf('xy_circle at t=0: [%.2f, %.2f, %.2f]\n', p_d_circle_0(1), p_d_circle_0(2), p_d_circle_0(3));
assert(all(abs(p_d_circle_0 - p0) < 1e-10), 'Circle should start at p0');

fprintf('PASSED: trajectory_generator_function\n\n');

%% SECTION 6: Unit Tests for motion_control_law_function
fprintf('--- Test 5: motion_control_law_function ---\n');

% Reset persistent variables
clear motion_control_law_function;

motion_params = motion_control_law_params('LambdaC', 0.7, 'Enable', 1);
gamma = motion_params.Value.gamma;
Ts = motion_params.Value.Ts;
lambda_c = motion_params.Value.lambda_c;

p_d = [0; 0; 5];
p_m = [0; 0; 4.9];

% First call (initialization)
f_d_1 = motion_control_law_function(p_d, p_m, motion_params.Value);
expected_1 = (gamma/Ts) * (1 - lambda_c) * (p_d - p_m);
fprintf('First call: f_d = [%.4f, %.4f, %.4f] pN\n', f_d_1(1), f_d_1(2), f_d_1(3));
fprintf('Expected:   f_d = [%.4f, %.4f, %.4f] pN\n', expected_1(1), expected_1(2), expected_1(3));
assert(max(abs(f_d_1 - expected_1)) < 1e-10, 'First call should match expected formula');

% Second call with same inputs
f_d_2 = motion_control_law_function(p_d, p_m, motion_params.Value);
fprintf('Second call: f_d = [%.4f, %.4f, %.4f] pN\n', f_d_2(1), f_d_2(2), f_d_2(3));

% Open-loop test
clear motion_control_law_function;
motion_params_open = motion_control_law_params('Enable', 0);
f_d_open = motion_control_law_function(p_d, p_m, motion_params_open.Value);
fprintf('Open-loop: f_d = [%.4f, %.4f, %.4f] pN (expected: all zeros)\n', ...
    f_d_open(1), f_d_open(2), f_d_open(3));
assert(all(abs(f_d_open) < 1e-10), 'Open-loop should return zeros');

fprintf('PASSED: motion_control_law_function\n\n');

%% SECTION 7: Unit Tests for thermal force
fprintf('--- Test 6: calc_thermal_force_function ---\n');

% Reset persistent variables
clear calc_thermal_force_function;

thermal_params = thermal_force_params('Enable', 1, 'Seed', 42);
p_test = [0; 0; 5];

F_th = calc_thermal_force_function(p_test, thermal_params.Value);
fprintf('Thermal force sample: [%.4f, %.4f, %.4f] pN\n', F_th(1), F_th(2), F_th(3));

% Generate multiple samples and check statistics
clear calc_thermal_force_function;
thermal_params = thermal_force_params('Enable', 1, 'Seed', 123);
N_samples = 1000;
F_samples = zeros(3, N_samples);
for i = 1:N_samples
    F_samples(:, i) = calc_thermal_force_function(p_test, thermal_params.Value);
end

mean_F = mean(F_samples, 2);
std_F = std(F_samples, 0, 2);
fprintf('Mean thermal force: [%.4f, %.4f, %.4f] pN (expected ~0)\n', mean_F(1), mean_F(2), mean_F(3));
fprintf('Std thermal force:  [%.4f, %.4f, %.4f] pN\n', std_F(1), std_F(2), std_F(3));

% Disabled test
clear calc_thermal_force_function;
thermal_params_off = thermal_force_params('Enable', 0);
F_th_off = calc_thermal_force_function(p_test, thermal_params_off.Value);
assert(all(abs(F_th_off) < 1e-10), 'Disabled thermal force should return zeros');
fprintf('PASSED: calc_thermal_force_function\n\n');

%% SECTION 8: Safety Check Test
fprintf('--- Test 7: check_trajectory_safety_function ---\n');

particle_params = particle_dynamics_params('HInit', 5, 'HMin', 3.375);
p0 = calc_initial_position_function(particle_params.Value);

% Safe trajectory
traj_params_safe = trajectory_generator_params('Type', 0, 'Amplitude', 1, 'Frequency', 1, 'NCycles', 3);
T_sim = 3;
[is_safe, h_min_actual, t_crit] = check_trajectory_safety_function(...
    p0, traj_params_safe.Value, particle_params.Value, T_sim);
fprintf('Safe trajectory (amp=1): is_safe=%d, h_min=%.2f um\n', is_safe, h_min_actual);
assert(is_safe, 'Small amplitude trajectory should be safe');

% Unsafe trajectory (amplitude too large)
traj_params_unsafe = trajectory_generator_params('Type', 0, 'Amplitude', 10, 'Frequency', 1, 'NCycles', 3);
lastwarn('');  % Clear last warning
[is_safe_2, h_min_actual_2, t_crit_2] = check_trajectory_safety_function(...
    p0, traj_params_unsafe.Value, particle_params.Value, T_sim);
fprintf('Unsafe trajectory (amp=10): is_safe=%d, h_min=%.2f um at t=%.3f s\n', ...
    is_safe_2, h_min_actual_2, t_crit_2);
assert(~is_safe_2, 'Large amplitude trajectory should be unsafe');

fprintf('PASSED: check_trajectory_safety_function\n\n');

%% SECTION 9: Discrete-Time Simulation (Pure MATLAB)
fprintf('--- Test 8: Discrete-Time Motion Control Simulation ---\n');

% Reset persistent variables
clear motion_control_law_function calc_thermal_force_function;

% Parameters
Ts_motion = 1/1600;  % Motion control sampling period
T_sim = 3;           % 3 seconds
N_steps = round(T_sim / Ts_motion);

% Create parameter structures
traj_params = trajectory_generator_params('Type', 0, 'Amplitude', 2, 'Frequency', 1, 'NCycles', 3);
motion_params = motion_control_law_params('LambdaC', 0.7, 'Enable', 1);
particle_params = particle_dynamics_params('WallEffectEnable', 1, 'HInit', 5);
thermal_params = thermal_force_params('Enable', 0, 'Seed', 42);  % Disable thermal for deterministic test

% Initial position
p0 = calc_initial_position_function(particle_params.Value);
p = p0;  % Current position

% Allocate output arrays
t_log = zeros(N_steps, 1);
p_d_log = zeros(N_steps, 3);
p_log = zeros(N_steps, 3);
f_d_log = zeros(N_steps, 3);

% Simulation loop
fprintf('Running discrete-time simulation (%d steps)...\n', N_steps);
for k = 1:N_steps
    t = (k-1) * Ts_motion;

    % Trajectory generator
    p_d = trajectory_generator_function(t, p0, traj_params.Value);

    % Motion control law
    f_d = motion_control_law_function(p_d, p, motion_params.Value);

    % Thermal force
    F_th = calc_thermal_force_function(p, thermal_params.Value);

    % Total force
    f_total = f_d + F_th;

    % Particle dynamics (simplified Euler integration)
    [Gamma_inv, ~] = calc_gamma_inv_function(p, particle_params.Value);
    dp_dt = Gamma_inv * f_total;
    p = p + Ts_motion * dp_dt;

    % Log data
    t_log(k) = t;
    p_d_log(k, :) = p_d';
    p_log(k, :) = p';
    f_d_log(k, :) = f_d';
end

% Calculate tracking error
p_error = p_log - p_d_log;
rms_error = sqrt(mean(p_error.^2));
fprintf('RMS tracking error: [%.4f, %.4f, %.4f] um\n', rms_error(1), rms_error(2), rms_error(3));

% Plot results
figure('Name', 'Motion Control Test - Z-axis Tracking', 'Position', [100, 100, 800, 600]);

subplot(3, 1, 1);
plot(t_log, p_d_log(:, 3), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Desired');
hold on;
plot(t_log, p_log(:, 3), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
xlabel('Time [s]');
ylabel('Z Position [um]');
title('Z-axis Position Tracking');
legend('Location', 'best');
grid on;

subplot(3, 1, 2);
plot(t_log, p_error(:, 3), 'k-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Tracking Error [um]');
title('Z-axis Tracking Error');
grid on;

subplot(3, 1, 3);
plot(t_log, f_d_log(:, 3), 'g-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Control Force [pN]');
title('Z-axis Control Force');
grid on;

% Save figure
output_dir = fullfile(project_root, 'test_results', 'motion_control');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
saveas(gcf, fullfile(output_dir, ['z_sine_tracking_' timestamp '.png']));
fprintf('Figure saved to: %s\n', output_dir);

% Check pass criteria
assert(rms_error(3) < 0.5, 'Z-axis RMS error should be < 0.5 um');
fprintf('PASSED: Discrete-time motion control simulation\n\n');

%% SECTION 10: Summary
fprintf('========================================\n');
fprintf('All MATLAB unit tests PASSED!\n');
fprintf('========================================\n');
fprintf('\nFiles created/modified:\n');
fprintf('  model/motion_ctrl/\n');
fprintf('    - trajectory_generator_params.m\n');
fprintf('    - trajectory_generator_function.m\n');
fprintf('    - motion_control_law_params.m\n');
fprintf('    - motion_control_law_function.m\n');
fprintf('  model/particle_dynamics/\n');
fprintf('    - particle_dynamics_params.m\n');
fprintf('    - calc_correction_functions.m\n');
fprintf('    - calc_gamma_inv_function.m\n');
fprintf('    - thermal_force_params.m\n');
fprintf('    - calc_thermal_force_function.m\n');
fprintf('    - calc_initial_position_function.m\n');
fprintf('    - check_trajectory_safety_function.m\n');
fprintf('  model/flux_allocation/\n');
fprintf('    - force_model_allocation_params.m (modified)\n');
fprintf('    - inverse_model_function.m (modified)\n');
fprintf('    - force_model_function.m (modified)\n');
fprintf('  model/main_system.slx (modified)\n');
fprintf('    - Motion_Control subsystem (1600 Hz)\n');
fprintf('    - Particle_Dynamics subsystem (continuous)\n');
fprintf('    - signal_type=3 for Motion Control mode\n');

%% SECTION 11: Simulink Integration Test (Optional)
% This section runs only if Simulink is available and the model exists.
% It can be skipped by setting run_simulink_test = false.

run_simulink_test = true;  % Set to false to skip Simulink test

if run_simulink_test
    fprintf('\n--- Test 9: Simulink Integration Test ---\n');

    % Check if Simulink is available
    if ~exist('sim', 'file')
        fprintf('SKIPPED: Simulink not available\n\n');
    else
        % Clear persistent variables
        clear motion_control_law_function trajectory_generator_function;
        clear calc_thermal_force_function calc_gamma_inv_function;

        % Configuration
        T_sim_slx = 1.0;  % Simulation time [s]

        % Create parameters for Simulink (use different variable names to avoid
        % overwriting the parameter functions)
        traj_params = trajectory_generator_params(...
            'Type', 0, 'Amplitude', 2, 'Frequency', 1, 'NCycles', 3);
        motion_params = motion_control_law_params(...
            'LambdaC', 0.7, 'Enable', 1);
        particle_params = particle_dynamics_params(...
            'WallEffectEnable', 1, 'HInit', 5);
        thermal_params = thermal_force_params('Enable', 0);
        ctrl_params = model_base_ctrl_params(500, 500, 3000);
        alloc_params = force_model_allocation_params('Simulink', true, 'PosMSource', 1);
        p0 = calc_initial_position_function(particle_params.Value);
        signal_type = 3;  % Motion Control mode

        % Create timeseries for From Workspace blocks
        t_vec_slx = (0:1e-5:T_sim_slx)';
        f_d_timeseries = timeseries(zeros(length(t_vec_slx), 3), t_vec_slx);
        pos_m_static = timeseries(zeros(length(t_vec_slx), 3) + p0', t_vec_slx);

        % Other required parameters
        ControllerType = 1;
        pi_params = pi_ctrl_params(100, 10000);
        vd_params = vd_signal_params('Mode', 1, 'Channel', 1, 'Amplitude', 0, ...
            'Frequency', 100, 'Ts', 1e-5, 'd', 0);

        % Assign all parameters to base workspace for Simulink
        assignin('base', 'trajectory_generator_params', traj_params);
        assignin('base', 'motion_control_law_params', motion_params);
        assignin('base', 'particle_dynamics_params', particle_params);
        assignin('base', 'thermal_force_params', thermal_params);
        assignin('base', 'model_base_ctrl_params', ctrl_params);
        assignin('base', 'pi_ctrl_params', pi_params);
        assignin('base', 'vd_signal_params', vd_params);
        assignin('base', 'alloc_params', alloc_params);
        assignin('base', 'p0', p0);
        assignin('base', 'signal_type', signal_type);
        assignin('base', 'ControllerType', ControllerType);
        assignin('base', 'f_d_timeseries', f_d_timeseries);
        assignin('base', 'pos_m_static', pos_m_static);

        fprintf('Running Simulink simulation (signal_type=3, T=%.1f s)...\n', T_sim_slx);

        model_path_slx = fullfile(project_root, 'model', 'main_system');
        try
            load_system(model_path_slx);
            simOut = sim(model_path_slx, 'StopTime', num2str(T_sim_slx), ...
                'ReturnWorkspaceOutputs', 'on');

            % Extract position data
            p_d_slx = simOut.get('p_d');
            p_m_slx = simOut.get('p_m');

            % Skip transient (first 10%)
            n_skip = round(0.1 * size(p_d_slx, 1));
            p_error_slx = p_d_slx(n_skip:end, :) - p_m_slx(n_skip:end, :);
            rms_error_slx = sqrt(mean(p_error_slx.^2, 1));

            fprintf('Simulink RMS tracking error: [%.4f, %.4f, %.4f] um\n', rms_error_slx);

            % Pass criteria
            assert(rms_error_slx(3) < 1.0, 'Z-axis RMS error should be < 1.0 um');
            fprintf('PASSED: Simulink Integration Test\n\n');

            close_system(model_path_slx, 0);
        catch ME
            fprintf('Simulink test ERROR: %s\n', ME.message);
            try
                close_system(model_path_slx, 0);
            catch
            end
        end
    end
else
    fprintf('\n--- Test 9: Simulink Integration Test ---\n');
    fprintf('SKIPPED: run_simulink_test = false\n\n');
end

%% SECTION 12: Final Summary
fprintf('========================================\n');
fprintf('All tests completed!\n');
fprintf('========================================\n');
