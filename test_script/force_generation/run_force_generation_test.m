% run_force_generation_test.m
% Phase 2 Integration Test: Force Control with Inverse Model
%
% This script demonstrates the complete force control pipeline:
%   1. Set desired force signal f_d
%   2. Call inverse_model to compute vd timeseries
%   3. Execute Simulink simulation (Model Based Control tracks vd)
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
addpath(fullfile(package_root, 'model', 'motion_ctrl'));
addpath(fullfile(package_root, 'model', 'particle_dynamics'));
addpath(fullfile(package_root, 'test_script', 'utils'));


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
%   'model_base_ctrl' - Model Based Control (discrete-time, feedforward + DOB + PI)
%   'pi_controller' - PI Controller (classic proportional-integral)
controller_type = 'model_base_ctrl';   % 'model_base_ctrl' or 'pi_controller'

% ─────────────────────────────────────────────────────────────────────────
% 1.2.1 Model Based Control Feedforward Settings (only when controller_type = 'model_base_ctrl')
% ─────────────────────────────────────────────────────────────────────────
% ff_enable: Enable/disable feedforward filter
%   true  - Feedforward enabled (ZPETC tracking)
%   false - Feedforward disabled (closed-loop only)
%
% ff_preview: Preview steps for ZPETC (only effective when ff_enable = true)
%   0 - No preview (ZPETC d=0, phase lag = -2*theta)
%   2 - 2-step preview (ZPETC d=2, zero phase error)
%
ff_enable = true;                       % Enable feedforward filter
ff_preview = 0;                         % Preview steps: 0 or 2

% ─────────────────────────────────────────────────────────────────────────
% 1.2.2 LPF Configuration
% ─────────────────────────────────────────────────────────────────────────
% lpf_enable: Enable LPF mode (3rd-order controller)
%   false = Use original 2nd-order controller, Simulink LPF bypassed
%   true  = Use new 3rd-order controller, Simulink LPF enabled
%
% f_low: LPF cutoff frequency [Hz] (only used when lpf_enable=true)
%   Typical values: 5000, 10000, 20000 Hz
%
lpf_enable = false;                     % Default: disabled for backward compatibility
f_low = 10000;                          % Default: 10 kHz

% ─────────────────────────────────────────────────────────────────────────
% 1.3 Desired Force Signal (f_d)
% ─────────────────────────────────────────────────────────────────────────
signal_type = 'sine';           % 'sine' or 'step'

% Force direction and magnitude
force_direction = [1; 0; 0];    % Unit direction vector [Fx; Fy; Fz]
force_amplitude = 5.0;          % Force amplitude [pN]
force_offset    = 0;

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
generation_mode = 'hardware';   % 'hardware' or 'ideal'

% Hardware mode parameters (only used when generation_mode = 'hardware')
pos_update_rate = 1600;         % Position update frequency [Hz]
interp_method = 'linear';       % Interpolation method: 'linear' or 'previous' (ZOH)
USE_REALTIME_INTERP = true;     % true = Real-time (1-period delay), false = Ideal (non-causal)

% ─────────────────────────────────────────────────────────────────────────
% 1.6 Model Based Control Parameters
% ─────────────────────────────────────────────────────────────────────────
fB_f = 1000;                    % Feedforward bandwidth [Hz]
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
USE_SIMULINK = true;            % true: use Simulink Model Based Control
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

% ─────────────────────────────────────────────────────────────────────────
% 1.11 Hardware Constants
% ─────────────────────────────────────────────────────────────────────────
% Voltage to current conversion gain (per channel)
K_A_DIAG = [0.3618, 0.3614, 0.3536, 0.3532, 0.3573, 0.3610];

% ─────────────────────────────────────────────────────────────────────────
% 1.12 Control Output Low-Pass Filter (in ZOH Plant)
% ─────────────────────────────────────────────────────────────────────────
% u_lpf_enable: Simulink LPF switch - automatically linked to lpf_enable
%   When lpf_enable=false: u_lpf_enable=0 (Simulink LPF bypassed)
%   When lpf_enable=true:  u_lpf_enable=1 (Simulink LPF enabled)
%
% f_low: LPF cutoff frequency [Hz] (shared with controller)
%   Transfer function: H(s) = w_low / (s + w_low), where w_low = 2*pi*f_low
%
u_lpf_enable = double(lpf_enable);  % Link to controller lpf_enable
% f_low is already set in LPF Configuration section (1.2.2)


%%                        SECTION 2: System Initialization


fprintf('【System Initialization】\n');
fprintf('────────────────────────\n');

% Validate controller type
controller_type = lower(controller_type);
if ~ismember(controller_type, {'model_base_ctrl', 'pi_controller'})
    error('Invalid controller_type: %s. Use ''model_base_ctrl'' or ''pi_controller''.', controller_type);
end

% Set ControllerType for Simulink (1=Model Based Control, 2=PI-Controller)
if strcmpi(controller_type, 'model_base_ctrl')
    ControllerType = 1;
    % Generate controller label with feedforward/LPF mode
    if lpf_enable
        controller_label = sprintf('Model Based Control (LPF f_low=%d Hz)', f_low);
    elseif ~ff_enable
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

% Load Model Based Control parameters (with ff_enable and lpf_enable settings)
model_base_ctrl_params_local = model_base_ctrl_params(fB_c, fB_e, fB_f, ...
    'ff_enable', ff_enable, ...
    'lpf_enable', lpf_enable, ...
    'f_low', f_low);

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
    fprintf('  Model Based Control: fB_f=%d, fB_c=%d, fB_e=%d Hz\n', fB_f, fB_c, fB_e);
    fprintf('  Feedforward: ff_enable=%d, ff_preview=%d\n', ff_enable, ff_preview);
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
    envelope = force_amplitude * sin(2*pi*force_frequency*t + deg2rad(force_phase)) + force_offset;
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


% Save original signal_type before potential Simulink overwrite
signal_type_original = signal_type;  % 'sine' or 'step'

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
        'Frequency', 100, 'Ts', Ts, 'ff_preview', ff_preview);
    assignin('base', 'vd_signal_params', vd_sig_params);

    % ─────────────────────────────────────────────────────────────────────
    % Controller parameters
    % ─────────────────────────────────────────────────────────────────────
    assignin('base', 'ControllerType', ControllerType);

    % Model Based Control parameters
    assignin('base', 'model_base_ctrl_params', model_base_ctrl_params_local);

    % PI-Controller parameters
    pi_ctrl_params_local = pi_ctrl_params(Kp_value, Ki_value, 'Ts', Ts);
    assignin('base', 'pi_ctrl_params', pi_ctrl_params_local);

    % Legacy parameters (for backward compatibility)
    assignin('base', 'Kp_value', Kp_value);
    assignin('base', 'Ki_value', Ki_value);

    % ─────────────────────────────────────────────────────────────────────
    % Control output LPF parameters (for ZOH Plant)
    % ─────────────────────────────────────────────────────────────────────
    w_low = 2*pi*f_low;              % Convert to rad/s
    assignin('base', 'u_lpf_enable', u_lpf_enable);
    assignin('base', 'f_low', f_low);
    assignin('base', 'w_low', w_low);

    % ─────────────────────────────────────────────────────────────────────
    % Motion Control parameters (required by Simulink model even in Force mode)
    % ─────────────────────────────────────────────────────────────────────
    motion_ctrl_params = motion_control_law_params('Enable', 0);
    traj_params = trajectory_generator_params();
    % Motion Control parameters (required by Simulink model even in Force mode)
    particle_params = particle_dynamics_params();
    thermal_params = thermal_force_params('Enable', 0);
    p0 = [0; 0; 5];

    assignin('base', 'motion_control_law_params', motion_ctrl_params);
    assignin('base', 'trajectory_generator_params', traj_params);
    assignin('base', 'particle_dynamics_params', particle_params);
    assignin('base', 'thermal_force_params', thermal_params);
    assignin('base', 'p0', p0);
    assignin('base', 'delay_steps', 0);  % No delay for force generation test

    % pos_m_static timeseries (required by From Workspace blocks)
    pos_m_static = timeseries(zeros(2, 3), [0; sim_time]);
    assignin('base', 'pos_m_static', pos_m_static);

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
    % Vm: measured Hall voltage (6x1) from Model Based Control output
    Vm_ts = simOut.get('Vm');
    if isa(Vm_ts, 'timeseries')
        vm_sim = Vm_ts.Data;
        t_vm = Vm_ts.Time;
    else
        vm_sim = Vm_ts;
        t_vm = (0:size(vm_sim,1)-1)' * Ts;
    end

    % f_m: estimated force (3x1) from Force_Model block
    Fm_ts = simOut.get('f_m');
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
    % Vd: desired Hall voltage (6x1) from Simulink (with rate transition)
    % ─────────────────────────────────────────────────────────────────────
    Vd_ts = simOut.get('Vd');
    if isa(Vd_ts, 'timeseries')
        vd_sim = Vd_ts.Data;
    else
        vd_sim = Vd_ts;
    end
    if N_vm ~= N
        vd = zeros(N, 6);
        for ch = 1:6
            vd(:, ch) = interp1(t_vm, vd_sim(:, ch), t, 'linear', 'extrap');
        end
    else
        vd = vd_sim;
    end
    fprintf('  vd range: [%.4f, %.4f] V\n', min(vd(:)), max(vd(:)));

    % ─────────────────────────────────────────────────────────────────────
    % f_d_interp: interpolated desired force from Simulink inverse_model_function
    % This is the actual f_d used by the inverse model (after rate transition)
    % ─────────────────────────────────────────────────────────────────────
    Fd_interp_ts = simOut.get('f_d_interp');
    if ~isempty(Fd_interp_ts)
        if isa(Fd_interp_ts, 'timeseries')
            fd_interp_sim = Fd_interp_ts.Data;
        else
            fd_interp_sim = Fd_interp_ts;
        end
        if N_vm ~= N
            f_d_interp = zeros(N, 3);
            for ax = 1:3
                f_d_interp(:, ax) = interp1(t_vm, fd_interp_sim(:, ax), t, 'linear', 'extrap');
            end
        else
            f_d_interp = fd_interp_sim;
        end
        fprintf('  f_d_interp range: [%.4f, %.4f] pN (from Simulink)\n', min(f_d_interp(:)), max(f_d_interp(:)));
        has_fd_interp = true;
    else
        % Fallback: f_d_interp not available from Simulink (old model)
        % Use f_d with delay compensation as before
        fprintf('  Warning: f_d_interp not available from Simulink.\n');
        fprintf('  Please update main_system.slx to output f_d_interp.\n');
        has_fd_interp = false;

        % Apply delay compensation to f_d as fallback
        if strcmpi(generation_mode, 'hardware')
            Ts_pos = 1 / pos_update_rate;  % 625 µs @ 1600 Hz
            t_delayed = t - Ts_pos;
            t_delayed(t_delayed < 0) = 0;
            f_d_interp = interp1(t, f_d, t_delayed, 'linear', 'extrap');
            fprintf('  f_d delay applied: %.1f µs (1T @ %d Hz)\n', Ts_pos*1e6, pos_update_rate);
        else
            f_d_interp = f_d;
        end
    end
    fprintf('  f_d range: [%.4f, %.4f] pN\n', min(f_d(:)), max(f_d(:)));

else
    fprintf('【Ideal Tracking Mode】\n');
    fprintf('────────────────────────\n');

    if strcmpi(generation_mode, 'hardware')
        % ─────────────────────────────────────────────────────────────────────
        % Hardware Mode: 1600 Hz sampling + interpolation
        % This tests the non-linear interpolation error without controller dynamics
        % ─────────────────────────────────────────────────────────────────────
        fprintf('  Generation mode: HARDWARE (1600 Hz + %s interpolation)\n', interp_method);

        % Step 1: Create 1600 Hz time grid
        Ts_1600 = 1 / pos_update_rate;
        t_1600 = (0:Ts_1600:sim_time)';
        N_1600 = length(t_1600);
        fprintf('  1600 Hz points: %d (downsample ratio: %.1f)\n', N_1600, N/N_1600);

        % Step 2: Downsample f_d to 1600 Hz
        f_d_1600 = interp1(t, f_d, t_1600, 'linear');

        % Step 3: Interpolate f_d back to 100 kHz FIRST
        % This is the new approach: interpolate f_d, then compute vd
        fprintf('  Interpolating f_d to 100 kHz (%s)...', interp_method);
        tic;
        f_d_interp = zeros(N, 3);
        if strcmpi(interp_method, 'previous')
            for ax = 1:3
                f_d_interp(:, ax) = interp1(t_1600, f_d_1600(:, ax), t, 'previous', 'extrap');
            end
        else
            for ax = 1:3
                f_d_interp(:, ax) = interp1(t_1600, f_d_1600(:, ax), t, 'linear', 'extrap');
            end
        end
        fprintf(' Done (%.2f sec)\n', toc);

        % Step 4: Compute vd at 100 kHz using inverse_model(f_d_interp)
        % New approach: compute vd from interpolated f_d (no non-linear interp error)
        fprintf('  Computing vd @ 100 kHz from f_d_interp...');
        tic;
        vd = zeros(N, 6);
        for i = 1:N
            vd(i, :) = inverse_model(f_d_interp(i, :)', bead_position, inv_params)';
        end
        fprintf(' Done (%.2f sec)\n', toc);

        % Also compute vd_1600 for visualization (1600 Hz sample points)
        vd_1600 = zeros(N_1600, 6);
        for i = 1:N_1600
            vd_1600(i, :) = inverse_model(f_d_1600(i, :)', bead_position, inv_params)';
        end

    else
        % ─────────────────────────────────────────────────────────────────────
        % Ideal Mode: Direct 100 kHz computation (no interpolation)
        % ─────────────────────────────────────────────────────────────────────
        fprintf('  Generation mode: IDEAL (Direct 100 kHz)\n');

        % Compute vd offline using inverse_model at every 100 kHz point
        fprintf('  Computing vd @ 100 kHz...');
        tic;
        vd = zeros(N, 6);
        for i = 1:N
            vd(i, :) = inverse_model(f_d(i, :)', bead_position, inv_params)';
        end
        fprintf(' Done (%.2f sec)\n', toc);
    end

    % Common: Assume perfect tracking (vm = vd)
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

    fprintf('  vd range: [%.4f, %.4f] V\n', min(vd(:)), max(vd(:)));
    fprintf('  f_m range: [%.4f, %.4f] pN\n', min(f_m(:)), max(f_m(:)));
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

% Compute analysis window (use signal_type_original to avoid Simulink overwrite)
if strcmpi(signal_type_original, 'sine')
    T_period = 1 / force_frequency;
    idx_start = round(skip_cycles * T_period / Ts) + 1;
    idx_display = round((total_cycles - display_cycles) * T_period / Ts) + 1;
else
    idx_start = round(step_time / Ts) + 1;
    idx_display = 1;
end

% Force error: use f_d_interp for Simulink mode (actual input to inverse model)
if USE_SIMULINK
    force_error = f_d_interp - f_m;
else
    % Ideal Tracking: use f_d_interp in hardware mode, f_d in ideal mode
    if strcmpi(generation_mode, 'hardware')
        force_error = f_d_interp - f_m;
    else
        force_error = f_d - f_m;
    end
end
force_error_rms = rms(force_error(idx_start:end, :));
force_error_max = max(abs(force_error(idx_start:end, :)));

% Max error as percentage of amplitude (per direction)
force_error_max_pct = force_error_max / force_amplitude * 100;

fprintf('  Force Error (RMS): [%.4f, %.4f, %.4f] pN\n', force_error_rms);
fprintf('  Force Error (Max): [%.4f, %.4f, %.4f] pN\n', force_error_max);
fprintf('  Force Error (Max %%): [%.2f%%, %.2f%%, %.2f%%] of amplitude\n', ...
    force_error_max_pct(1), force_error_max_pct(2), force_error_max_pct(3));

% Relative error (RMS-based, for overall summary)
if force_amplitude > 0
    relative_error = norm(force_error_rms) / force_amplitude * 100;
    fprintf('  Relative Error (RMS): %.2f%%\n', relative_error);
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

    % Prepare 1600 Hz sample points for hardware mode visualization
    if strcmpi(generation_mode, 'hardware') && ~USE_SIMULINK
        % Find 1600 Hz points within display range (t_1600 and f_d_1600 from Section 4)
        t_display_start = t(idx_display);
        t_display_end = t(end);
        idx_1600_display = (t_1600 >= t_display_start) & (t_1600 <= t_display_end);
        t_1600_display = t_1600(idx_1600_display);
        fd_1600_display = f_d_1600(idx_1600_display, :);
    end

    for ax_idx = 1:3
        ax = nexttile(tl1);
        hold(ax, 'on');

        if USE_SIMULINK
            % Simulink mode: Show f_d_interp from Simulink
            plot(ax, t(idx_display:end)*1000, f_d_interp(idx_display:end, ax_idx), 'b-', ...
                'LineWidth', measurement_linewidth, 'DisplayName', 'f_d\_interp (Simulink)');
            legend_entries = {'f_d\_interp', 'f_m'};
        elseif strcmpi(generation_mode, 'hardware')
            % Ideal Tracking + Hardware mode: Show interpolated fd and 1600 Hz samples
            % 1. Interpolated fd (100 kHz from 1600 Hz) - thin blue line
            plot(ax, t(idx_display:end)*1000, f_d_interp(idx_display:end, ax_idx), 'b-', ...
                'LineWidth', 1.5, 'DisplayName', 'f_d (interp from 1600)');
            % 2. 1600 Hz sample points - large blue markers
            plot(ax, t_1600_display*1000, fd_1600_display(:, ax_idx), 'bo', ...
                'MarkerSize', 10, 'MarkerFaceColor', 'b', 'LineWidth', 1.5, ...
                'DisplayName', 'f_d (1600 Hz samples)');
            legend_entries = {'f_d (interp)', 'f_d (1600 Hz)', 'f_m'};
        else
            % Ideal mode: Show fd as solid line
            plot(ax, t(idx_display:end)*1000, f_d(idx_display:end, ax_idx), 'b-', ...
                'LineWidth', measurement_linewidth, 'DisplayName', 'f_d (desired)');
            legend_entries = {'f_d (desired)', 'f_m (estimated)'};
        end

        % f_m (estimated) - always dashed red
        plot(ax, t(idx_display:end)*1000, f_m(idx_display:end, ax_idx), 'r--', ...
            'LineWidth', reference_linewidth, 'DisplayName', 'f_m (estimated)');

        xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
        ylabel(ax, sprintf('%s [pN]', labels{ax_idx}), 'FontSize', ylabel_fontsize);
        legend(ax, legend_entries, 'Location', 'northeast', 'FontSize', legend_fontsize);
        title(ax, sprintf('%s: RMS=%.4f pN (%.2f%%), Max=%.4f pN (%.2f%%)', ...
            labels{ax_idx}, force_error_rms(ax_idx), per_dir_error_pct(ax_idx), ...
            force_error_max(ax_idx), force_error_max_pct(ax_idx)), ...
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
    % Tab 2.5: Vd Spectrum (only for sine mode)
    % FFT analysis of vd, normalized to fundamental frequency
    % ═══════════════════════════════════════════════════════════════════════
    if strcmpi(signal_type_original, 'sine')
        tab_spectrum = uitab(tabgroup, 'Title', 'Vd Spectrum');
        tab_handles.vd_spectrum = tab_spectrum;

        % Use steady-state data for FFT
        vd_steady = vd(idx_start:end, :);
        N_fft = size(vd_steady, 1);
        fs = 1 / Ts;  % 100 kHz

        % Compute FFT for each channel
        freq_axis = (0:N_fft-1) * fs / N_fft;
        half_N = floor(N_fft / 2);
        freq_half = freq_axis(1:half_N);

        % Find fundamental frequency bin
        [~, fund_idx] = min(abs(freq_half - force_frequency));

        % Create 3x2 layout
        tl_spectrum = tiledlayout(tab_spectrum, 3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl_spectrum, 'Vd Spectrum', 'FontWeight', 'bold', 'FontSize', title_fontsize);

        for ch = 1:6
            ax = nexttile(tl_spectrum);

            % Compute FFT magnitude
            Y = fft(vd_steady(:, ch));
            mag = abs(Y(1:half_N)) / N_fft * 2;  % Single-sided amplitude

            % Get fundamental amplitude for normalization (in mV)
            fund_amp_V = mag(fund_idx);
            fund_amp_mV = fund_amp_V * 1000;

            % Normalize to fundamental
            mag_norm = mag / fund_amp_V;

            % Plot with black color, log X axis
            plot(ax, freq_half, mag_norm, 'k-', 'LineWidth', 1.5);

            % Set log scale and limits
            set(ax, 'XScale', 'log');
            xlim(ax, [10, 10000]);
            ylim(ax, [0, 1.1]);

            % Labels - no Y axis label
            xlabel(ax, 'Frequency [Hz]', 'FontSize', xlabel_fontsize);

            % Title with channel and fundamental amplitude in mV
            title(ax, sprintf('P%d (%.1f)', ch, fund_amp_mV), ...
                'FontSize', title_fontsize-2, 'FontWeight', 'bold');

            ax.FontSize = tick_fontsize;
            ax.LineWidth = axis_linewidth;
            grid(ax, 'on');
            box(ax, 'on');
        end

        fprintf('  Tab 2.5: Vd Spectrum (6 channels)\n');

        % ═══════════════════════════════════════════════════════════════════════
        % Tab 2.6: Fm Spectrum (3-axis force)
        % FFT analysis of f_m, normalized to fundamental frequency
        % Non-excited axes use auto-scaled Y-axis to show crosstalk spectrum
        % ═══════════════════════════════════════════════════════════════════════
        tab_fm_spectrum = uitab(tabgroup, 'Title', 'Fm Spectrum');
        tab_handles.fm_spectrum = tab_fm_spectrum;

        % Use steady-state data for FFT
        fm_steady = f_m(idx_start:end, :);
        N_fft_fm = size(fm_steady, 1);

        % Compute FFT for each axis
        freq_axis_fm = (0:N_fft_fm-1) * fs / N_fft_fm;
        half_N_fm = floor(N_fft_fm / 2);
        freq_half_fm = freq_axis_fm(1:half_N_fm);

        % Find fundamental frequency bin
        [~, fund_idx_fm] = min(abs(freq_half_fm - force_frequency));

        % Determine excited axis (largest component in force_direction)
        [~, excited_axis] = max(abs(force_direction));

        % Create 3x1 layout
        tl_fm_spectrum = tiledlayout(tab_fm_spectrum, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl_fm_spectrum, 'Fm Spectrum (Force)', 'FontWeight', 'bold', 'FontSize', title_fontsize);

        axis_labels_force = {'Fx', 'Fy', 'Fz'};
        for ax_idx = 1:3
            ax = nexttile(tl_fm_spectrum);

            % Compute FFT magnitude
            Y_fm = fft(fm_steady(:, ax_idx));
            mag_fm = abs(Y_fm(1:half_N_fm)) / N_fft_fm * 2;  % Single-sided amplitude

            % Get fundamental amplitude (in pN)
            fund_amp_pN = mag_fm(fund_idx_fm);

            % Compute THD (harmonics 2-10)
            thd_sum_sq = 0;
            for h = 2:10
                harm_freq = h * force_frequency;
                [~, harm_idx] = min(abs(freq_half_fm - harm_freq));
                if harm_idx <= half_N_fm
                    thd_sum_sq = thd_sum_sq + mag_fm(harm_idx)^2;
                end
            end
            thd_pct = sqrt(thd_sum_sq) / fund_amp_pN * 100;

            if ax_idx == excited_axis
                % Excited axis: normalize to fundamental, fixed Y-axis
                mag_norm_fm = mag_fm / fund_amp_pN;
                plot(ax, freq_half_fm, mag_norm_fm, 'k-', 'LineWidth', 1.5);
                set(ax, 'XScale', 'log');
                xlim(ax, [10, 10000]);
                ylim(ax, [0, 1.1]);
                title(ax, sprintf('%s (%.3f pN, THD=%.2f%%)', axis_labels_force{ax_idx}, fund_amp_pN, thd_pct), ...
                    'FontSize', title_fontsize-2, 'FontWeight', 'bold');
            else
                % Non-excited axis: show absolute amplitude in pN, auto Y-axis
                plot(ax, freq_half_fm, mag_fm, 'k-', 'LineWidth', 1.5);
                set(ax, 'XScale', 'log');
                xlim(ax, [10, 10000]);
                % Auto Y-axis for visibility
                ylabel(ax, '[pN]', 'FontSize', ylabel_fontsize);
                title(ax, sprintf('%s (%.4f pN, THD=%.2f%%)', axis_labels_force{ax_idx}, fund_amp_pN, thd_pct), ...
                    'FontSize', title_fontsize-2, 'FontWeight', 'bold');
            end

            xlabel(ax, 'Frequency [Hz]', 'FontSize', xlabel_fontsize);
            ax.FontSize = tick_fontsize;
            ax.LineWidth = axis_linewidth;
            grid(ax, 'on');
            box(ax, 'on');
        end

        fprintf('  Tab 2.6: Fm Spectrum (3 axes)\n');
    end

    % ═══════════════════════════════════════════════════════════════════════
    % Tab 2.6: Interpolation Detail (only for hardware mode + Ideal Tracking)
    % Shows 1-2 periods of 1600 Hz with all 100 kHz interpolation points
    % ═══════════════════════════════════════════════════════════════════════
    if strcmpi(generation_mode, 'hardware') && ~USE_SIMULINK
        tab_interp = uitab(tabgroup, 'Title', 'Interp Detail');
        tab_handles.interpolation_detail = tab_interp;

        % Calculate time range for 2 periods of 1600 Hz
        Ts_1600_period = 1 / pos_update_rate;  % ~625 µs
        num_periods_to_show = 2;

        % Start from a stable point (after skip_cycles)
        if strcmpi(signal_type_original, 'sine')
            T_period = 1 / force_frequency;
            t_detail_start = skip_cycles * T_period;
        else
            t_detail_start = step_time + 0.01;  % 10 ms after step
        end
        t_detail_end = t_detail_start + num_periods_to_show * Ts_1600_period;

        % Get indices for detail view
        idx_detail_100k = (t >= t_detail_start) & (t <= t_detail_end);
        idx_detail_1600 = (t_1600 >= t_detail_start) & (t_1600 <= t_detail_end);

        t_detail_100k = t(idx_detail_100k);
        t_detail_1600 = t_1600(idx_detail_1600);

        % Calculate max error ratio for each direction (using full steady-state data)
        interp_error = f_d_interp(idx_start:end, :) - f_m(idx_start:end, :);
        interp_error_max = max(abs(interp_error));
        interp_error_max_pct = interp_error_max / force_amplitude * 100;

        % Also calculate for detail window
        interp_error_detail = f_d_interp(idx_detail_100k, :) - f_m(idx_detail_100k, :);
        interp_error_detail_max = max(abs(interp_error_detail));
        interp_error_detail_max_pct = interp_error_detail_max / force_amplitude * 100;

        % Create 3x1 layout for Fx, Fy, Fz
        tl_interp = tiledlayout(tab_interp, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl_interp, sprintf('Interpolation Detail: %.0f periods @ 1600 Hz (%.1f ms) | Max Error: [%.2f%%, %.2f%%, %.2f%%]', ...
            num_periods_to_show, (t_detail_end - t_detail_start)*1000, ...
            interp_error_max_pct(1), interp_error_max_pct(2), interp_error_max_pct(3)), ...
            'FontWeight', 'bold', 'FontSize', title_fontsize);

        for ax_idx = 1:3
            ax = nexttile(tl_interp);
            hold(ax, 'on');

            % 1. 100 kHz interpolated points (small blue dots)
            plot(ax, t_detail_100k*1000, f_d_interp(idx_detail_100k, ax_idx), 'b.', ...
                'MarkerSize', 8, 'DisplayName', 'f_d (100 kHz interp)');

            % 2. 1600 Hz sample points (large blue circles)
            plot(ax, t_detail_1600*1000, f_d_1600(idx_detail_1600, ax_idx), 'bo', ...
                'MarkerSize', 14, 'MarkerFaceColor', 'b', 'LineWidth', 2, ...
                'DisplayName', 'f_d (1600 Hz samples)');

            % 3. fm points (small red dots)
            plot(ax, t_detail_100k*1000, f_m(idx_detail_100k, ax_idx), 'r.', ...
                'MarkerSize', 8, 'DisplayName', 'f_m (100 kHz)');

            % Add vertical lines at 1600 Hz boundaries
            for t_boundary = t_detail_1600'
                xline(ax, t_boundary*1000, 'k:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
            end

            xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
            ylabel(ax, sprintf('%s [pN]', labels{ax_idx}), 'FontSize', ylabel_fontsize);
            legend(ax, 'Location', 'best', 'FontSize', legend_fontsize);
            title(ax, sprintf('%s: Max Error = %.4f pN (%.2f%% of amplitude)', ...
                labels{ax_idx}, interp_error_max(ax_idx), interp_error_max_pct(ax_idx)), ...
                'FontSize', title_fontsize, 'FontWeight', 'bold');
            ax.FontSize = tick_fontsize;
            ax.LineWidth = axis_linewidth;
            grid(ax, 'on');
            box(ax, 'on');
        end

        % Print error summary to console
        fprintf('  Interpolation Error (fd_interp - fm):\n');
        fprintf('    Max Error: [%.4f, %.4f, %.4f] pN\n', interp_error_max);
        fprintf('    Max Error %%: [%.2f%%, %.2f%%, %.2f%%] of amplitude\n', ...
            interp_error_max_pct(1), interp_error_max_pct(2), interp_error_max_pct(3));

        fprintf('  Tab 2.6: Interpolation Detail (%.0f periods @ 1600 Hz)\n', num_periods_to_show);
    end

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
        if ControllerType == 1  % Model Based Control
            u_ts = simOut.get('u');
        else  % PI-Controller
            u_ts = simOut.get('u_pi');
        end

        % Handle timeseries or array format
        if isa(u_ts, 'timeseries')
            u_data_raw = u_ts.Data;
            t_u = u_ts.Time;
        else
            u_data_raw = u_ts;
            t_u = (0:size(u_data_raw, 1)-1)' * Ts;
        end

        % Ensure u_data_raw has 6 columns (handle different output formats)
        num_cols = size(u_data_raw, 2);
        if num_cols < 6
            fprintf('  Warning: u_data has %d columns (expected 6), padding with zeros\n', num_cols);
            u_data_padded = zeros(size(u_data_raw, 1), 6);
            u_data_padded(:, 1:num_cols) = u_data_raw;
            u_data_raw = u_data_padded;
        end

        % Resample u_data to match t if needed
        N_u = size(u_data_raw, 1);
        if N_u ~= N
            u_data_voltage = zeros(N, 6);
            for ch = 1:6
                u_data_voltage(:, ch) = interp1(t_u, u_data_raw(:, ch), t, 'linear', 'extrap');
            end
        else
            u_data_voltage = u_data_raw;
        end

        % Convert voltage to current using K_A_DIAG
        u_data = u_data_voltage .* K_A_DIAG;  % Now in Amperes

        tl5 = tiledlayout(tab5, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        if ControllerType == 1
            ctrl_param_str = sprintf('Model Based Control (fB: f=%d, c=%d, e=%d Hz)', fB_f, fB_c, fB_e);
        else
            ctrl_param_str = sprintf('PI-Controller (Kp=%.1f, Ki=%.1f)', Kp_value, Ki_value);
        end
        title(tl5, sprintf('Control Input u (Current) - %s', ctrl_param_str), ...
            'FontWeight', 'bold', 'FontSize', title_fontsize);

        for ch = 1:6
            ax = nexttile(tl5);
            % Display in A (Amperes)
            u_display = u_data(idx_display:end, ch);
            plot(ax, t(idx_display:end)*1000, u_display, ...
                'Color', colors(ch,:), 'LineWidth', measurement_linewidth);
            xlabel(ax, 'Time [ms]', 'FontSize', xlabel_fontsize);
            ylabel(ax, 'u [A]', 'FontSize', ylabel_fontsize);
            title(ax, sprintf('P%d (Range: %.4f ~ %.4f A)', ch, ...
                min(u_display), max(u_display)), ...
                'FontSize', title_fontsize-2, 'FontWeight', 'bold');
            ax.FontSize = tick_fontsize;
            ax.LineWidth = axis_linewidth;
            grid(ax, 'on');
            box(ax, 'on');
        end
        fprintf('  Tab 5: Control Input (u) - Current [A]\n');

        % ───────────────────────────────────────────────────────────────────
        % Tab 5.5: u Spectrum (6-channel current)
        % FFT analysis of control input current, normalized to fundamental
        % Only for sine mode
        % ───────────────────────────────────────────────────────────────────
        if strcmpi(signal_type_original, 'sine')
            tab_u_spectrum = uitab(tabgroup, 'Title', 'u Spectrum');
            tab_handles.u_spectrum = tab_u_spectrum;

            % Use steady-state data for FFT (u_data is already in Amperes)
            u_steady = u_data(idx_start:end, :);
            N_fft_u = size(u_steady, 1);
            fs_u = 1 / Ts;  % 100 kHz sampling rate

            % Compute FFT parameters
            freq_axis_u = (0:N_fft_u-1) * fs_u / N_fft_u;
            half_N_u = floor(N_fft_u / 2);
            freq_half_u = freq_axis_u(1:half_N_u);

            % Find fundamental frequency bin
            [~, fund_idx_u] = min(abs(freq_half_u - force_frequency));

            % Create 2x3 layout
            tl_u_spectrum = tiledlayout(tab_u_spectrum, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
            title(tl_u_spectrum, 'u Spectrum (Current)', 'FontWeight', 'bold', 'FontSize', title_fontsize);

            for ch = 1:6
                ax = nexttile(tl_u_spectrum);

                % Compute FFT magnitude
                Y_u = fft(u_steady(:, ch));
                mag_u = abs(Y_u(1:half_N_u)) / N_fft_u * 2;  % Single-sided amplitude [A]

                % Get fundamental amplitude (in A)
                fund_amp_A = mag_u(fund_idx_u);

                % Normalize to fundamental
                if fund_amp_A > 1e-6  % Avoid division by zero
                    mag_norm_u = mag_u / fund_amp_A;
                else
                    mag_norm_u = mag_u;  % Just show raw if no fundamental
                end

                % Plot with channel color, log X axis
                plot(ax, freq_half_u, mag_norm_u, 'Color', colors(ch,:), 'LineWidth', 1.5);

                % Set log scale and limits (extend to 20kHz)
                set(ax, 'XScale', 'log');
                xlim(ax, [10, 20000]);
                ylim(ax, [0, 1.1]);

                % Labels
                xlabel(ax, 'Frequency [Hz]', 'FontSize', xlabel_fontsize);

                % Title with channel and fundamental amplitude in A
                title(ax, sprintf('P%d (%.4f A)', ch, fund_amp_A), ...
                    'FontSize', title_fontsize-2, 'FontWeight', 'bold');

                ax.FontSize = tick_fontsize;
                ax.LineWidth = axis_linewidth;
                grid(ax, 'on');
                box(ax, 'on');
            end

            fprintf('  Tab 5.5: u Spectrum (6 channels) - Current [A]\n');
        end

        % ───────────────────────────────────────────────────────────────────
        % Tab 6: Vm vs Vd Tracking (Lissajous-style plot)
        %        X: Vd, Y: Vm, 6 channels overlaid, 3 cycles
        %        Style: matching reference (analyze_invmodel_test.m Tab 8)
        % ───────────────────────────────────────────────────────────────────
        tab6 = uitab(tabgroup, 'Title', 'Vm vs Vd');
        tab_handles.vd_vs_vm = tab6;

        % Calculate cycle window indices (from steady state)
        cycles_for_plot = 3;  % Overlay 3 cycles
        if strcmpi(signal_type_original, 'sine')
            T_period = 1 / force_frequency;
            % Start from skip_cycles, take cycles_for_plot cycles
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
                'Color', colors_6ch(ch,:), 'LineWidth', 4.0);
        end

        % Add reference line (y = x) as black dashed
        all_voltage = [vd(idx_plot_start:idx_plot_end, :); vm(idx_plot_start:idx_plot_end, :)];
        lims_v = [min(all_voltage(:)), max(all_voltage(:))];
        plot(ax6, lims_v, lims_v, 'k--', 'LineWidth', 2.5);

        % Title with frequency (enlarged, bold)
        title(ax6, sprintf('Vm vs Vd (Freq: %.1f Hz)', force_frequency), ...
            'FontSize', 26, 'FontWeight', 'bold');

        % Set axis labels (enlarged, bold)
        xlabel(ax6, 'Vd [V]', 'FontSize', 26, 'FontWeight', 'bold');
        ylabel(ax6, 'Vm [V]', 'FontSize', 26, 'FontWeight', 'bold');

        % Axis properties (enlarged, bold)
        ax6.FontSize = 22;
        ax6.FontWeight = 'bold';
        ax6.LineWidth = 2.5;
        ax6.Box = 'on';
        axis(ax6, 'equal');
        grid(ax6, 'on');

        % Legend at southeast (enlarged, bold)
        legend(ax6, plot_handles, channel_labels, ...
            'Location', 'southeast', ...
            'FontSize', 16, 'FontWeight', 'bold');

        fprintf('  Tab 6: Vm vs Vd Tracking (%d cycles)\n', cycles_for_plot);

        % ───────────────────────────────────────────────────────────────────
        % Tab 7: fm vs f_d_interp Tracking
        %        X: f_d_interp (from Simulink, excited direction)
        %        Y: fm (all 3 directions)
        %        This shows pure controller tracking without inverse model error
        % ───────────────────────────────────────────────────────────────────
        tab7 = uitab(tabgroup, 'Title', 'fm vs fd\_interp');
        tab_handles.fd_vs_fm = tab7;

        % Find the excited direction index (the one with largest component in force_direction)
        [~, excited_axis] = max(abs(force_direction));
        force_labels_short = {'X', 'Y', 'Z'};
        force_colors = [0.0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880];

        % Extract data for plotting (using f_d_interp from Simulink)
        idx_one_cycle = idx_plot_start:idx_plot_end;
        fd_excited = f_d_interp(idx_one_cycle, excited_axis);  % f_d_interp from Simulink
        fm_all = f_m(idx_one_cycle, :);  % All 3 directions

        % Create single axes for the plot
        ax7 = uiaxes(tab7);
        ax7.Position = [100 80 1200 750];
        hold(ax7, 'on');

        % Plot fm (all 3 directions) vs fd (excited) as single LINE
        % One continuous line per axis showing the Lissajous trajectory
        plot_handles_7 = gobjects(1, 3);
        for i = 1:3
            plot_handles_7(i) = plot(ax7, fd_excited, fm_all(:, i), ...
                '-', 'Color', force_colors(i,:), 'LineWidth', 3.5);
        end

        % Add ideal reference line (y=x) as black dashed
        axis_limit = force_amplitude * 1.05;
        plot(ax7, [-axis_limit, axis_limit], [-axis_limit, axis_limit], ...
            'k--', 'LineWidth', 2.5);

        % Set axis limits
        xlim(ax7, [-axis_limit, axis_limit]);
        ylim(ax7, [-axis_limit, axis_limit]);

        % Title (enlarged, bold)
        title(ax7, sprintf('fm vs fd\\_interp (Freq: %.1f Hz)', force_frequency), ...
            'FontSize', 26, 'FontWeight', 'bold');

        % Set axis labels (enlarged, bold)
        xlabel(ax7, 'fd\\_interp [pN]', 'FontSize', 26, 'FontWeight', 'bold');
        ylabel(ax7, 'fm [pN]', 'FontSize', 26, 'FontWeight', 'bold');

        % Axis properties (enlarged, bold)
        ax7.FontSize = 22;
        ax7.FontWeight = 'bold';
        ax7.LineWidth = 2.5;
        ax7.XColor = 'k';
        ax7.YColor = 'k';
        ax7.GridColor = [0.8 0.8 0.8];
        ax7.GridAlpha = 1;
        grid(ax7, 'on');
        box(ax7, 'on');

        % Legend at bottom right (enlarged, bold)
        legend(ax7, plot_handles_7, force_labels_short, ...
            'Location', 'southeast', ...
            'FontSize', 16, 'FontWeight', 'bold');

        fprintf('  Tab 7: fm vs fd_interp Tracking (Freq: %.1f Hz)\n', force_frequency);

        % ───────────────────────────────────────────────────────────────────
        % Tab 8: Vm[k] vs Vd[k-2] (Delayed comparison)
        %        Compare with Tab 6 to see effect of 2-step delay compensation
        % ───────────────────────────────────────────────────────────────────
        tab8 = uitab(tabgroup, 'Title', 'Vm vs Vd (Delayed)');
        tab_handles.vd_vs_vm_delayed = tab8;

        % Delay parameters
        delay_samples = 2;  % 2 steps @ 100kHz = 20µs
        delay_us = delay_samples * Ts * 1e6;

        % Adjust plot range to avoid boundary issues
        idx_delayed_start = max(idx_plot_start, 1 + delay_samples);
        idx_delayed_end = min(idx_plot_end, N);

        % Extract delayed data: Vm[k] vs Vd[k-2]
        % X-axis: Vd[k-2] (vd from 2 steps earlier)
        % Y-axis: Vm[k] (current vm)
        vd_for_delayed = vd(idx_delayed_start - delay_samples : idx_delayed_end - delay_samples, :);
        vm_for_delayed = vm(idx_delayed_start : idx_delayed_end, :);

        % Create single axes for overlaid plot
        ax8 = uiaxes(tab8);
        ax8.Position = [100 80 1200 750];
        hold(ax8, 'on');

        % Plot 6 channels: X=Vd[k-2], Y=Vm[k]
        plot_handles_8 = gobjects(1, 6);
        for ch = 1:6
            plot_handles_8(ch) = plot(ax8, ...
                vd_for_delayed(:, ch), ...
                vm_for_delayed(:, ch), ...
                'Color', colors_6ch(ch,:), 'LineWidth', 4.0);
        end

        % Add reference line (y = x) as black dashed
        all_voltage_8 = [vd_for_delayed; vm_for_delayed];
        lims_v8 = [min(all_voltage_8(:)), max(all_voltage_8(:))];
        plot(ax8, lims_v8, lims_v8, 'k--', 'LineWidth', 2.5);

        % Title (enlarged, bold)
        title(ax8, sprintf('Vm[k] vs Vd[k-2] (Freq: %.1f Hz)', force_frequency), ...
            'FontSize', 26, 'FontWeight', 'bold');

        % Set axis labels (enlarged, bold)
        xlabel(ax8, 'Vd[k-2] [V]', 'FontSize', 26, 'FontWeight', 'bold');
        ylabel(ax8, 'Vm[k] [V]', 'FontSize', 26, 'FontWeight', 'bold');

        % Axis properties (enlarged, bold)
        ax8.FontSize = 22;
        ax8.FontWeight = 'bold';
        ax8.LineWidth = 2.5;
        ax8.Box = 'on';
        axis(ax8, 'equal');
        grid(ax8, 'on');

        % Legend at southeast (enlarged, bold)
        legend(ax8, plot_handles_8, channel_labels, ...
            'Location', 'southeast', ...
            'FontSize', 16, 'FontWeight', 'bold');

        fprintf('  Tab 8: Vm[k] vs Vd[k-2]\n');

        % ───────────────────────────────────────────────────────────────────
        % Tab 9: Fm[k] vs fd_interp[k-2] (Delayed comparison)
        %        Using f_d_interp from Simulink for accurate comparison
        % ───────────────────────────────────────────────────────────────────
        tab9 = uitab(tabgroup, 'Title', 'fm vs fd\_interp (Delayed)');
        tab_handles.fd_vs_fm_delayed = tab9;

        % Extract delayed data: Fm[k] vs fd_interp[k-2]
        % X-axis: fd_interp[k-2] (f_d_interp from 2 steps earlier)
        % Y-axis: Fm[k] (current f_m)
        fd_for_delayed = f_d_interp(idx_delayed_start - delay_samples : idx_delayed_end - delay_samples, excited_axis);
        fm_for_delayed = f_m(idx_delayed_start : idx_delayed_end, :);

        % Create single axes for the plot
        ax9 = uiaxes(tab9);
        ax9.Position = [100 80 1200 750];
        hold(ax9, 'on');

        % Plot fm (all 3 directions) vs fd (excited, delayed)
        plot_handles_9 = gobjects(1, 3);
        for i = 1:3
            plot_handles_9(i) = plot(ax9, fd_for_delayed, fm_for_delayed(:, i), ...
                '-', 'Color', force_colors(i,:), 'LineWidth', 3.5);
        end

        % Add ideal reference line (y=x) as black dashed
        plot(ax9, [-axis_limit, axis_limit], [-axis_limit, axis_limit], ...
            'k--', 'LineWidth', 2.5);

        % Set axis limits
        xlim(ax9, [-axis_limit, axis_limit]);
        ylim(ax9, [-axis_limit, axis_limit]);

        % Title (enlarged, bold)
        title(ax9, sprintf('Fm[k] vs fd\\_interp[k-2] (Freq: %.1f Hz)', force_frequency), ...
            'FontSize', 26, 'FontWeight', 'bold');

        % Set axis labels (enlarged, bold)
        xlabel(ax9, 'fd\\_interp[k-2] [pN]', 'FontSize', 26, 'FontWeight', 'bold');
        ylabel(ax9, 'Fm[k] [pN]', 'FontSize', 26, 'FontWeight', 'bold');

        % Axis properties (enlarged, bold)
        ax9.FontSize = 22;
        ax9.FontWeight = 'bold';
        ax9.LineWidth = 2.5;
        ax9.XColor = 'k';
        ax9.YColor = 'k';
        ax9.GridColor = [0.8 0.8 0.8];
        ax9.GridAlpha = 1;
        grid(ax9, 'on');
        box(ax9, 'on');

        % Legend at bottom right (enlarged, bold)
        legend(ax9, plot_handles_9, force_labels_short, ...
            'Location', 'southeast', ...
            'FontSize', 16, 'FontWeight', 'bold');

        fprintf('  Tab 9: Fm[k] vs fd_interp[k-2]\n');

    else
        % ═══════════════════════════════════════════════════════════════════════
        % Ideal Tracking Mode: Tab 4 - fd vs fm (Round-trip verification)
        % ═══════════════════════════════════════════════════════════════════════
        tab4 = uitab(tabgroup, 'Title', 'fm vs fd');
        tab_handles.fd_vs_fm = tab4;

        % Find the excited direction index
        [~, excited_axis] = max(abs(force_direction));
        force_labels_short = {'X', 'Y', 'Z'};
        force_colors = [0.0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880];

        % Calculate cycle window indices for plotting
        cycles_for_plot = 3;
        if strcmpi(signal_type_original, 'sine')
            T_period = 1 / force_frequency;
            idx_plot_start = round(skip_cycles * T_period / Ts) + 1;
            idx_plot_end = round((skip_cycles + cycles_for_plot) * T_period / Ts);
            idx_plot_end = min(idx_plot_end, N);
        else
            idx_plot_start = idx_display;
            idx_plot_end = N;
        end

        % Extract data
        idx_one_cycle = idx_plot_start:idx_plot_end;
        % Use f_d_interp in hardware mode for consistent comparison
        if strcmpi(generation_mode, 'hardware')
            fd_excited = f_d_interp(idx_one_cycle, excited_axis);
        else
            fd_excited = f_d(idx_one_cycle, excited_axis);
        end
        fm_all = f_m(idx_one_cycle, :);

        % Create single axes for the plot
        ax4 = uiaxes(tab4);
        ax4.Position = [100 80 1200 750];
        hold(ax4, 'on');

        % Plot fm (all 3 directions) vs fd (excited)
        plot_handles_4 = gobjects(1, 3);
        for i = 1:3
            plot_handles_4(i) = plot(ax4, fd_excited, fm_all(:, i), ...
                '-', 'Color', force_colors(i,:), 'LineWidth', 3.5);
        end

        % Add ideal reference line (y=x)
        axis_limit = force_amplitude * 1.05;
        plot(ax4, [-axis_limit, axis_limit], [-axis_limit, axis_limit], ...
            'k--', 'LineWidth', 2.5);

        % Set axis limits
        xlim(ax4, [-axis_limit, axis_limit]);
        ylim(ax4, [-axis_limit, axis_limit]);

        % Title (enlarged, bold)
        if strcmpi(generation_mode, 'hardware')
            title_str = sprintf('fm vs fd_{interp} - Hardware Mode (Freq: %.1f Hz)', force_frequency);
        else
            title_str = sprintf('fm vs fd - Ideal Mode (Freq: %.1f Hz)', force_frequency);
        end
        title(ax4, title_str, ...
            'FontSize', 26, 'FontWeight', 'bold');

        % Set axis labels (enlarged, bold)
        if strcmpi(generation_mode, 'hardware')
            xlabel(ax4, 'fd_{interp} [pN]', 'FontSize', 26, 'FontWeight', 'bold');
        else
            xlabel(ax4, 'fd [pN]', 'FontSize', 26, 'FontWeight', 'bold');
        end
        ylabel(ax4, 'fm [pN]', 'FontSize', 26, 'FontWeight', 'bold');

        % Axis properties (enlarged, bold)
        ax4.FontSize = 22;
        ax4.FontWeight = 'bold';
        ax4.LineWidth = 2.5;
        ax4.XColor = 'k';
        ax4.YColor = 'k';
        ax4.GridColor = [0.8 0.8 0.8];
        ax4.GridAlpha = 1;
        grid(ax4, 'on');
        box(ax4, 'on');

        % Legend at bottom right (enlarged, bold)
        legend(ax4, plot_handles_4, force_labels_short, ...
            'Location', 'southeast', ...
            'FontSize', 16, 'FontWeight', 'bold');

        fprintf('  Tab 4: fm vs fd - Ideal Tracking (%d cycles)\n', cycles_for_plot);
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

                % Tab 2.5: Vd Spectrum (only for sine mode)
                if strcmpi(signal_type_original, 'sine') && isfield(tab_handles, 'vd_spectrum')
                    tabgroup.SelectedTab = tab_spectrum;
                    drawnow; pause(0.1);
                    exportgraphics(tab_spectrum, fullfile(test_dir, 'tab2_5_vd_spectrum.png'), 'Resolution', export_resolution);
                end

                % Tab 2.6: Fm Spectrum (only for sine mode)
                if strcmpi(signal_type_original, 'sine') && isfield(tab_handles, 'fm_spectrum')
                    tabgroup.SelectedTab = tab_fm_spectrum;
                    drawnow; pause(0.1);
                    exportgraphics(tab_fm_spectrum, fullfile(test_dir, 'tab2_6_fm_spectrum.png'), 'Resolution', export_resolution);
                end

                % Tab 3: Error Analysis
                tabgroup.SelectedTab = tab3;
                drawnow; pause(0.1);
                exportgraphics(tab3, fullfile(test_dir, 'tab3_error_analysis.png'), 'Resolution', export_resolution);

                tab_count = 3;
                if strcmpi(signal_type_original, 'sine')
                    tab_count = tab_count + 2;  % Vd Spectrum + Fm Spectrum
                end

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

                    % Tab 5.5: u Spectrum (only for sine mode)
                    if strcmpi(signal_type_original, 'sine') && isfield(tab_handles, 'u_spectrum')
                        tabgroup.SelectedTab = tab_u_spectrum;
                        drawnow; pause(0.1);
                        exportgraphics(tab_u_spectrum, fullfile(test_dir, 'tab5_5_u_spectrum.png'), 'Resolution', export_resolution);
                    end

                    % Tab 6: vd vs vm Tracking
                    tabgroup.SelectedTab = tab6;
                    drawnow; pause(0.1);
                    exportgraphics(tab6, fullfile(test_dir, 'tab6_vd_vs_vm.png'), 'Resolution', export_resolution);

                    % Tab 7: fm vs fd_interp Tracking
                    tabgroup.SelectedTab = tab7;
                    drawnow; pause(0.1);
                    exportgraphics(tab7, fullfile(test_dir, 'tab7_fm_vs_fd_interp.png'), 'Resolution', export_resolution);

                    % Tab 8: Vm vs Vd (Delayed)
                    tabgroup.SelectedTab = tab8;
                    drawnow; pause(0.1);
                    exportgraphics(tab8, fullfile(test_dir, 'tab8_vm_vs_vd_delayed.png'), 'Resolution', export_resolution);

                    % Tab 9: fm vs fd_interp (Delayed)
                    tabgroup.SelectedTab = tab9;
                    drawnow; pause(0.1);
                    exportgraphics(tab9, fullfile(test_dir, 'tab9_fm_vs_fd_interp_delayed.png'), 'Resolution', export_resolution);

                    tab_count = 9;
                    if strcmpi(signal_type_original, 'sine')
                        tab_count = tab_count + 1;  % u Spectrum
                    end
                else
                    % Ideal Tracking mode: Tab 4 (fd vs fm)
                    tabgroup.SelectedTab = tab4;
                    drawnow; pause(0.1);
                    exportgraphics(tab4, fullfile(test_dir, 'tab4_fd_vs_fm.png'), 'Resolution', export_resolution);

                    tab_count = 4;
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
            results.data.f_d_interp = f_d_interp;  % Interpolated f_d from Simulink
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
