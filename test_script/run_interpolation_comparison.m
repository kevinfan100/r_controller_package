% run_interpolation_comparison.m
% Compare interpolation methods (ZOH vs Linear) for vd scheduling
%
% Key points:
%   - ZOH: No extra delay needed (uses latest available sample)
%   - Linear: Needs 1-period delay (requires two samples to interpolate)
%   - Reference: Ideal continuous sine wave (no interpolation delay)

clear; clc; close all;

% Add paths
script_dir = fileparts(mfilename('fullpath'));
package_root = fullfile(script_dir, '..');
addpath(fullfile(package_root, 'model'));


%%                        SECTION 1: Configuration


fprintf('\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('     Interpolation Comparison (ZOH vs Linear)\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');

% ─────────────────────────────────────────────────────────────────────────
% Test Configuration
% ─────────────────────────────────────────────────────────────────────────
interp_methods = {'previous', 'linear'};  % 'previous' = ZOH
interp_labels = {'ZOH', 'Linear'};
n_methods = length(interp_methods);

% Controller
controller_type = 'r_controller';

% Signal
force_direction = [0; 1; 0];
force_amplitude = 1.0;          % [pN]
force_frequency = 10;           % [Hz]
bead_position = [0; 0; 0];

% Timing
pos_update_rate = 1600;         % [Hz]
ctrl_rate = 100000;             % [Hz]

% Simulation
total_cycles = 50;
skip_cycles = 20;
analysis_cycles = 20;           % Integer for clean FFT

sim_time = total_cycles / force_frequency;

% R-Controller parameters
fB_f = 3000; fB_c = 1000; fB_e = 5000;
Kp_value = 2; Ki_value = Kp_value * 2206;

% FFT settings
USE_WINDOW = true;
REMOVE_DC = true;

% Output
USE_SIMULINK = true;
SAVE_PNG = true;
output_dir = fullfile(package_root, 'test_results', 'interpolation_comparison');

% Colors
colors_method = [0.8500, 0.3250, 0.0980;   % ZOH - Orange
                 0.0000, 0.4470, 0.7410];  % Linear - Blue


%%                        SECTION 2: Initialization


fprintf('【Initialization】\n');

% Controller setup
if strcmpi(controller_type, 'r_controller')
    ControllerType = 1;
    controller_label = 'R-Controller';
else
    ControllerType = 2;
    controller_label = 'PI-Controller';
end

% Load parameters
inv_params = system_params();
ctrl_params = r_controller_calc_params(fB_c, fB_e, fB_f);

% Timing
Ts = 1 / ctrl_rate;
Ts_pos = 1 / pos_update_rate;
T_period = 1 / force_frequency;

% Time axes
N_ctrl = round(sim_time / Ts) + 1;
N_pos = round(sim_time / Ts_pos) + 1;
t_ctrl = (0:N_ctrl-1)' * Ts;
t_pos = (0:N_pos-1)' * Ts_pos;

% Force direction
force_direction = force_direction / norm(force_direction);
force_dir_idx = find(force_direction ~= 0, 1);

% Analysis indices (integer cycles for clean FFT)
idx_analysis_start = round(skip_cycles * T_period / Ts) + 1;
idx_analysis_end = idx_analysis_start + round(analysis_cycles * T_period / Ts) - 1;
idx_analysis = idx_analysis_start:idx_analysis_end;
N_analysis = length(idx_analysis);

% FFT setup
df = 1 / (N_analysis * Ts);
freq_axis = (0:N_analysis-1) * df;
[~, fundamental_bin] = min(abs(freq_axis - force_frequency));

fprintf('  Position update: %d Hz (period = %.1f us)\n', pos_update_rate, Ts_pos*1e6);
fprintf('  FFT: %d cycles, resolution %.3f Hz, fundamental bin %d\n', ...
    analysis_cycles, df, fundamental_bin);
fprintf('\n');


%%                        SECTION 3: Generate Signals


fprintf('【Generate Signals】\n');

% ─────────────────────────────────────────────────────────────────────────
% IDEAL reference: Continuous sine wave at 100 kHz (NO interpolation delay)
% ─────────────────────────────────────────────────────────────────────────
f_d_ideal = force_amplitude * sin(2*pi*force_frequency*t_ctrl) .* force_direction';

fprintf('  f_d_ideal: Direct sine wave @ 100 kHz (zero delay reference)\n');

% ─────────────────────────────────────────────────────────────────────────
% f_d at 1600 Hz (sampled version)
% ─────────────────────────────────────────────────────────────────────────
f_d_low = force_amplitude * sin(2*pi*force_frequency*t_pos) .* force_direction';

% ─────────────────────────────────────────────────────────────────────────
% vd at 1600 Hz via inverse_model
% ─────────────────────────────────────────────────────────────────────────
fprintf('  Computing inverse_model @ %d Hz...', pos_update_rate);
vd_low = zeros(N_pos, 6);
for i = 1:N_pos
    vd_low(i, :) = inverse_model(f_d_low(i, :)', bead_position, inv_params)';
end
fprintf(' Done\n');

fprintf('\n');


%%                        SECTION 4: Run Tests


fprintf('【Run Tests】\n');

% Storage
results = struct();
results.vd = cell(n_methods, 1);
results.f_m = cell(n_methods, 1);
results.delay_us = zeros(n_methods, 1);
results.error_rms = zeros(n_methods, 1);

% Simulink setup
if USE_SIMULINK
    assignin('base', 'SignalType', 3);
    assignin('base', 'Channel', 1);
    assignin('base', 'Amplitude', 0);
    assignin('base', 'Frequency', force_frequency);
    assignin('base', 'Phase', 0);
    assignin('base', 'StepTime', 0);
    assignin('base', 'd', 0);
    assignin('base', 'params', ctrl_params);
    assignin('base', 'ControllerType', ControllerType);
    assignin('base', 'Kp_value', Kp_value);
    assignin('base', 'Ki_value', Ki_value);

    model_name = 'r_controller_system_integrated';
    if ~bdIsLoaded(model_name)
        load_system(fullfile(package_root, 'model', [model_name '.slx']));
    end
    set_param(model_name, 'StopTime', num2str(sim_time));
end

% Test each method
for m = 1:n_methods
    method = interp_methods{m};
    label = interp_labels{m};
    fprintf('  %s: ', label);

    % ─────────────────────────────────────────────────────────────────────
    % CORRECTED Causal Interpolation
    % ─────────────────────────────────────────────────────────────────────
    % ZOH: Uses latest available sample, no extra delay needed
    %      At time t, use vd[floor(t/Ts_pos)]
    %      Inherent delay: ~0.5T (holds value until next update)
    %
    % Linear: Needs two samples to interpolate, requires 1-period delay
    %         At time t in [kT, (k+1)T), interpolate between vd[k-1] and vd[k]
    %         Inherent delay: ~1.0T
    % ─────────────────────────────────────────────────────────────────────

    if strcmp(method, 'previous')  % ZOH
        % No extra delay - use current time directly
        t_query = t_ctrl;
        t_query(t_query < 0) = 0;
    else  % Linear
        % Need 1-period delay to have two samples available
        t_query = t_ctrl - Ts_pos;
        t_query(t_query < 0) = 0;
    end

    vd = interp1(t_pos, vd_low, t_query, method, 'extrap');
    results.vd{m} = vd;

    % Simulink
    if USE_SIMULINK
        vd_ts = timeseries(vd, t_ctrl);
        assignin('base', 'vd_timeseries', vd_ts);
        out = sim(model_name);

        vm_sim = out.Vm;
        N_vm = size(vm_sim, 1);
        if N_vm ~= N_ctrl
            vm = interp1((0:N_vm-1)'*Ts, vm_sim, t_ctrl, 'linear', 'extrap');
        else
            vm = vm_sim;
        end
    else
        vm = vd;
    end

    % Force model
    f_m = zeros(N_ctrl, 3);
    for i = 1:N_ctrl
        f_m(i, :) = force_model(vm(i, :)', bead_position, inv_params)';
    end
    results.f_m{m} = f_m;

    % Error
    results.error_rms(m) = rms(f_d_ideal(idx_analysis, force_dir_idx) - f_m(idx_analysis, force_dir_idx));

    fprintf('Done\n');
end

fprintf('\n');


%%                        SECTION 5: FFT Phase Analysis


fprintf('【FFT Phase Analysis】\n');

% ─────────────────────────────────────────────────────────────────────────
% Reference: Ideal sine wave (f_d_ideal) - should have ~0 phase at t=0
% ─────────────────────────────────────────────────────────────────────────
sig_fd = f_d_ideal(idx_analysis, force_dir_idx);
if REMOVE_DC, sig_fd = sig_fd - mean(sig_fd); end
if USE_WINDOW, sig_fd = sig_fd .* hanning(N_analysis); end
fft_fd = fft(sig_fd);
phase_fd = angle(fft_fd(fundamental_bin));

fprintf('  Reference (ideal sine): phase = %.2f deg\n', rad2deg(phase_fd));

% ─────────────────────────────────────────────────────────────────────────
% Phase for each method
% ─────────────────────────────────────────────────────────────────────────
for m = 1:n_methods
    sig_fm = results.f_m{m}(idx_analysis, force_dir_idx);
    if REMOVE_DC, sig_fm = sig_fm - mean(sig_fm); end
    if USE_WINDOW, sig_fm = sig_fm .* hanning(N_analysis); end

    fft_fm = fft(sig_fm);
    phase_fm = angle(fft_fm(fundamental_bin));

    % Phase difference and delay
    phase_diff = wrapToPi(phase_fm - phase_fd);
    results.delay_us(m) = -phase_diff / (2*pi*force_frequency) * 1e6;

    fprintf('  %s: phase = %.2f deg, delay = %.1f us (%.2f T)\n', ...
        interp_labels{m}, rad2deg(phase_fm), results.delay_us(m), ...
        results.delay_us(m) / (Ts_pos*1e6));
end

fprintf('\n');


%%                        SECTION 6: Results Summary


fprintf('════════════════════════════════════════════════════════════\n');
fprintf('                      Results\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

fprintf('┌───────────┬─────────────┬───────────┬─────────────┐\n');
fprintf('│  Method   │  Delay [us] │ Delay [T] │  Error [pN] │\n');
fprintf('├───────────┼─────────────┼───────────┼─────────────┤\n');
for m = 1:n_methods
    delay_T = results.delay_us(m) / (Ts_pos*1e6);
    fprintf('│ %-9s │   %7.1f   │   %5.2f   │    %.4f   │\n', ...
        interp_labels{m}, results.delay_us(m), delay_T, results.error_rms(m));
end
fprintf('└───────────┴─────────────┴───────────┴─────────────┘\n\n');

fprintf('1 position update period (T) = %.1f us\n', Ts_pos*1e6);
fprintf('Delay difference (Linear - ZOH) = %.1f us (%.2f T)\n\n', ...
    results.delay_us(2) - results.delay_us(1), ...
    (results.delay_us(2) - results.delay_us(1)) / (Ts_pos*1e6));

fprintf('Expected delays:\n');
fprintf('  ZOH:    ~0.5T = %.1f us (hold current value)\n', 0.5*Ts_pos*1e6);
fprintf('  Linear: ~1.0T = %.1f us (interpolate from previous)\n', 1.0*Ts_pos*1e6);
fprintf('\n');


%%                        SECTION 7: Plotting


fprintf('【Plotting】\n');

fig = figure('Name', 'Interpolation Comparison', 'Position', [100 100 1200 500]);

% ─────────────────────────────────────────────────────────────────────────
% Left: One Cycle Comparison
% ─────────────────────────────────────────────────────────────────────────
subplot(1, 2, 1);

% One cycle data
one_cycle_samples = round(T_period / Ts);
idx_one = idx_analysis_start:(idx_analysis_start + one_cycle_samples - 1);
t_one = (0:one_cycle_samples-1) * Ts * 1000;  % ms

% Plot
plot(t_one, f_d_ideal(idx_one, force_dir_idx), 'k--', 'LineWidth', 2, ...
    'DisplayName', 'f_d (ideal)');
hold on;
for m = 1:n_methods
    delay_T = results.delay_us(m) / (Ts_pos*1e6);
    plot(t_one, results.f_m{m}(idx_one, force_dir_idx), ...
        'Color', colors_method(m,:), 'LineWidth', 2, ...
        'DisplayName', sprintf('%s (%.2fT = %.0f us)', ...
            interp_labels{m}, delay_T, results.delay_us(m)));
end
hold off;

xlabel('Time [ms]', 'FontSize', 12);
ylabel('Force [pN]', 'FontSize', 12);
title('One Cycle Comparison', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
box on;

% ─────────────────────────────────────────────────────────────────────────
% Right: Delay Bar Chart
% ─────────────────────────────────────────────────────────────────────────
subplot(1, 2, 2);

b = bar(results.delay_us);
b.FaceColor = 'flat';
for m = 1:n_methods
    b.CData(m,:) = colors_method(m,:);
end

% Add theoretical lines
hold on;
yline(0.5*Ts_pos*1e6, 'r--', 'LineWidth', 1.5, 'Label', '0.5T (ZOH expected)');
yline(1.0*Ts_pos*1e6, 'b--', 'LineWidth', 1.5, 'Label', '1.0T (Linear expected)');
hold off;

set(gca, 'XTickLabel', interp_labels);
xlabel('Method', 'FontSize', 12);
ylabel('Delay [us]', 'FontSize', 12);
title(sprintf('Phase Delay vs Ideal Sine (T = %.0f us)', Ts_pos*1e6), ...
    'FontSize', 14, 'FontWeight', 'bold');
grid on;
box on;
ylim([0, max(results.delay_us)*1.3]);

% Add value labels
for m = 1:n_methods
    delay_T = results.delay_us(m) / (Ts_pos*1e6);
    text(m, results.delay_us(m) + max(results.delay_us)*0.05, ...
        sprintf('%.0f us\n(%.2fT)', results.delay_us(m), delay_T), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

sgtitle(sprintf('ZOH vs Linear (%s, %d Hz update)', ...
    controller_label, pos_update_rate), 'FontSize', 14, 'FontWeight', 'bold');

% ─────────────────────────────────────────────────────────────────────────
% Save
% ─────────────────────────────────────────────────────────────────────────
if SAVE_PNG
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    saveas(fig, fullfile(output_dir, sprintf('comparison_%s.png', timestamp)));
    fprintf('  Figure saved\n');
end

fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('                    Complete\n');
fprintf('════════════════════════════════════════════════════════════\n\n');


%% Helper
function wrapped = wrapToPi(angle)
    wrapped = mod(angle + pi, 2*pi) - pi;
end
