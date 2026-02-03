%% LPF Effect Comparison Script
% COMPARE_LPF_EFFECT Compare control effort (u) with and without LPF enabled
%
% This script loads two .mat files from run_force_generation_test.m and
% compares the time-domain and spectral characteristics of the control effort
% signal (u) before and after LPF filtering.
%
% Signal source:
%   - Original mode: u (control effort before LPF)
%   - LPF mode: u_lpf (control effort after LPF filtering)
%
% Output:
%   - 6 tabs (P1-P6), each with time-domain (top) and spectrum (bottom)
%
% Example:
%   Set mat_original and mat_lpf paths, then run the script.
%
% See also: run_force_generation_test.m, CLAUDE.md

clear; clc;

%% ════════════════════════════════════════════════════════════════════════
%                    Configuration
%% ════════════════════════════════════════════════════════════════════════

% ─────────────────────────────────────────────────────────────────────────
% Input .mat file paths
% ─────────────────────────────────────────────────────────────────────────
mat_original = '/Users/kevin/Code/r_controller_package/test_results/force_generation/force_control/force_control_test_20260202_143257/results.mat';
mat_lpf = '/Users/kevin/Code/r_controller_package/test_results/force_generation/force_control/force_control_test_20260202_143344/results.mat';

% ─────────────────────────────────────────────────────────────────────────
% Display options
% ─────────────────────────────────────────────────────────────────────────
SAVE_PNG = true;    % Save output as PNG (one per tab)

%% ════════════════════════════════════════════════════════════════════════
%                    Path Setup
%% ════════════════════════════════════════════════════════════════════════
script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
addpath(fullfile(project_root, 'test_script', 'utils'));

% Load plot styles
styles = plot_styles();

%% ════════════════════════════════════════════════════════════════════════
%                    Input Validation
%% ════════════════════════════════════════════════════════════════════════
if isempty(mat_original) || isempty(mat_lpf)
    error('compare_lpf_effect:missingInput', ...
        'Please set mat_original and mat_lpf paths in the Configuration section.');
end

if ~isfile(mat_original)
    error('compare_lpf_effect:fileNotFound', ...
        'Original .mat file not found: %s', mat_original);
end

if ~isfile(mat_lpf)
    error('compare_lpf_effect:fileNotFound', ...
        'LPF .mat file not found: %s', mat_lpf);
end

%% ════════════════════════════════════════════════════════════════════════
%                    Load Data
%% ════════════════════════════════════════════════════════════════════════
fprintf('Loading data...\n');

% Load original (LPF disabled) results
data_orig = load(mat_original, 'results');
results_orig = data_orig.results;
fprintf('  Original: %s\n', mat_original);

% Load LPF enabled results
data_lpf = load(mat_lpf, 'results');
results_lpf = data_lpf.results;
fprintf('  LPF:      %s\n', mat_lpf);

%% ════════════════════════════════════════════════════════════════════════
%                    Data Validation
%% ════════════════════════════════════════════════════════════════════════
fprintf('\nValidating data compatibility...\n');

% Check required fields exist
if ~isfield(results_orig.data, 'u')
    error('compare_lpf_effect:missingData', ...
        'Original results missing u data. Ensure USE_SIMULINK was enabled.');
end

if ~isfield(results_lpf.data, 'u_lpf')
    error('compare_lpf_effect:missingData', ...
        'LPF results missing u_lpf data. Ensure lpf_enable was true during test.');
end

fprintf('  Validation complete.\n');

%% ════════════════════════════════════════════════════════════════════════
%                    Extract Common Parameters
%% ════════════════════════════════════════════════════════════════════════
fprintf('\nExtracting signals for all 6 channels...\n');

% Time vector
t_orig = results_orig.data.t;
t_lpf = results_lpf.data.t;

% Control effort signals (in Amperes) - all 6 channels
u_orig_all = results_orig.data.u;        % Original: u
u_lpf_all = results_lpf.data.u_lpf;      % LPF: u_lpf

% Get test parameters from .mat files
force_frequency = results_orig.config.force_frequency;
force_amplitude = results_orig.config.force_amplitude;
f_low = results_lpf.config.f_low;

% Get force error metrics (Max %)
error_max_orig = max(results_orig.analysis.force_error_max) / force_amplitude * 100;
error_max_lpf = max(results_lpf.analysis.force_error_max) / force_amplitude * 100;

fprintf('  Force frequency: %d Hz\n', force_frequency);
fprintf('  Force amplitude: %.1f pN\n', force_amplitude);
fprintf('  LPF cutoff (f_low): %d Hz\n', f_low);
fprintf('  Force error (Max) - Original: %.2f%%, LPF: %.2f%%\n', error_max_orig, error_max_lpf);

%% ════════════════════════════════════════════════════════════════════════
%                    FFT Parameters
%% ════════════════════════════════════════════════════════════════════════
fprintf('\nComputing FFT parameters...\n');

% Sampling parameters
Ts = t_orig(2) - t_orig(1);
fs = 1 / Ts;

% Use steady-state portion (skip first 40% of data)
samples_to_skip = round(size(u_orig_all, 1) * 0.4);

% Use common length for FFT
N_fft = min(size(u_orig_all, 1) - samples_to_skip, size(u_lpf_all, 1) - samples_to_skip);

% Frequency vector
half_N = floor(N_fft / 2);
freq = (0:half_N-1) * fs / N_fft;

% Frequency display range (for log scale, start from > 0)
freq_min_display = 10;  % Start from 10 Hz
freq_max_display = min(50000, fs/2);
freq_display_idx = (freq >= freq_min_display) & (freq <= freq_max_display);

fprintf('  FFT length: N = %d samples\n', N_fft);

%% ════════════════════════════════════════════════════════════════════════
%                    Color Definitions
%% ════════════════════════════════════════════════════════════════════════
color_original = [0.6, 0.6, 0.6];        % Gray for original
color_lpf = [0.466, 0.674, 0.188];       % Green for LPF
color_cutoff = [1, 0, 0];                 % Red for cutoff frequency

%% ════════════════════════════════════════════════════════════════════════
%                    Pre-compute Y-axis Limits for All Channels
%% ════════════════════════════════════════════════════════════════════════
fprintf('\nPre-computing Y-axis limits...\n');

% Time range for display (show 2 cycles in steady state)
t_display_start = samples_to_skip * Ts;
cycles_to_show = 2;
t_display_end = t_display_start + cycles_to_show / force_frequency;

% Find indices for display range
idx_start = find(t_orig >= t_display_start, 1, 'first');
idx_end = find(t_orig <= t_display_end, 1, 'last');
if isempty(idx_end) || idx_end > length(t_orig)
    idx_end = length(t_orig);
end
idx_end_lpf = min(idx_end, length(t_lpf));

% Initialize limits
time_y_min = inf;
time_y_max = -inf;
spec_y_max = 0;

% Store FFT results for each channel
fund_amp_orig_all = zeros(1, 6);
fund_amp_lpf_all = zeros(1, 6);
mag_orig_norm_all = cell(1, 6);
mag_lpf_norm_all = cell(1, 6);

% First pass: compute all FFT and find global limits
for ch = 1:6
    u_orig_ch = u_orig_all(:, ch);
    u_lpf_ch = u_lpf_all(:, ch);

    % Time domain limits
    u_orig_display = u_orig_ch(idx_start:idx_end);
    u_lpf_display = u_lpf_ch(idx_start:idx_end_lpf);
    time_y_min = min([time_y_min; u_orig_display; u_lpf_display]);
    time_y_max = max([time_y_max; u_orig_display; u_lpf_display]);

    % FFT computation
    u_orig_steady = u_orig_ch(samples_to_skip:samples_to_skip+N_fft-1);
    u_lpf_steady = u_lpf_ch(samples_to_skip:samples_to_skip+N_fft-1);

    Y_orig = fft(u_orig_steady);
    Y_lpf = fft(u_lpf_steady);

    mag_orig = abs(Y_orig(1:half_N)) / N_fft * 2;
    mag_lpf = abs(Y_lpf(1:half_N)) / N_fft * 2;

    % Get fundamental amplitude
    fund_idx = round(force_frequency * N_fft / fs) + 1;
    if fund_idx > 1 && fund_idx <= half_N
        fund_amp_orig_all(ch) = mag_orig(fund_idx);
        fund_amp_lpf_all(ch) = mag_lpf(fund_idx);

        if fund_amp_orig_all(ch) > 0
            mag_orig_norm_all{ch} = mag_orig / fund_amp_orig_all(ch);
        else
            mag_orig_norm_all{ch} = mag_orig;
        end

        if fund_amp_lpf_all(ch) > 0
            mag_lpf_norm_all{ch} = mag_lpf / fund_amp_lpf_all(ch);
        else
            mag_lpf_norm_all{ch} = mag_lpf;
        end
    else
        fund_amp_orig_all(ch) = 0;
        fund_amp_lpf_all(ch) = 0;
        mag_orig_norm_all{ch} = mag_orig;
        mag_lpf_norm_all{ch} = mag_lpf;
    end

    % Spectrum Y limit (exclude fundamental peak for better visualization)
    mag_orig_no_fund = mag_orig_norm_all{ch};
    mag_lpf_no_fund = mag_lpf_norm_all{ch};
    % Zero out fundamental region for max calculation
    fund_region = max(1, fund_idx-5):min(half_N, fund_idx+5);
    mag_orig_no_fund(fund_region) = 0;
    mag_lpf_no_fund(fund_region) = 0;
    spec_y_max = max([spec_y_max, max(mag_orig_no_fund(freq_display_idx)), max(mag_lpf_no_fund(freq_display_idx))]);
end

% Round Y-axis limits to nice values for clean ticks
% Time domain: round to nearest 10 or 50
time_range = time_y_max - time_y_min;
if time_range > 100
    time_tick_step = 50;
elseif time_range > 50
    time_tick_step = 20;
elseif time_range > 20
    time_tick_step = 10;
else
    time_tick_step = 5;
end
time_y_min_round = floor(time_y_min / time_tick_step) * time_tick_step;
time_y_max_round = ceil(time_y_max / time_tick_step) * time_tick_step;
time_ylim = [time_y_min_round, time_y_max_round];
time_yticks = time_y_min_round:time_tick_step:time_y_max_round;

% Spectrum: round to nice values
if spec_y_max <= 0.5
    spec_y_max_nice = ceil(spec_y_max * 10) / 10;
    spec_tick_step = 0.1;
elseif spec_y_max <= 1
    spec_y_max_nice = ceil(spec_y_max * 5) / 5;
    spec_tick_step = 0.2;
elseif spec_y_max <= 2
    spec_y_max_nice = ceil(spec_y_max * 2) / 2;
    spec_tick_step = 0.5;
elseif spec_y_max <= 5
    spec_y_max_nice = ceil(spec_y_max);
    spec_tick_step = 1;
else
    spec_y_max_nice = ceil(spec_y_max / 2) * 2;
    spec_tick_step = 2;
end
if spec_y_max_nice < 0.1
    spec_y_max_nice = 0.1;
    spec_tick_step = 0.05;
end
spec_ylim = [0, spec_y_max_nice];
spec_yticks = 0:spec_tick_step:spec_y_max_nice;

fprintf('  Time Y-axis: [%.0f, %.0f] A, step=%.0f\n', time_ylim(1), time_ylim(2), time_tick_step);
fprintf('  Spectrum Y-axis: [0, %.2f], step=%.2f\n', spec_ylim(2), spec_tick_step);

%% ════════════════════════════════════════════════════════════════════════
%                    Create Figure with 6 Tabs
%% ════════════════════════════════════════════════════════════════════════
fprintf('\nCreating comparison figure (6 tabs)...\n');

fig = figure('Name', 'LPF Effect Comparison', ...
    'Position', [100, 100, 1000, 700], ...
    'Color', 'white');

tabgroup = uitabgroup(fig);
tab_handles = cell(1, 6);

% Create 6 tabs (P1-P6)
for ch = 1:6
    % Create tab
    tab_handles{ch} = uitab(tabgroup, 'Title', sprintf('P%d', ch));

    % Create tiledlayout inside tab
    tl = tiledlayout(tab_handles{ch}, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    % Overall title with force max error only
    title(tl, sprintf('Force Max Error: Original=%.2f%%, LPF=%.2f%%', ...
        error_max_orig, error_max_lpf), 'FontSize', styles.title_fontsize + 2, 'FontWeight', 'bold');

    % Extract channel data
    u_orig_ch = u_orig_all(:, ch);
    u_lpf_ch = u_lpf_all(:, ch);

    % ─────────────────────────────────────────────────────────────────────
    % Top: Time Domain
    % ─────────────────────────────────────────────────────────────────────
    ax1 = nexttile(tl);
    hold(ax1, 'on');

    % Plot LPF first (green), then Original (gray)
    plot(ax1, t_lpf(idx_start:idx_end_lpf)*1000, u_lpf_ch(idx_start:idx_end_lpf), ...
        'Color', color_lpf, 'LineWidth', 2.0, 'DisplayName', sprintf('LPF (f_{low}=%dHz)', f_low));
    plot(ax1, t_orig(idx_start:idx_end)*1000, u_orig_ch(idx_start:idx_end), ...
        'Color', color_original, 'LineWidth', 2.5, 'DisplayName', 'Original (No LPF)');

    hold(ax1, 'off');

    xlabel(ax1, 'Time [ms]', 'FontSize', styles.xlabel_fontsize, 'FontWeight', styles.label_fontweight);
    ylabel(ax1, 'Control Effort u [A]', 'FontSize', styles.ylabel_fontsize, 'FontWeight', styles.label_fontweight);
    title(ax1, sprintf('P%d - Time Domain (f_{force}=%dHz)', ch, force_frequency), ...
        'FontSize', styles.title_fontsize, 'FontWeight', styles.title_fontweight);

    % Legend below title
    legend(ax1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', styles.legend_fontsize);

    grid(ax1, 'on');
    set(ax1, 'FontSize', styles.tick_fontsize, 'Box', 'on', 'LineWidth', styles.axis_linewidth);

    % ─────────────────────────────────────────────────────────────────────
    % Bottom: Spectrum (Linear Y, Log X)
    % ─────────────────────────────────────────────────────────────────────
    ax2 = nexttile(tl);
    hold(ax2, 'on');

    % Use pre-computed normalized spectrum
    mag_orig_norm = mag_orig_norm_all{ch};
    mag_lpf_norm = mag_lpf_norm_all{ch};

    % Plot spectrum (Linear Y, will set Log X after)
    % Plot LPF first (green), then Original (gray)
    plot(ax2, freq(freq_display_idx), mag_lpf_norm(freq_display_idx), ...
        'Color', color_lpf, 'LineWidth', 2.0, 'DisplayName', sprintf('LPF (f_{low}=%dHz)', f_low));
    plot(ax2, freq(freq_display_idx), mag_orig_norm(freq_display_idx), ...
        'Color', color_original, 'LineWidth', 2.0, 'DisplayName', 'Original (No LPF)');

    % Add cutoff frequency line
    xline(ax2, f_low, '--', 'Color', color_cutoff, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('f_{low}=%dHz', f_low));

    hold(ax2, 'off');

    % Set log scale for X axis
    set(ax2, 'XScale', 'log');

    xlabel(ax2, 'Frequency [Hz]', 'FontSize', styles.xlabel_fontsize, 'FontWeight', styles.label_fontweight);
    ylabel(ax2, 'Magnitude (normalized)', 'FontSize', styles.ylabel_fontsize, 'FontWeight', styles.label_fontweight);

    % Title with fundamental magnitude
    title(ax2, sprintf('P%d - Spectrum (Fundamental: Original=%.4f A, LPF=%.4f A)', ...
        ch, fund_amp_orig_all(ch), fund_amp_lpf_all(ch)), ...
        'FontSize', styles.title_fontsize, 'FontWeight', styles.title_fontweight);

    % Legend below title
    legend(ax2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', styles.legend_fontsize);

    grid(ax2, 'on');
    set(ax2, 'FontSize', styles.tick_fontsize, 'Box', 'on', 'LineWidth', styles.axis_linewidth);
    xlim(ax2, [freq_min_display, freq_max_display]);

    fprintf('  Tab P%d created\n', ch);
end

fprintf('  All 6 tabs created.\n');

%% ════════════════════════════════════════════════════════════════════════
%                    Save Output
%% ════════════════════════════════════════════════════════════════════════
if SAVE_PNG
    [lpf_dir, ~, ~] = fileparts(mat_lpf);

    fprintf('\nSaving figures...\n');
    for ch = 1:6
        tabgroup.SelectedTab = tab_handles{ch};
        drawnow; pause(0.1);

        output_file = fullfile(lpf_dir, sprintf('lpf_comparison_P%d.png', ch));
        exportgraphics(tab_handles{ch}, output_file, 'Resolution', styles.export_resolution);
        fprintf('  Saved: lpf_comparison_P%d.png\n', ch);
    end

    fprintf('\nAll figures saved to:\n  %s\n', lpf_dir);
end

%% ════════════════════════════════════════════════════════════════════════
%                    Summary
%% ════════════════════════════════════════════════════════════════════════
fprintf('\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('                 LPF Effect Comparison Summary\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('  Channels:        P1 - P6 (6 tabs)\n');
fprintf('  Force frequency: %d Hz\n', force_frequency);
fprintf('  Force amplitude: %.1f pN\n', force_amplitude);
fprintf('  LPF cutoff:      %d Hz\n', f_low);
fprintf('────────────────────────────────────────────────────────────\n');
fprintf('  Force Error (Max %%)\n');
fprintf('    Original:      %.2f %%\n', error_max_orig);
fprintf('    LPF On:        %.2f %%\n', error_max_lpf);
fprintf('    Difference:    %+.2f %%\n', error_max_lpf - error_max_orig);
fprintf('════════════════════════════════════════════════════════════\n');
