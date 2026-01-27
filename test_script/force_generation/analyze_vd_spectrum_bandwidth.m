%% ANALYZE_VD_SPECTRUM_BANDWIDTH Vd 頻譜分析與 Inner Loop 頻寬需求驗證
%
% 驗證兩個假設：
%   1. 頻譜不變性：Inverse Model 是靜態非線性，Vd 的正規化諧波比例不隨基頻變化
%   2. 頻寬需求：若 Vd 在 N 倍頻的能量 ≥ 門檻，則 Inner Loop 需 ≥ N × f_d
%
% 輸出：
%   - Console 分析報告
%   - 頻譜不變性驗證圖
%   - 頻寬需求曲線圖
%
% See also: inverse_model, force_model_allocation_params, CLAUDE.md

clear; clc;

%% SECTION 1: Configuration
fprintf('=== Vd Spectrum Analysis for Inner Loop Bandwidth ===\n\n');

% Test frequencies (Hz)
test_frequencies = [10, 50, 100, 200];

% Force parameters
force_amplitude = 5;        % pN
force_direction = [1; 0; 0]; % X-direction (Measuring coordinate)
bead_position = [0; 0; 0];  % um (center position)

% Simulation parameters
sim_cycles = 20;            % Number of cycles to simulate
fs = 100000;                % Sampling frequency (Hz) - matches controller rate

% Analysis parameters
harmonic_threshold = 0.01;  % 1% threshold for significant harmonics
max_harmonic_order = 20;    % Maximum harmonic order to analyze

% Path setup
script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
addpath(fullfile(project_root, 'model', 'flux_allocation'));
addpath(fullfile(project_root, 'test_script', 'utils'));

% Load parameters and styles
alloc_params = force_model_allocation_params();
styles = plot_styles();

%% SECTION 2: Generate Vd for Each Frequency
fprintf('[Phase 1: Generating Vd signals]\n');

% Storage for results
vd_data = cell(length(test_frequencies), 1);
time_data = cell(length(test_frequencies), 1);
fd_data = cell(length(test_frequencies), 1);

for f_idx = 1:length(test_frequencies)
    f0 = test_frequencies(f_idx);

    % Time vector
    T_sim = sim_cycles / f0;  % Total simulation time
    dt = 1 / fs;
    t = (0:dt:(T_sim - dt))';
    N_samples = length(t);

    % Generate sinusoidal force demand
    f_d_scalar = force_amplitude * sin(2 * pi * f0 * t);

    % Generate v_d for each time step
    v_d_all = zeros(N_samples, 6);

    for k = 1:N_samples
        f_d = force_direction * f_d_scalar(k);
        v_d = inverse_model(f_d, bead_position, alloc_params);
        v_d_all(k, :) = v_d';
    end

    % Store data
    vd_data{f_idx} = v_d_all;
    time_data{f_idx} = t;
    fd_data{f_idx} = f_d_scalar;

    fprintf('  f0 = %4d Hz: Generated %d samples (%.3f s)\n', f0, N_samples, T_sim);
end

%% SECTION 3: FFT Analysis
fprintf('\n[Phase 2: FFT Analysis]\n');

% Storage for harmonic amplitudes (normalized)
% Dimensions: [frequency_idx, channel_idx, harmonic_order]
harmonic_ratios = zeros(length(test_frequencies), 6, max_harmonic_order);
fundamental_amplitudes = zeros(length(test_frequencies), 6);

for f_idx = 1:length(test_frequencies)
    f0 = test_frequencies(f_idx);
    v_d_all = vd_data{f_idx};
    N = size(v_d_all, 1);

    % Skip first few cycles (transient)
    skip_cycles = 2;
    skip_samples = round(skip_cycles / f0 * fs);
    v_d_steady = v_d_all(skip_samples+1:end, :);
    N_steady = size(v_d_steady, 1);

    % Frequency resolution
    df = fs / N_steady;
    freq_axis = (0:N_steady-1) * df;

    for ch = 1:6
        % FFT of steady-state signal
        Y = fft(v_d_steady(:, ch));
        mag = abs(Y) / N_steady * 2;  % Single-sided amplitude

        % Find fundamental frequency bin
        [~, fund_bin] = min(abs(freq_axis - f0));
        fundamental_amp = mag(fund_bin);
        fundamental_amplitudes(f_idx, ch) = fundamental_amp;

        if fundamental_amp < 1e-10
            continue;  % Skip if no signal
        end

        % Extract harmonic amplitudes (normalized to fundamental)
        for h = 1:max_harmonic_order
            target_freq = h * f0;
            if target_freq >= fs/2
                break;  % Nyquist limit
            end
            [~, harm_bin] = min(abs(freq_axis - target_freq));
            harmonic_ratios(f_idx, ch, h) = mag(harm_bin) / fundamental_amp;
        end
    end
end

%% SECTION 4: Verify Spectrum Invariance
fprintf('\n[Phase 3: Spectrum Invariance Verification]\n');

% Select active channel (highest fundamental amplitude)
[~, active_ch] = max(mean(fundamental_amplitudes, 1));
fprintf('  Active channel for analysis: P%d\n\n', active_ch);

% Calculate statistics across frequencies for each harmonic
fprintf('  Normalized Harmonic Ratios (Channel P%d):\n', active_ch);
fprintf('  %-12s %-12s %-12s %-12s\n', 'Harmonic', 'Mean', 'Std', 'Variation');
fprintf('  %s\n', repmat('-', 1, 52));

invariance_pass = true;
significant_harmonics = [];

for h = 2:10  % Check 2nd to 10th harmonics
    ratios = squeeze(harmonic_ratios(:, active_ch, h));
    mean_ratio = mean(ratios);
    std_ratio = std(ratios);

    if mean_ratio > 1e-4
        variation = std_ratio / mean_ratio * 100;
        fprintf('  %2d×f₀       %8.4f     %8.4f     %5.1f%%\n', h, mean_ratio, std_ratio, variation);

        if mean_ratio >= harmonic_threshold
            significant_harmonics = [significant_harmonics, h];
        end

        if variation > 5
            invariance_pass = false;
        end
    else
        fprintf('  %2d×f₀       %8.4f     (negligible)\n', h, mean_ratio);
    end
end

fprintf('\n  Spectrum Invariance: ');
if invariance_pass
    fprintf('PASS - Normalized spectra are consistent across frequencies\n');
else
    fprintf('FAIL - Significant variation detected\n');
end

%% SECTION 5: Calculate Bandwidth Requirement
fprintf('\n[Phase 4: Bandwidth Requirement Analysis]\n');

% Use mean ratios across all test frequencies
mean_harmonic_ratios = squeeze(mean(harmonic_ratios(:, active_ch, :), 1));

% Find highest significant harmonic
effective_bw_multiplier = 1;
for h = 1:max_harmonic_order
    if mean_harmonic_ratios(h) >= harmonic_threshold
        effective_bw_multiplier = h;
    end
end

% Find harmonic at different thresholds
thresholds = [0.1, 0.05, 0.01, 0.001];
threshold_labels = {'10%', '5%', '1%', '0.1%'};

fprintf('  Energy Threshold Analysis:\n');
fprintf('  %-12s %-20s %-20s\n', 'Threshold', 'Highest Harmonic', 'BW Multiplier');
fprintf('  %s\n', repmat('-', 1, 52));

for t_idx = 1:length(thresholds)
    thresh = thresholds(t_idx);
    bw_mult = 1;
    for h = 1:max_harmonic_order
        if mean_harmonic_ratios(h) >= thresh
            bw_mult = h;
        end
    end
    fprintf('  %-12s %-20d %-20d×\n', threshold_labels{t_idx}, bw_mult, bw_mult);
end

fprintf('\n  Primary Result (1%% threshold):\n');
fprintf('    --> Effective bandwidth = %d × f_d\n', effective_bw_multiplier);
fprintf('    --> For f_d = 100 Hz, Inner Loop needs ≥ %d Hz\n', effective_bw_multiplier * 100);
fprintf('    --> For f_d = 200 Hz, Inner Loop needs ≥ %d Hz\n', effective_bw_multiplier * 200);

%% SECTION 6: Calculate Cumulative Energy
fprintf('\n[Phase 5: Cumulative Energy Analysis]\n');

% Calculate cumulative energy (sum of squared amplitudes)
% Energy of each harmonic: E_n = |H_n|^2
harmonic_energy = mean_harmonic_ratios.^2;
total_energy = sum(harmonic_energy(1:max_harmonic_order));
cumulative_energy = cumsum(harmonic_energy(1:max_harmonic_order)) / total_energy * 100;

% Find harmonic order for different energy thresholds
energy_thresholds = [90, 95, 99, 99.9];
energy_bw_multipliers = zeros(size(energy_thresholds));

fprintf('  Cumulative Energy Analysis:\n');
fprintf('  %-15s %-20s\n', 'Energy Captured', 'Harmonics Required');
fprintf('  %s\n', repmat('-', 1, 40));

for t_idx = 1:length(energy_thresholds)
    thresh = energy_thresholds(t_idx);
    bw_mult = find(cumulative_energy >= thresh, 1, 'first');
    if isempty(bw_mult)
        bw_mult = max_harmonic_order;
    end
    energy_bw_multipliers(t_idx) = bw_mult;
    fprintf('  %-15s %-20d\n', sprintf('%.1f%%', thresh), bw_mult);
end

%% SECTION 7: Generate Figures
fprintf('\n[Phase 6: Generating Figures]\n');

% Create output directory
output_dir = fullfile(project_root, 'test_results', 'force_generation', 'spectrum_analysis');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
timestamp = datestr(now, 'yyyymmdd_HHMMSS');

% --- Figure 1: Spectrum Invariance Verification ---
% Single figure showing normalized spectra
% Draw order: thickest first (10Hz), thinnest last (200Hz) - so thin lines overlay thick
fig1 = figure('Position', [100, 100, 900, 520]);

% Draw order: 10Hz (thickest) -> 50Hz -> 100Hz -> 200Hz (thinnest)
draw_order = [1, 2, 3, 4];  % Indices into test_frequencies
line_widths = [4.0, 3.2, 2.4, 1.6];  % Decreasing
marker_sizes = [12, 10, 8, 6];  % Decreasing
markers = {'o', 's', '^', 'd'};

% Color scheme: high contrast
freq_colors = [
    0.85, 0.33, 0.10;  % f0=10Hz:  Orange-red
    0.00, 0.45, 0.74;  % f0=50Hz:  Blue
    0.47, 0.67, 0.19;  % f0=100Hz: Green
    0.49, 0.18, 0.56;  % f0=200Hz: Purple
];

n_harmonics_plot = 12;

% Create axes with room for title and legend at top
ax1 = axes('Position', [0.10, 0.13, 0.87, 0.65]);
hold on;

for draw_idx = 1:length(draw_order)
    f_idx = draw_order(draw_idx);
    f0 = test_frequencies(f_idx);
    ratios = squeeze(harmonic_ratios(f_idx, active_ch, 1:n_harmonics_plot));

    % Convert to percentage
    ratios_percent = ratios * 100;

    semilogy(1:n_harmonics_plot, ratios_percent, '-', ...
        'Color', freq_colors(f_idx, :), ...
        'LineWidth', line_widths(draw_idx), ...
        'Marker', markers{draw_idx}, ...
        'MarkerSize', marker_sizes(draw_idx), ...
        'MarkerFaceColor', freq_colors(f_idx, :), ...
        'MarkerEdgeColor', 'none');
end

% Axis labels (larger, bold)
xlabel('Harmonic Order, n', 'FontSize', 18, 'FontWeight', 'bold');
ylabel('H_n / H_1  [%]', 'FontSize', 18, 'FontWeight', 'bold');

% Legend (horizontal, with box) - positioned inside axes at top
legend_entries = cell(4, 1);
for k = 1:4
    legend_entries{k} = sprintf('f_d = %d Hz', test_frequencies(k));
end
% Get handles in reverse order (last plotted = first in Children)
h = get(gca, 'Children');
leg = legend(h([4, 3, 2, 1]), legend_entries{:}, ...
    'Orientation', 'horizontal', ...
    'FontSize', 12, 'FontWeight', 'bold', ...
    'Box', 'on', 'LineWidth', 1.5);
% Position legend at top center, below title area
leg.Position = [0.22, 0.82, 0.56, 0.05];

% Title above legend using annotation
annotation('textbox', [0.1, 0.90, 0.8, 0.08], ...
    'String', 'Vd Normalized Spectrum', ...
    'FontSize', 20, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none');

% Axis settings - clean look without grid
set(gca, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 2.0, 'Box', 'on');
grid off;  % No grid for cleaner look
xlim([0.5, n_harmonics_plot + 0.5]);
ylim([2, 100]);  % Start from 2% to avoid crowding at bottom
xticks(1:n_harmonics_plot);
% Y ticks with good spacing on log scale
yticks([2, 5, 10, 20, 50, 100]);
yticklabels({'2', '5', '10', '20', '50', '100'});

% Save Figure 1
fig1_path = fullfile(output_dir, sprintf('fig1_spectrum_invariance_%s.png', timestamp));
exportgraphics(fig1, fig1_path, 'Resolution', styles.export_resolution);
fprintf('  Saved: %s\n', fig1_path);

% --- Figure 2: Cumulative Energy & Bandwidth Requirement ---
fig2 = figure('Position', [100, 100, 900, 520]);

% Create axes with room for title at top
ax2 = axes('Position', [0.10, 0.13, 0.85, 0.70]);
hold on;

% Plot cumulative energy curve
plot(1:max_harmonic_order, cumulative_energy, '-o', ...
    'Color', [0.00, 0.45, 0.74], ...
    'LineWidth', 3.5, ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', [0.00, 0.45, 0.74], ...
    'MarkerEdgeColor', 'none');

% Key threshold points
n_95 = energy_bw_multipliers(2);   % 95%
n_99 = energy_bw_multipliers(3);   % 99%
n_999 = energy_bw_multipliers(4);  % 99.9%

% Add threshold lines (only 99% as main reference)
yline(99, '--', 'Color', [0.8, 0, 0], 'LineWidth', 2.5);

% Mark key points with different colors
% 99% point (main)
plot(n_99, cumulative_energy(n_99), 'v', ...
    'MarkerSize', 16, 'MarkerFaceColor', [0.8, 0, 0], ...
    'MarkerEdgeColor', 'none', 'LineWidth', 2);

% 99.9% point
plot(n_999, cumulative_energy(n_999), 's', ...
    'MarkerSize', 14, 'MarkerFaceColor', [0.5, 0, 0.5], ...
    'MarkerEdgeColor', 'none', 'LineWidth', 2);

% Axis labels
xlabel('Harmonics Included (1 to n)', 'FontSize', 18, 'FontWeight', 'bold');
ylabel('Cumulative Energy [%]', 'FontSize', 18, 'FontWeight', 'bold');

% Title using annotation
annotation('textbox', [0.1, 0.90, 0.8, 0.08], ...
    'String', 'Vd Cumulative Energy', ...
    'FontSize', 20, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none');

% Axis settings - clean look
set(gca, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 2.0, 'Box', 'on');
grid off;
xlim([1, 16]);
ylim([80, 100.5]);
xticks(1:16);
yticks([80, 85, 90, 95, 99, 100]);

% Information table (right side, inside plot)
info_x = 0.98;
info_y = 0.38;
info_text = {
    ' Energy     n    Force Err ', ...
    '───────────────────────', ...
    sprintf('   95%%      %d      ~45%%', n_95), ...
    sprintf('   99%%      %d      ~20%%', n_99), ...
    sprintf('  99.9%%    %d       ~6%%', n_999), ...
    '───────────────────────', ...
    sprintf('  BW = n × f_d')
};
text(info_x, info_y, info_text, ...
    'Units', 'normalized', 'FontSize', 11, 'FontName', 'FixedWidth', ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', ...
    'BackgroundColor', 'white', 'EdgeColor', 'black', ...
    'LineWidth', 1.5, 'Margin', 6);

% Label the marked points
text(n_99 + 0.4, cumulative_energy(n_99) - 1.5, sprintf('n = %d', n_99), ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.8, 0, 0]);
text(n_999 + 0.4, cumulative_energy(n_999) + 0.8, sprintf('n = %d', n_999), ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.5, 0, 0.5]);

% 99% label on right
text(16.3, 99, '99%', 'FontSize', 13, 'FontWeight', 'bold', ...
    'Color', [0.8, 0, 0], 'VerticalAlignment', 'middle');

% Save Figure 2
fig2_path = fullfile(output_dir, sprintf('fig2_cumulative_energy_%s.png', timestamp));
exportgraphics(fig2, fig2_path, 'Resolution', styles.export_resolution);
fprintf('  Saved: %s\n', fig2_path);

%% SECTION 8: Summary
fprintf('\n=== Analysis Complete ===\n');
fprintf('\nKey Results:\n');
fprintf('  1. Spectrum Invariance: %s\n', conditional_str(invariance_pass, 'VERIFIED', 'FAILED'));
fprintf('\n  2. Bandwidth Requirement (Energy Method):\n');
fprintf('     Energy Capture    Harmonics (N)    Est. Force Error\n');
fprintf('     ─────────────────────────────────────────────────\n');
fprintf('         95%%              %d               ~45%%\n', energy_bw_multipliers(2));
fprintf('         99%%              %d               ~20%%\n', energy_bw_multipliers(3));
fprintf('        99.9%%            %d                ~6%%\n', energy_bw_multipliers(4));
fprintf('\n  3. Inner Loop Bandwidth Formula:\n');
fprintf('     BW_inner_loop ≥ N × f_d_max\n');
fprintf('\n     Example (99%% energy, ~20%% force error):\n');
fprintf('     - f_d_max = 100 Hz → BW ≥ %d Hz\n', energy_bw_multipliers(3) * 100);
fprintf('     - f_d_max = 200 Hz → BW ≥ %d Hz\n', energy_bw_multipliers(3) * 200);
fprintf('\n     Example (99.9%% energy, ~6%% force error):\n');
fprintf('     - f_d_max = 100 Hz → BW ≥ %d Hz\n', energy_bw_multipliers(4) * 100);
fprintf('     - f_d_max = 200 Hz → BW ≥ %d Hz\n', energy_bw_multipliers(4) * 200);
fprintf('\nFigures saved to: %s\n', output_dir);


% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function str = conditional_str(condition, true_str, false_str)
% CONDITIONAL_STR Return string based on condition
    if condition
        str = true_str;
    else
        str = false_str;
    end
end
