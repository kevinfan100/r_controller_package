function results = analyze_vd_nonlinearity(varargin)
% ANALYZE_VD_NONLINEARITY Analyze force-to-voltage nonlinearity (vd THD analysis)
%
% Analyzes the Total Harmonic Distortion (THD) in the vd output from
% inverse_model when driven by sinusoidal force inputs with varying DC offsets.
%
% The nonlinearity source is the sqrt(F_mag) scaling in inverse_model.m:
%   current_scale = sqrt(F_mag / (params.g_H / params.force_scale))
%
% This causes distortion when the force signal crosses or approaches zero.
%
% Usage:
%   results = analyze_vd_nonlinearity();           % Use defaults
%   results = analyze_vd_nonlinearity('Amplitude', 10, 'Frequency', 100);
%
% Parameters (Name-Value pairs):
%   'Amplitude'  - Force amplitude [pN] (default: 5)
%   'Frequency'  - Signal frequency [Hz] (default: 50)
%   'Offsets'    - Array of DC offset values [pN] (default: [0, 2.5, 5, 7.5, 10])
%   'Position'   - Bead position [um] (default: [0, 0, 0])
%   'Duration'   - Signal duration [s] (default: 0.5)
%   'SavePlots'  - Save plots to files (default: true)
%   'ShowPlots'  - Display plots (default: true)
%
% Output:
%   results - Structure containing:
%     .test_params     - Test parameters used
%     .thd_results     - THD values [num_offsets x 6 channels]
%     .harmonic_mags   - Harmonic magnitudes {cell array}
%     .vd_data         - vd output data {cell array}
%     .fd_data         - Force input data {cell array}
%
% Example:
%   % Analyze with default parameters
%   results = analyze_vd_nonlinearity();
%
%   % Analyze with custom amplitude and frequency
%   results = analyze_vd_nonlinearity('Amplitude', 10, 'Frequency', 100);
%
%   % Extract THD for channel 5 (Z-axis related)
%   thd_ch5 = results.thd_results(:, 5);
%
% Author: ZPETC Package Team
% Date: 2025-01-20

    % ========================================
    % Parse Input Arguments
    % ========================================
    p = inputParser;
    addParameter(p, 'Amplitude', 5, @isnumeric);
    addParameter(p, 'Frequency', 50, @isnumeric);
    addParameter(p, 'Offsets', [0, 2.5, 5, 7.5, 10], @isnumeric);
    addParameter(p, 'Position', [0, 0, 0], @isnumeric);
    addParameter(p, 'Duration', 0.5, @isnumeric);
    addParameter(p, 'SavePlots', true, @islogical);
    addParameter(p, 'ShowPlots', true, @islogical);
    parse(p, varargin{:});

    amplitude = p.Results.Amplitude;
    freq = p.Results.Frequency;
    offsets = p.Results.Offsets(:)';  % Ensure row vector
    pos_m = p.Results.Position(:);    % Ensure column vector
    duration = p.Results.Duration;
    save_plots = p.Results.SavePlots;
    show_plots = p.Results.ShowPlots;

    % ========================================
    % Setup
    % ========================================
    script_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(script_dir, '..', '..', 'model', 'flux_allocation'));

    params = system_params();

    fs = 100000;  % Sampling frequency [Hz]
    t = (0:1/fs:duration-1/fs)';
    N = length(t);
    num_offsets = length(offsets);
    n_harmonics = 10;  % Number of harmonics to analyze

    fprintf('========================================\n');
    fprintf('vd Nonlinearity Analysis (THD)\n');
    fprintf('========================================\n');
    fprintf('Test Parameters:\n');
    fprintf('  Amplitude: %.1f pN\n', amplitude);
    fprintf('  Frequency: %.1f Hz\n', freq);
    fprintf('  Offsets: [%s] pN\n', num2str(offsets));
    fprintf('  Bead Position: [%.1f, %.1f, %.1f] um\n', pos_m(1), pos_m(2), pos_m(3));
    fprintf('  Duration: %.3f s (%d samples)\n', duration, N);

    % ========================================
    % Generate vd Data for Each Offset
    % ========================================
    fprintf('\nGenerating vd data...\n');

    vd_data = cell(num_offsets, 1);
    fd_data = cell(num_offsets, 1);

    for i = 1:num_offsets
        offset = offsets(i);

        % Generate sinusoidal force input in Z direction
        f_d_z = amplitude * sin(2*pi*freq*t) + offset;
        fd_data{i} = f_d_z;

        % Calculate vd for each time sample
        vd = zeros(N, 6);
        for k = 1:N
            f_d = [0; 0; f_d_z(k)];  % Force only in Z direction
            vd(k, :) = inverse_model(f_d, pos_m, params)';
        end
        vd_data{i} = vd;

        fprintf('  Offset = %.1f pN: Force range = [%.1f, %.1f] pN\n', ...
            offset, min(f_d_z), max(f_d_z));
    end

    % ========================================
    % FFT Analysis and THD Calculation
    % ========================================
    fprintf('\nPerforming FFT analysis...\n');

    thd_results = zeros(num_offsets, 6);
    harmonic_mags = cell(num_offsets, 1);

    for i = 1:num_offsets
        vd = vd_data{i};

        % Use steady-state portion (skip first 5 cycles)
        skip_samples = round(5 * fs / freq);
        vd_steady = vd(skip_samples+1:end, :);
        N_steady = size(vd_steady, 1);

        freq_axis = (0:N_steady-1) * fs / N_steady;

        harm_mags = zeros(n_harmonics, 6);

        for ch = 1:6
            vd_fft = fft(vd_steady(:, ch));
            mag_spectrum = abs(vd_fft) * 2 / N_steady;

            % Extract harmonic magnitudes
            for h = 1:n_harmonics
                harm_freq = h * freq;
                [~, harm_bin] = min(abs(freq_axis - harm_freq));
                harm_mags(h, ch) = mag_spectrum(harm_bin);
            end

            % Calculate THD
            V1 = harm_mags(1, ch);
            V_harmonics = harm_mags(2:end, ch);

            if V1 > 1e-12
                thd_results(i, ch) = sqrt(sum(V_harmonics.^2)) / V1 * 100;
            else
                thd_results(i, ch) = NaN;
            end
        end

        harmonic_mags{i} = harm_mags;
    end

    % ========================================
    % Print Results
    % ========================================
    fprintf('\n========================================\n');
    fprintf('THD Results Summary\n');
    fprintf('========================================\n');
    fprintf('%-10s %-15s %-12s\n', 'Offset', 'Force Range', 'THD (Ch5)');
    fprintf('%-10s %-15s %-12s\n', '[pN]', '[pN]', '[%]');
    fprintf('%s\n', repmat('-', 1, 40));

    for i = 1:num_offsets
        f_min = offsets(i) - amplitude;
        f_max = offsets(i) + amplitude;
        fprintf('%-10.1f [%5.1f, %5.1f] %10.2f\n', ...
            offsets(i), f_min, f_max, thd_results(i, 5));
    end

    % ========================================
    % Create Visualizations
    % ========================================
    if show_plots || save_plots
        output_dir = fullfile(script_dir, '..', '..', 'test_results', 'vd_nonlinearity');
        if save_plots && ~exist(output_dir, 'dir')
            mkdir(output_dir);
        end

        colors = lines(num_offsets);

        % Figure 1: Time Domain Comparison
        fig1 = figure('Position', [100, 100, 1200, 800], 'Visible', bool2onoff(show_plots));

        cycles_to_show = 2;
        samples_to_show = round(cycles_to_show * fs / freq);
        t_show = t(1:samples_to_show) * 1000;

        for i = 1:num_offsets
            subplot(num_offsets, 2, (i-1)*2 + 1);
            plot(t_show, fd_data{i}(1:samples_to_show), 'b-', 'LineWidth', 1.5);
            hold on; yline(0, 'k--', 'LineWidth', 0.5); grid on;
            ylabel('f_d [pN]');
            title(sprintf('Force Input (Offset = %.1f pN)', offsets(i)));

            subplot(num_offsets, 2, (i-1)*2 + 2);
            plot(t_show, vd_data{i}(1:samples_to_show, 5), 'r-', 'LineWidth', 1.5);
            hold on; yline(0, 'k--', 'LineWidth', 0.5); grid on;
            ylabel('v_d [V]');
            title(sprintf('vd Ch5 Output (THD = %.1f%%)', thd_results(i, 5)));
        end

        subplot(num_offsets, 2, num_offsets*2 - 1); xlabel('Time [ms]');
        subplot(num_offsets, 2, num_offsets*2); xlabel('Time [ms]');
        sgtitle('Force Input vs vd Output - Time Domain Comparison', 'FontSize', 14, 'FontWeight', 'bold');

        if save_plots
            saveas(fig1, fullfile(output_dir, 'time_domain_comparison.png'));
        end

        % Figure 2: Harmonic Spectrum
        fig2 = figure('Position', [100, 100, 1400, 600], 'Visible', bool2onoff(show_plots));
        ch = 5;

        for i = 1:num_offsets
            subplot(1, num_offsets, i);
            harm = harmonic_mags{i};
            V1 = harm(1, ch);
            harm_dB = 20 * log10(harm(:, ch) / V1);
            harm_dB(1) = 0;

            bar(1:n_harmonics, harm_dB, 'FaceColor', colors(i, :));
            hold on;
            yline(-20, 'r--', 'LineWidth', 1);
            yline(-40, 'g--', 'LineWidth', 1);
            grid on;
            xlabel('Harmonic Order');
            ylabel('Magnitude [dB rel. to fundamental]');
            title(sprintf('Offset = %.1f pN, THD = %.1f%%', offsets(i), thd_results(i, ch)));
            ylim([-60, 5]);
            xticks(1:n_harmonics);
        end

        sgtitle('Harmonic Spectrum Analysis (Channel 5)', 'FontSize', 14, 'FontWeight', 'bold');

        if save_plots
            saveas(fig2, fullfile(output_dir, 'harmonic_spectrum.png'));
        end

        % Figure 3: THD vs Offset
        fig3 = figure('Position', [100, 100, 800, 500], 'Visible', bool2onoff(show_plots));

        subplot(1, 2, 1);
        plot(offsets, thd_results(:, 5), 'bo-', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        hold on;
        yline(10, 'r--', 'LineWidth', 1);
        yline(5, 'g--', 'LineWidth', 1);
        grid on;
        xlabel('Offset [pN]');
        ylabel('THD [%]');
        title('THD vs Force Offset (Channel 5)');
        xlim([-0.5, max(offsets) + 0.5]);

        subplot(1, 2, 2);
        hold on;
        ch_colors = lines(6);
        for ch = 1:6
            plot(offsets, thd_results(:, ch), 'o-', 'LineWidth', 1.5, ...
                'MarkerSize', 8, 'Color', ch_colors(ch, :));
        end
        grid on;
        xlabel('Offset [pN]');
        ylabel('THD [%]');
        title('THD vs Force Offset (All Channels)');
        legend({'Ch1', 'Ch2', 'Ch3', 'Ch4', 'Ch5', 'Ch6'}, 'Location', 'northeast');
        ylim([0, min(100, max(thd_results(:)) * 1.1)]);

        sgtitle('THD Analysis Summary', 'FontSize', 14, 'FontWeight', 'bold');

        if save_plots
            saveas(fig3, fullfile(output_dir, 'thd_vs_offset.png'));
        end

        % Figure 4: Transfer Characteristic
        fig4 = figure('Position', [100, 100, 1000, 400], 'Visible', bool2onoff(show_plots));
        ch = 5;
        selected = [1, ceil(num_offsets/2), num_offsets];

        for idx = 1:length(selected)
            i = selected(idx);
            subplot(1, 3, idx);

            plot(fd_data{i}, vd_data{i}(:, ch), '.', 'MarkerSize', 1);
            hold on;

            fd_range = [min(fd_data{i}), max(fd_data{i})];
            vd_range = [min(vd_data{i}(:, ch)), max(vd_data{i}(:, ch))];
            plot(fd_range, vd_range, 'r--', 'LineWidth', 2);

            grid on;
            xlabel('Force f_d [pN]');
            ylabel('Voltage v_d [V]');
            title(sprintf('Offset = %.1f pN, THD = %.1f%%', offsets(i), thd_results(i, ch)));
        end

        sgtitle('Force-to-Voltage Transfer Characteristic (Channel 5)', 'FontSize', 14, 'FontWeight', 'bold');

        if save_plots
            saveas(fig4, fullfile(output_dir, 'transfer_characteristic.png'));
            fprintf('\nPlots saved to: %s\n', output_dir);
        end
    end

    % ========================================
    % Build Results Structure
    % ========================================
    results = struct();
    results.test_params = struct();
    results.test_params.amplitude = amplitude;
    results.test_params.frequency = freq;
    results.test_params.offsets = offsets;
    results.test_params.fs = fs;
    results.test_params.duration = duration;
    results.test_params.bead_position = pos_m';

    results.thd_results = thd_results;
    results.harmonic_mags = harmonic_mags;
    results.vd_data = vd_data;
    results.fd_data = fd_data;
    results.t = t;

    if save_plots
        save(fullfile(output_dir, 'vd_nonlinearity_analysis.mat'), 'results');
        fprintf('Data saved to: %s\n', fullfile(output_dir, 'vd_nonlinearity_analysis.mat'));
    end

    fprintf('\nAnalysis complete.\n');
end


function s = bool2onoff(b)
% Convert boolean to 'on'/'off' string
    if b
        s = 'on';
    else
        s = 'off';
    end
end
