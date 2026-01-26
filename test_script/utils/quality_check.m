function results = quality_check(signal, fs, amplitude, varargin)
% QUALITY_CHECK Perform quality checks on periodic signals
%
% Checks steady-state condition, THD, and DC offset for periodic signals.
%
% Inputs:
%   signal    - Signal data (N x 1 or N x M for M channels)
%   fs        - Sampling frequency [Hz]
%   amplitude - Expected signal amplitude for threshold calculation
%
% Optional Parameters (Name-Value pairs):
%   'Frequency'         - Signal frequency [Hz] (required for steady-state check)
%   'SteadyThreshold'   - Steady-state threshold as fraction of amplitude (default: 0.02)
%   'THDThreshold'      - THD threshold in percent (default: 1.0)
%   'DCTolerance'       - DC tolerance as fraction of amplitude (default: 0.01)
%   'NumHarmonics'      - Number of harmonics for THD calculation (default: 10)
%
% Output:
%   results - Structure containing:
%     .steady_state_pass - Boolean array (1 x M)
%     .thd_values        - THD values in percent (1 x M)
%     .thd_pass          - Boolean array (1 x M)
%     .dc_error          - DC offset values (1 x M)
%     .dc_pass           - Boolean array (1 x M)
%     .overall_pass      - Boolean array (1 x M) - all checks passed
%
% Example:
%   results = quality_check(Vm, 100000, 2.0, 'Frequency', 100);
%   if results.overall_pass(1)
%       fprintf('Channel 1 passed all quality checks\n');
%   end
%
% Author: ZPETC Package Team
% Date: 2025-01-17

    % Parse optional parameters
    p = inputParser;
    addParameter(p, 'Frequency', []);
    addParameter(p, 'SteadyThreshold', 0.02);
    addParameter(p, 'THDThreshold', 1.0);
    addParameter(p, 'DCTolerance', 0.01);
    addParameter(p, 'NumHarmonics', 10);
    parse(p, varargin{:});

    freq = p.Results.Frequency;
    steady_threshold = p.Results.SteadyThreshold;
    thd_threshold = p.Results.THDThreshold;
    dc_tolerance = p.Results.DCTolerance;
    num_harmonics = p.Results.NumHarmonics;

    % Ensure column vectors
    if size(signal, 1) < size(signal, 2)
        signal = signal';
    end

    % Get dimensions
    [N, num_channels] = size(signal);
    Ts = 1 / fs;

    % Initialize results
    results.steady_state_pass = true(1, num_channels);
    results.thd_values = zeros(1, num_channels);
    results.thd_pass = true(1, num_channels);
    results.dc_error = zeros(1, num_channels);
    results.dc_pass = true(1, num_channels);

    % ========================================================================
    % Steady-State Check (Cycle Repeatability)
    % ========================================================================
    if ~isempty(freq)
        period = 1 / freq;
        samples_per_cycle = round(period / Ts);
        num_cycles = floor(N / samples_per_cycle);

        if num_cycles >= 2
            threshold_val = steady_threshold * amplitude;

            for ch = 1:num_channels
                cycle_diffs = [];

                for k = 2:min(num_cycles, 10)  % Check up to 10 cycles
                    idx_start_prev = (k-2) * samples_per_cycle + 1;
                    idx_end_prev = (k-1) * samples_per_cycle;
                    idx_start_curr = (k-1) * samples_per_cycle + 1;
                    idx_end_curr = k * samples_per_cycle;

                    if idx_end_curr <= N
                        cycle_prev = signal(idx_start_prev:idx_end_prev, ch);
                        cycle_curr = signal(idx_start_curr:idx_end_curr, ch);

                        max_diff = max(abs(cycle_curr - cycle_prev));
                        cycle_diffs = [cycle_diffs; max_diff];
                    end
                end

                if ~isempty(cycle_diffs)
                    results.steady_state_pass(ch) = all(cycle_diffs < threshold_val);
                else
                    results.steady_state_pass(ch) = false;
                end
            end
        else
            % Not enough cycles for steady-state check
            results.steady_state_pass = false(1, num_channels);
        end
    end

    % ========================================================================
    % THD Check (Total Harmonic Distortion)
    % ========================================================================
    for ch = 1:num_channels
        try
            thd_dB = thd(signal(:, ch), fs, num_harmonics);
            thd_percent = 10^(thd_dB/20) * 100;
            results.thd_values(ch) = thd_percent;
            results.thd_pass(ch) = (thd_percent < thd_threshold);
        catch
            % THD calculation failed (e.g., signal too poor)
            results.thd_values(ch) = NaN;
            results.thd_pass(ch) = false;
        end
    end

    % ========================================================================
    % DC Offset Check
    % ========================================================================
    for ch = 1:num_channels
        % Use FFT to extract DC component
        signal_fft = fft(signal(:, ch));
        dc_value = abs(signal_fft(1)) / N;

        results.dc_error(ch) = dc_value;
        results.dc_pass(ch) = (dc_value < dc_tolerance * amplitude);
    end

    % ========================================================================
    % Overall Pass (all checks must pass)
    % ========================================================================
    results.overall_pass = results.steady_state_pass & ...
                           results.thd_pass & ...
                           results.dc_pass;

    % Store threshold values for reference
    results.params.steady_threshold = steady_threshold;
    results.params.thd_threshold = thd_threshold;
    results.params.dc_tolerance = dc_tolerance;
    results.params.amplitude = amplitude;
    results.params.frequency = freq;

end
