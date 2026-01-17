function results = fft_analysis(ref_signal, meas_signal, fs, target_freq)
% FFT_ANALYSIS Compute frequency response using FFT
%
% Computes magnitude ratio and phase difference between reference and
% measured signals at a specific target frequency.
%
% Inputs:
%   ref_signal  - Reference signal (N x 1 or N x M for M channels)
%   meas_signal - Measured signal (N x 1 or N x M for M channels)
%   fs          - Sampling frequency [Hz]
%   target_freq - Target frequency for analysis [Hz]
%
% Output:
%   results - Structure containing:
%     .magnitude_ratio - Magnitude ratio |meas/ref| (1 x M or scalar)
%     .magnitude_dB    - Magnitude in dB (1 x M or scalar)
%     .phase_lag_deg   - Phase difference [degrees] (1 x M or scalar)
%     .phase_lag_rad   - Phase difference [radians] (1 x M or scalar)
%     .actual_freq     - Actual frequency bin used [Hz]
%     .freq_error_pct  - Frequency bin error [%]
%     .ref_mag         - Reference signal magnitude at target freq
%     .meas_mag        - Measured signal magnitude at target freq
%
% Example:
%   results = fft_analysis(Vd(:,1), Vm(:,1), 100000, 100);
%   fprintf('Magnitude ratio: %.2f%%, Phase: %.2f deg\n', ...
%           results.magnitude_ratio*100, results.phase_lag_deg);
%
% Author: R-Controller Package Team
% Date: 2025-01-17

    % Ensure column vectors
    if size(ref_signal, 1) < size(ref_signal, 2)
        ref_signal = ref_signal';
    end
    if size(meas_signal, 1) < size(meas_signal, 2)
        meas_signal = meas_signal';
    end

    % Get dimensions
    N = size(ref_signal, 1);
    num_ref_ch = size(ref_signal, 2);
    num_meas_ch = size(meas_signal, 2);

    % Validate input dimensions
    if size(meas_signal, 1) ~= N
        error('Reference and measured signals must have same length');
    end

    % Compute frequency axis
    freq_axis = (0:N-1) * fs / N;

    % Find FFT bin closest to target frequency
    [~, freq_bin] = min(abs(freq_axis - target_freq));
    actual_freq = freq_axis(freq_bin);

    % Compute frequency error
    freq_error_pct = abs(target_freq - actual_freq) / target_freq * 100;

    % FFT of reference signal (use first channel if multi-channel)
    ref_fft = fft(ref_signal(:, 1));
    ref_mag = abs(ref_fft(freq_bin)) * 2 / N;
    ref_phase = angle(ref_fft(freq_bin));

    % Preallocate results
    meas_mag = zeros(1, num_meas_ch);
    meas_phase = zeros(1, num_meas_ch);
    magnitude_ratio = zeros(1, num_meas_ch);
    phase_lag_rad = zeros(1, num_meas_ch);

    % FFT of each measured channel
    for ch = 1:num_meas_ch
        meas_fft = fft(meas_signal(:, ch));
        meas_mag(ch) = abs(meas_fft(freq_bin)) * 2 / N;
        meas_phase(ch) = angle(meas_fft(freq_bin));

        % Compute magnitude ratio
        if ref_mag > 0
            magnitude_ratio(ch) = meas_mag(ch) / ref_mag;
        else
            magnitude_ratio(ch) = NaN;
        end

        % Compute phase lag
        phase_diff = meas_phase(ch) - ref_phase;

        % Normalize to [-pi, pi]
        while phase_diff > pi
            phase_diff = phase_diff - 2*pi;
        end
        while phase_diff < -pi
            phase_diff = phase_diff + 2*pi;
        end

        phase_lag_rad(ch) = phase_diff;
    end

    % Convert to dB and degrees
    magnitude_dB = 20 * log10(magnitude_ratio);
    phase_lag_deg = phase_lag_rad * 180 / pi;

    % Build results structure
    results.magnitude_ratio = magnitude_ratio;
    results.magnitude_dB = magnitude_dB;
    results.phase_lag_deg = phase_lag_deg;
    results.phase_lag_rad = phase_lag_rad;
    results.actual_freq = actual_freq;
    results.freq_error_pct = freq_error_pct;
    results.ref_mag = ref_mag;
    results.meas_mag = meas_mag;
    results.freq_bin = freq_bin;
    results.N_fft = N;

end
