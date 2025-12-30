% run_frequency_sweep.m
% R Controller 頻率響應測試腳本 - Bode Plot 分析
%
% 功能：
%   1. 掃過多個頻率點（1 Hz ~ 4 kHz, 21 點）
%   2. 測試 d=0 的頻率響應
%   3. 使用 FFT 分析計算增益和相位
%   4. 品質檢測（穩態、THD、DC）
%   5. 繪製 Bode Plot 並標註 -3dB 點
%   6. 儲存結果（.mat 和 .png）

clear; clc; close all;

fprintf('\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('           R Controller 頻率響應測試 (Bode Plot)\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% SECTION 1: 測試配置 

% 添加必要的路徑
script_dir = fileparts(mfilename('fullpath'));
package_root = fullfile(script_dir, '..');
addpath(fullfile(package_root, 'model'));

% 所有頻率都能產生整數的 samples_per_cycle，避免相位漂移問題
frequencies = [1, 10, 50, 100, ...               
               125, 200, 250, 400, 500, ...       
               625, 800, 1000, 1250, 2000, ...    
               2500, 3125, 4000];                 

d_values = [0];

% Vd Generator 設定
signal_type_name = 'sine';
Channel = 1;              % 激勵通道
Amplitude = 0.1;            % 振幅 [V]
Phase = 0;                % 相位 [deg]
SignalType = 1;           % Sine mode

% Controller 參數
T = 1e-5;                 % 採樣時間 [s] (100 kHz)

fB_f = 1000;              
fB_c = 500;              
fB_e = 500;             

% ==================== 計算控制器參數 ====================
% 使用 r_controller_calc_params 計算所有控制器係數
% 此函數會自動創建 Bus Object 並包裝為 Simulink.Parameter
params = r_controller_calc_params(fB_c, fB_e, fB_f);
% ======================================================

% ========== Phase 2 系統參數（Force Model / Inverse Model 用）==========
% 這些參數目前未使用，但在 Phase 2 實現後會需要
R_norm = 550.0;                 % μm, 正規化半徑（LUT 位址計算用）
FGain = 8.0;                    % pN, 力量增益（g_H = g_I）
force_scale = 10.0 / FGain;     % 力量縮放因子（Inverse Model 輸入縮放）
% ======================================================================

% Simulink 參數
Ts = 1e-5;                % 採樣時間 [s] (100 kHz)
solver = 'ode5';          % 固定步長 solver
StepTime = 0;             % Step 時間（不使用）

% 模擬時間設定
total_cycles = 100;       % 總週期數
skip_cycles = 60;         % 跳過暫態週期數
fft_cycles = 40;          % FFT 分析週期數
min_sim_time = 0.1;       % 最小模擬時間 [s]（高頻用）
max_sim_time = Inf;       % 最大模擬時間 [s]（不設限）

% 品質檢測參數
steady_state_threshold = 0.02;  % 穩態檢測閾值 (2% of Amplitude)
thd_threshold = 1.0;            % THD 閾值 (1%)
dc_tolerance = 0.01;            % DC 值容忍度 (1% of Amplitude)
freq_error_threshold = 0.1;     % 頻率誤差警告閾值 (0.1%)

% 輸出設定
test_timestamp = datestr(now, 'yyyymmdd_HHMMSS');
test_folder_name = sprintf('d%d_ch%d_%s', d_values(1), Channel, test_timestamp);
output_dir = fullfile(package_root, 'test_results', 'frequency_response', test_folder_name);

% 模型設定
model_name = 'r_controller_system_integrated';
model_path = fullfile(package_root, 'model', [model_name '.slx']);

%% SECTION 2: 初始化

fprintf('【測試配置】\n');
fprintf('────────────────────────\n');
fprintf('  頻率範圍: %.1f Hz ~ %.1f kHz\n', frequencies(1), frequencies(end)/1000);
fprintf('  激勵通道: P%d\n', Channel);
fprintf('  振幅: %.2f V\n', Amplitude);
fprintf('  前饋濾波器頻寬: %.1f kHz\n', fB_f/1000);
fprintf('  控制器頻寬: %.1f kHz\n', fB_c/1000);
fprintf('  估測器頻寬: %.1f kHz\n', fB_e/1000);
fprintf('  總週期數: %d (跳過 %d, 分析 %d)\n', total_cycles, skip_cycles, fft_cycles);
fprintf('  Solver: %s (固定步長)\n', solver);
fprintf('\n');

% 取得 b 參數值用於理論曲線計算
b_value = params.Value.b;
fprintf('  理論模型參數 b: %.4f\n', b_value);
fprintf('\n');

% 檢查模型
if ~exist(model_path, 'file')
    error('找不到模型檔案: %s', model_path);
end

% 創建輸出目錄
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
    fprintf('  ✓ 已創建輸出目錄: %s\n', output_dir);
else
    fprintf('  ✓ 輸出目錄已存在\n');
end

% 創建診斷圖目錄
diagnostic_dir = fullfile(output_dir, 'diagnostics');
if ~exist(diagnostic_dir, 'dir')
    mkdir(diagnostic_dir);
end

% 載入模型（不開啟 GUI）
if ~bdIsLoaded(model_name)
    load_system(model_path);
end
fprintf('  ✓ 模型已載入\n');

% 計算 lambda 參數
lambda_c = exp(-fB_c*T*2*pi);
lambda_e = exp(-fB_e*T*2*pi);
beta = sqrt(lambda_e * lambda_c);

fprintf('\n');

%%  SECTION 3: 頻率掃描主迴圈 

% 初始化結果結構
num_d = length(d_values);
num_freq = length(frequencies);

for d_idx = 1:num_d
    d = d_values(d_idx);

    fprintf('════════════════════════════════════════════════════════════\n');
    fprintf('  開始測試 d = %d\n', d);
    fprintf('════════════════════════════════════════════════════════════\n');
    fprintf('\n');

    % 初始化此 d 值的結果矩陣
    magnitude_ratio_all = zeros(num_freq, 6);
    phase_lag_all = zeros(num_freq, 6);
    sim_times = zeros(num_freq, 1);

    % 初始化品質檢測結果矩陣
    quality_steady_state = true(num_freq, 6);  % 穩態檢測結果
    quality_thd = zeros(num_freq, 6);          % THD 值 (%)
    quality_dc_error = zeros(num_freq, 6);     % DC 誤差 (V)
    quality_thd_pass = true(num_freq, 6);      % THD 檢測通過
    quality_dc_pass = true(num_freq, 6);       % DC 檢測通過

    % 頻率掃描
    for freq_idx = 1:num_freq
        Frequency = frequencies(freq_idx);
        period = 1 / Frequency;

        % 計算模擬時間
        sim_time = total_cycles * period;
        sim_time = max(min_sim_time, min(sim_time, max_sim_time));
        sim_times(freq_idx) = sim_time;

        fprintf('────────────────────────────────────────────────────────\n');
        fprintf('[%2d/%2d] 測試頻率: %8.2f Hz (週期: %.4f s, 模擬: %.2f s)\n', ...
                freq_idx, num_freq, Frequency, period, sim_time);
        fprintf('        當前 d 值: %d\n', d);
        fprintf('────────────────────────────────────────────────────────\n');

        % 設定 Simulink 模擬參數
        set_param(model_name, 'StopTime', num2str(sim_time));
        set_param(model_name, 'Solver', solver);
        set_param(model_name, 'FixedStep', num2str(Ts));

        % 執行模擬
        fprintf('  ⏳ 執行模擬中（d=%d）...\n', d);
        tic;
        try
            out = sim(model_name);
            elapsed = toc;
            fprintf('  ✓ 模擬完成 (耗時 %.2f 秒)\n', elapsed);
        catch ME
            fprintf('  ✗ 模擬失敗: %s\n', ME.message);
            continue;
        end

        % 提取數據
        try
            Vd_data = out.Vd;
            Vm_data = out.Vm;

            N = size(Vd_data, 1);
            t = (0:N-1)' * Ts;

            fprintf('  ✓ 數據提取完成 (數據點: %d, 時間: %.2f s)\n', N, t(end));
        catch ME
            fprintf('  ✗ 數據提取失敗: %s\n', ME.message);
            continue;
        end

        % 選取穩態數據（跳過前 skip_cycles 個週期）
        skip_time = skip_cycles * period;
        fft_time = fft_cycles * period;

        t_start = skip_time;
        t_end = min(skip_time + fft_time, t(end));

        idx_steady = (t >= t_start) & (t <= t_end);

        if sum(idx_steady) < 100
            fprintf('  ✗ 穩態數據點不足 (%d 點)，跳過此頻率\n', sum(idx_steady));
            continue;
        end

        Vd_steady = Vd_data(idx_steady, :);
        Vm_steady = Vm_data(idx_steady, :);
        t_steady = t(idx_steady);

        actual_cycles = (t_end - t_start) / period;
        fprintf('  ✓ 穩態數據選取: %.2f ~ %.2f s (%.1f 個週期, %d 點)\n', ...
                t_start, t_end, actual_cycles, sum(idx_steady));

        %% ========== 新增：品質檢測 ==========

        % === 1. 穩態檢測：檢查週期重複性 ===
        fprintf('  🔍 執行品質檢測...\n');

        samples_per_cycle = round(period / Ts);
        num_cycles_to_check = min(fft_cycles, floor(length(t_steady) / samples_per_cycle));

        for ch = 1:6
            % 提取每個週期的數據
            cycle_diffs = [];

            for k = 2:num_cycles_to_check
                idx_start_prev = (k-2) * samples_per_cycle + 1;
                idx_end_prev = (k-1) * samples_per_cycle;
                idx_start_curr = (k-1) * samples_per_cycle + 1;
                idx_end_curr = k * samples_per_cycle;

                if idx_end_curr <= length(Vm_steady(:, ch))
                    cycle_prev = Vm_steady(idx_start_prev:idx_end_prev, ch);
                    cycle_curr = Vm_steady(idx_start_curr:idx_end_curr, ch);

                    % 計算相鄰週期的最大差異
                    max_diff = max(abs(cycle_curr - cycle_prev));
                    cycle_diffs = [cycle_diffs; max_diff];
                end
            end

            % 判斷穩態（所有週期差異都要小於閾值）
            threshold = steady_state_threshold * Amplitude;
            if ~isempty(cycle_diffs)
                quality_steady_state(freq_idx, ch) = all(cycle_diffs < threshold);

                % 如果未達穩態，保存診斷圖
                if ~quality_steady_state(freq_idx, ch)
                    % 生成週期疊圖
                    fig_diag = figure('Visible', 'off', 'Position', [100, 100, 800, 600]);
                    hold on; grid on;

                    for k = 1:num_cycles_to_check
                        idx_start = (k-1) * samples_per_cycle + 1;
                        idx_end = k * samples_per_cycle;

                        if idx_end <= length(Vm_steady(:, ch))
                            cycle_data = Vm_steady(idx_start:idx_end, ch);
                            t_cycle = (0:length(cycle_data)-1)' * Ts * 1000;  % ms

                            % 使用顏色漸層表示時間順序
                            color_intensity = (k-1) / (num_cycles_to_check-1);
                            plot(t_cycle, cycle_data, 'LineWidth', 1.5, ...
                                 'Color', [color_intensity, 0, 1-color_intensity]);
                        end
                    end

                    xlabel('Time within Cycle [ms]', 'FontSize', 12, 'FontWeight', 'bold');
                    ylabel('Vm [V]', 'FontSize', 12, 'FontWeight', 'bold');
                    title(sprintf('Cycle Overlay - %.1f Hz, P%d (NOT STEADY)', Frequency, ch), ...
                          'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');

                    % 添加圖例說明
                    colormap(jet(num_cycles_to_check));
                    cb = colorbar;
                    cb.Label.String = 'Cycle Number';
                    caxis([1, num_cycles_to_check]);

                    % 保存診斷圖（300 DPI）
                    diag_filename = sprintf('steady_fail_%.1fHz_P%d.png', Frequency, ch);
                    exportgraphics(fig_diag, fullfile(diagnostic_dir, diag_filename), 'Resolution', 300);
                    close(fig_diag);
                end
            else
                quality_steady_state(freq_idx, ch) = false;
            end
        end

        % === 2. THD 和 DC 值檢測 ===
        fs = 1 / Ts;

        for ch = 1:6
            % FFT 分析（用於 DC 檢測）
            Vm_fft_temp = fft(Vm_steady(:, ch));
            N_fft_temp = length(Vm_fft_temp);

            % DC 成分
            DC_value = abs(Vm_fft_temp(1)) / N_fft_temp;
            DC_target = 0;  % 純正弦波應該沒有 DC
            quality_dc_error(freq_idx, ch) = abs(DC_value - DC_target);
            quality_dc_pass(freq_idx, ch) = (quality_dc_error(freq_idx, ch) < dc_tolerance * Amplitude);

            % THD 計算
            try
                thd_dB = thd(Vm_steady(:, ch), fs, 10);
                thd_percent = 10^(thd_dB/20) * 100;
                quality_thd(freq_idx, ch) = thd_percent;
                quality_thd_pass(freq_idx, ch) = (thd_percent < thd_threshold);
            catch
                % 如果 THD 計算失敗（信號太差）
                quality_thd(freq_idx, ch) = NaN;
                quality_thd_pass(freq_idx, ch) = false;
            end
        end

        % === 3. 顯示品質檢測結果 ===
        fprintf('  ✓ 品質檢測完成\n');
        fprintf('    通道 | 穩態 | THD          | DC誤差  | 狀態\n');
        fprintf('    ─────┼──────┼──────────────┼─────────┼──────\n');

        for ch = 1:6
            steady_mark = '✓';
            if ~quality_steady_state(freq_idx, ch)
                steady_mark = '✗';
            end

            thd_mark = '✓';
            if ~quality_thd_pass(freq_idx, ch)
                thd_mark = '✗';
            end

            dc_mark = '✓';
            if ~quality_dc_pass(freq_idx, ch)
                dc_mark = '✗';
            end

            % 整體狀態判斷
            if ch == Channel
                % 激勵通道：必須全部通過
                if quality_steady_state(freq_idx, ch) && quality_thd_pass(freq_idx, ch) && quality_dc_pass(freq_idx, ch)
                    status = 'PASS';
                else
                    status = 'WARN';
                end
            else
                % 其他通道：標記但不影響 FFT
                if quality_steady_state(freq_idx, ch) && quality_thd_pass(freq_idx, ch) && quality_dc_pass(freq_idx, ch)
                    status = 'OK';
                else
                    status = 'FAIL';
                end
            end

            % THD 動態格式顯示
            thd_val = quality_thd(freq_idx, ch);
            if isnan(thd_val)
                thd_str = '    N/A    ';
            elseif thd_val < 0.01
                thd_str = sprintf('%10.2e%%', thd_val);
            else
                thd_str = sprintf('%10.4f%%', thd_val);
            end

            fprintf('     P%d  |  %s   | %s %s | %.4fV %s | %s\n', ...
                    ch, steady_mark, thd_str, thd_mark, ...
                    quality_dc_error(freq_idx, ch), dc_mark, status);
        end

        fprintf('\n');

        %% ========== 品質檢測結束 ==========

        % FFT 分析
        fprintf('  📊 執行 FFT 分析...\n');

        N_fft = length(Vd_steady);
        fs = 1 / Ts;
        freq_axis = (0:N_fft-1) * fs / N_fft;

        % 找到激勵頻率對應的 bin
        [~, freq_bin_idx] = min(abs(freq_axis - Frequency));
        actual_freq = freq_axis(freq_bin_idx);

        % 頻率誤差檢查
        freq_error = abs(Frequency - actual_freq);
        freq_error_percent = (freq_error / Frequency) * 100;

        fprintf('    目標頻率: %.2f Hz\n', Frequency);
        fprintf('    FFT bin:  %.2f Hz (誤差: %.4f Hz, %.3f%%)\n', ...
                actual_freq, freq_error, freq_error_percent);

        if freq_error_percent > freq_error_threshold
            fprintf('    ⚠️ 警告：頻率誤差 %.3f%% > %.3f%%\n', ...
                    freq_error_percent, freq_error_threshold);
        end

        % 對激勵通道的 Vd 做 FFT
        Vd_fft = fft(Vd_steady(:, Channel));
        Vd_mag = abs(Vd_fft(freq_bin_idx)) * 2 / N_fft;
        Vd_phase = angle(Vd_fft(freq_bin_idx)) * 180 / pi;

        % 對所有 Vm 通道做 FFT
        for ch = 1:6
            Vm_fft = fft(Vm_steady(:, ch));
            Vm_mag = abs(Vm_fft(freq_bin_idx)) * 2 / N_fft;
            Vm_phase = angle(Vm_fft(freq_bin_idx)) * 180 / pi;

            % 計算頻率響應
            magnitude_ratio_all(freq_idx, ch) = Vm_mag / Vd_mag;
            phase_lag_all(freq_idx, ch) = Vm_phase - Vd_phase;

            % 相位正規化到 [-180, 180]
            while phase_lag_all(freq_idx, ch) > 180
                phase_lag_all(freq_idx, ch) = phase_lag_all(freq_idx, ch) - 360;
            end
            while phase_lag_all(freq_idx, ch) < -180
                phase_lag_all(freq_idx, ch) = phase_lag_all(freq_idx, ch) + 360;
            end
        end

        fprintf('  ✓ FFT 完成 (頻率 bin: %.2f Hz, P%d 增益: %.2f%%)\n', ...
                actual_freq, Channel, magnitude_ratio_all(freq_idx, Channel)*100);
        fprintf('\n');
    end

    % ========== 計算新的理論值（A(θ; λf, b) 和 φ(θ; λf, b)）==========
    % 提取理論曲線所需參數
    lambda_f = params.Value.lambda_f;
    kf = (1 - lambda_f) / (1 + b_value);

    % 定義密集頻率點（讓理論曲線平滑）
    freq_theory = logspace(0, log10(4000), 500);  % 1 Hz ~ 4000 Hz，500 點

    % 計算新理論公式
    A_theory = zeros(size(freq_theory));
    phi_theory = zeros(size(freq_theory));

    for i = 1:length(freq_theory)
        theta = 2*pi*freq_theory(i)*Ts;  % θ = ωT

        % A(θ; λf, b) - 振幅函數
        A_theory(i) = kf * sqrt((1 + 2*b_value*cos(theta) + b_value^2) / ...
                                (1 - 2*lambda_f*cos(theta) + lambda_f^2));

        % φ(θ; λf, b) - 相位函數（弧度）
        phi_theory(i) = -(theta + ...
                          atan(b_value*sin(theta)/(1 + b_value*cos(theta))) + ...
                          atan(lambda_f*sin(theta)/(1 - lambda_f*cos(theta))));
    end

    % 轉換相位為度數
    phi_theory_deg = phi_theory * (180/pi);

    % 為相容性保留 H_theory_save（使用實驗頻率點）
    H_theory_save = zeros(size(frequencies));
    for i = 1:length(frequencies)
        theta = 2*pi*frequencies(i)*Ts;
        H_theory_save(i) = kf * sqrt((1 + 2*b_value*cos(theta) + b_value^2) / ...
                                     (1 - 2*lambda_f*cos(theta) + lambda_f^2));
    end

    % 儲存此 d 值的結果
    results(d_idx).d_value = d;
    results(d_idx).frequencies = frequencies;
    results(d_idx).magnitude_ratio = magnitude_ratio_all;
    results(d_idx).phase_lag = phase_lag_all;
    results(d_idx).magnitude_dB = 20 * log10(magnitude_ratio_all);
    results(d_idx).sim_times = sim_times;
    results(d_idx).Channel = Channel;
    results(d_idx).fB_c = fB_c;
    results(d_idx).fB_e = fB_e;

    % 理論值和誤差分析
    results(d_idx).theory.b_value = b_value;
    results(d_idx).theory.H_magnitude = H_theory_save;
    results(d_idx).theory.H_magnitude_dB = 20 * log10(H_theory_save);
    results(d_idx).theory.error_percent = abs(magnitude_ratio_all(:, Channel) - H_theory_save') ./ H_theory_save' * 100;
    results(d_idx).theory.max_error_percent = max(results(d_idx).theory.error_percent);
    results(d_idx).theory.mean_error_percent = mean(results(d_idx).theory.error_percent);
    results(d_idx).theory.rms_error_percent = sqrt(mean(results(d_idx).theory.error_percent.^2));

    % 品質檢測結果
    results(d_idx).quality.steady_state = quality_steady_state;
    results(d_idx).quality.thd = quality_thd;
    results(d_idx).quality.dc_error = quality_dc_error;
    results(d_idx).quality.thd_pass = quality_thd_pass;
    results(d_idx).quality.dc_pass = quality_dc_pass;

    fprintf('════════════════════════════════════════════════════════════\n');
    fprintf('  d = %d 測試完成！\n', d);
    fprintf('════════════════════════════════════════════════════════════\n');
    fprintf('\n\n');
end

%%  SECTION 4: 繪製 Bode Plot 

fprintf('【繪製 Bode Plot】\n');
fprintf('────────────────────────\n');

% 顏色設定
colors_d = [
    0.0000, 0.4470, 0.7410;  % d=0: 藍色
    0.8500, 0.3250, 0.0980;  % d=2: 橘色
];

channel_colors = [
    0.0000, 0.0000, 0.5000;  % P1: 深藍色
    0.0000, 0.0000, 1.0000;  % P2: 藍色
    0.0000, 0.5000, 0.0000;  % P3: 綠色
    1.0000, 0.0000, 0.0000;  % P4: 紅色
    0.8000, 0.0000, 0.8000;  % P5: 粉紫色
    0.0000, 0.7500, 0.7500;  % P6: 青色
];

% 定義每個通道的標記形狀
markers = {'o', 's', '^', 'd', 'v', 'p'};  % P1-P6: 圓形、方形、上三角、菱形、下三角、五角星

% === 圖 1 & 2: 各 d 值的所有通道響應 ===
for d_idx = 1:num_d
    d = results(d_idx).d_value;

    fig = figure('Name', sprintf('Frequency Response fB_f=%.0fHz (Ch P%d)', fB_f, Channel), ...
                 'Position', [100+50*d_idx, 100+50*d_idx, 1200, 800]);

    % 計算線性增益比（不是 dB）
    magnitude_ratio = results(d_idx).magnitude_ratio;

    % ===== 上圖：Magnitude（線性刻度 0~1.25，所有通道）=====
    % 手動設定位置：[left, bottom, width, height]
    % 為圖例預留上方空間（0.1），兩個子圖平分剩餘空間
    subplot('Position', [0.1, 0.55, 0.85, 0.35]);
    hold on; grid off;  % 取消背景網格線

    % 線寬設定（統一粗細）
    unified_linewidth = 3.5;  % 所有通道統一線寬
    unified_markersize = 9;   % 統一標記大小

    % 定義理論曲線顏色
    theory_color = [0.5, 0.5, 0.5];  % 中灰色（50% 亮度）

    % === 先畫理論曲線（底層，實線）===
    if d == 0
        plot_handle_theory = semilogx(freq_theory, A_theory, '-', ...
                                       'LineWidth', unified_linewidth, ...
                                       'Color', theory_color, ...
                                       'DisplayName', 'Theory');
        hold on;
    end

    % 儲存 plot handles 以便圖例對應
    plot_handles_mag = gobjects(6, 1);

    % === 再畫實驗曲線（上層，虛線）===
    for ch = 1:6
        mag = magnitude_ratio(:, ch);

        if ch == Channel
            % 激發通道（改為虛線）
            plot_handles_mag(ch) = semilogx(frequencies, mag, ['--' markers{ch}], ...
                     'LineWidth', unified_linewidth, ...
                     'Color', channel_colors(ch, :), ...
                     'MarkerFaceColor', 'none', ...
                     'MarkerEdgeColor', channel_colors(ch, :), ...
                     'MarkerSize', unified_markersize, ...
                     'DisplayName', sprintf('P%d (Excited)', ch));
        else
            % 非激發通道（改為虛線）
            plot_handles_mag(ch) = semilogx(frequencies, mag, ['--' markers{ch}], ...
                     'LineWidth', unified_linewidth, ...
                     'Color', channel_colors(ch, :), ...
                     'MarkerFaceColor', 'none', ...
                     'MarkerEdgeColor', channel_colors(ch, :), ...
                     'MarkerSize', unified_markersize, ...
                     'DisplayName', sprintf('P%d', ch));
        end
        hold on;
    end

    % 設定 Y 軸範圍和刻度
    ylim([0, 1.25]);
    yticks([0, 0.25, 0.5, 0.75, 1.0, 1.25]);

    % 移除 X 軸標籤和標題
    ylabel('Magnitude', 'FontSize', 22, 'FontWeight', 'bold');  % 字體更大
    % title('Frequency Response', 'FontSize', 18, 'FontWeight', 'bold');  % 移除標題

    xlim([frequencies(1), frequencies(end)]);

    % 設定座標軸格式
    ax1 = gca;
    ax1.XScale = 'log';
    ax1.XTick = [1, 10, 100, 1000, 10000];
    ax1.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
    ax1.FontSize = 18;  % 刻度數字更大
    ax1.FontWeight = 'bold';
    ax1.LineWidth = 2.5;  % 座標軸線加粗
    ax1.Box = 'on';  % 保留完整框線

    % ===== 下圖：Phase（只顯示激勵通道）=====
    % 手動設定位置，與 Magnitude 圖高度相同
    subplot('Position', [0.1, 0.1, 0.85, 0.35]);
    hold on; grid off;  % 取消背景網格線

    % === 先畫理論相位曲線（底層，實線）===
    if d == 0
        plot_handle_theory_phase = semilogx(freq_theory, phi_theory_deg, '-', ...
                                            'LineWidth', unified_linewidth, ...
                                            'Color', theory_color, ...
                                            'DisplayName', 'Theory');
        hold on;
    end

    % === 再畫實驗相位曲線（激發通道，虛線）===
    phase_ch = results(d_idx).phase_lag(:, Channel);

    plot_handles_phase = semilogx(frequencies, phase_ch, ['--' markers{Channel}], ...
             'LineWidth', unified_linewidth, ...
             'Color', channel_colors(Channel, :), ...
             'MarkerSize', unified_markersize, ...
             'MarkerFaceColor', 'none', ...
             'MarkerEdgeColor', channel_colors(Channel, :), ...
             'DisplayName', sprintf('P%d (Excited)', Channel));

    xlabel('Frequency (Hz)', 'FontSize', 22, 'FontWeight', 'bold');  % 字體更大
    ylabel('Phase (deg)', 'FontSize', 22, 'FontWeight', 'bold');  % 字體更大
    xlim([frequencies(1), frequencies(end)]);

    % 設定座標軸格式
    ax2 = gca;
    ax2.XScale = 'log';
    ax2.XTick = [1, 10, 100, 1000, 10000];
    ax2.XTickLabel = {'10^0', '10^1', '10^2', '10^3', '10^4'};
    ax2.FontSize = 18;  % 刻度數字更大
    ax2.FontWeight = 'bold';
    ax2.LineWidth = 2.5;  % 座標軸線加粗
    ax2.Box = 'on';  % 保留完整框線

    % Phase 圖不顯示圖例（依照使用者要求）

    % 在兩個子圖都完成後，統一添加圖例到 Magnitude 圖上方
    % 回到 Magnitude 子圖
    subplot('Position', [0.1, 0.55, 0.85, 0.35]);

    % 添加圖例（Theory 在最後，單列排開）
    if d == 0
        legend_labels = {'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'Theory'};
        leg_mag = legend([plot_handles_mag; plot_handle_theory], legend_labels, ...
               'Location', 'northoutside', 'NumColumns', 7, 'FontSize', 13, 'FontWeight', 'bold', ...
               'Orientation', 'horizontal');
    else
        leg_mag = legend(plot_handles_mag, {'P1', 'P2', 'P3', 'P4', 'P5', 'P6'}, ...
               'Location', 'northoutside', 'NumColumns', 6, 'FontSize', 13, 'FontWeight', 'bold', ...
               'Orientation', 'horizontal');
    end
    leg_mag.EdgeColor = [0 0 0];  % 黑色外框
    leg_mag.LineWidth = 2.0;  % 圖例框線加粗

    % 不強制恢復位置，讓 MATLAB 自動調整子圖為圖例留出空間

    fprintf('  ✓ 圖 %d: d=%d 完成\n', d_idx, d);
end

% === 圖 3: 相位對比（d=0 vs d=2，單圖）===
if num_d == 2
    fig_compare = figure('Name', 'Phase Comparison (d=0 vs d=2)', ...
                         'Position', [200, 200, 1200, 600]);

    % 提取兩個 d 值的相位
    phase_d0 = results(1).phase_lag(:, Channel);
    phase_d2 = results(2).phase_lag(:, Channel);

    % ===== 相位對比曲線（單圖）=====
    hold on; grid off;

    % 使用更鮮明的顏色和更粗的線
    semilogx(frequencies, phase_d0, '-o', 'LineWidth', 3.5, ...
             'Color', [0, 0.4470, 0.7410], 'MarkerSize', 8, ...
             'MarkerFaceColor', [0, 0.4470, 0.7410], ...
             'DisplayName', sprintf('P%d (d=0)', Channel));

    semilogx(frequencies, phase_d2, '-s', 'LineWidth', 3.5, ...
             'Color', [0.8500, 0.3250, 0.0980], 'MarkerSize', 8, ...
             'MarkerFaceColor', [0.8500, 0.3250, 0.0980], ...
             'DisplayName', sprintf('P%d (d=2)', Channel));

    xlabel('Frequency [Hz]', 'FontSize', 14);
    ylabel('Phase [deg]', 'FontSize', 14);
    title(sprintf('Phase Comparison - P%d (d=0 vs d=2)', Channel), ...
          'FontSize', 16, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 13);
    xlim([frequencies(1), frequencies(end)]);
    ylim([-50, 0]);  % 限定 Y 軸範圍：-50° ~ 0°

    % 設定 X 軸刻度為 10^n 格式
    set(gca, 'XScale', 'log');
    set(gca, 'XTick', [1, 10, 100, 1000, 10000]);
    set(gca, 'XTickLabel', {'10^0', '10^1', '10^2', '10^3', '10^4'});
    set(gca, 'FontSize', 12);

    fprintf('  ✓ 圖 3: 相位對比完成\n');
end

fprintf('\n');

%% SECTION 5: 分析與顯示結果 

fprintf('【頻率響應分析結果】\n');
fprintf('════════════════════════════════════════════════════════════\n');

for d_idx = 1:num_d
    d = results(d_idx).d_value;
    mag_dB = results(d_idx).magnitude_dB(:, Channel);

    fprintf('\n[ d = %d ]\n', d);
    fprintf('────────────────────────────────────────────────────────\n');

    % 找 -3dB 頻寬（修正版：使用內插）
    idx_below_3dB = find(mag_dB < -3, 1, 'first');
    if ~isempty(idx_below_3dB) && idx_below_3dB > 1
        % 線性內插計算精確的 -3dB 頻率
        idx_above = idx_below_3dB - 1;
        f1 = frequencies(idx_above);
        f2 = frequencies(idx_below_3dB);
        mag_dB1 = mag_dB(idx_above);
        mag_dB2 = mag_dB(idx_below_3dB);

        f_3dB = f1 + (f2 - f1) * (-3 - mag_dB1) / (mag_dB2 - mag_dB1);
        fprintf('  -3dB 頻寬: %.2f Hz (內插計算)\n', f_3dB);
    elseif ~isempty(idx_below_3dB)
        % 第一個點就 < -3dB（異常）
        fprintf('  -3dB 頻寬: < %.2f Hz (第一個測試點)\n', frequencies(1));
    else
        fprintf('  -3dB 頻寬: > %.2f Hz (未達到)\n', frequencies(end));
    end

    % 低頻增益（最低測試頻率，近似 DC 增益）
    dc_gain_dB = mag_dB(1);
    fprintf('  低頻增益 (%.1f Hz): %.2f dB (%.2f%%) [近似 DC]\n', ...
            frequencies(1), dc_gain_dB, 10^(dc_gain_dB/20)*100);

    % 高頻增益（最高頻）
    hf_gain_dB = mag_dB(end);
    fprintf('  高頻增益 (%.1f Hz): %.2f dB (%.2f%%)\n', ...
            frequencies(end), hf_gain_dB, 10^(hf_gain_dB/20)*100);

    % 最大增益
    [max_gain_dB, max_idx] = max(mag_dB);
    fprintf('  最大增益: %.2f dB at %.2f Hz\n', max_gain_dB, frequencies(max_idx));

    % 相位統計
    phase_ch = results(d_idx).phase_lag(:, Channel);
    fprintf('\n  相位範圍: %.2f° ~ %.2f°\n', min(phase_ch), max(phase_ch));
    fprintf('  平均相位: %.2f°\n', mean(phase_ch));

    % 理論對比統計
    fprintf('\n【理論對比分析 (b = %.4f)】\n', results(d_idx).theory.b_value);
    fprintf('  最大誤差: %.2f%% @ %.1f Hz\n', ...
            results(d_idx).theory.max_error_percent, ...
            frequencies(find(results(d_idx).theory.error_percent == results(d_idx).theory.max_error_percent, 1)));
    fprintf('  平均誤差: %.2f%%\n', results(d_idx).theory.mean_error_percent);
    fprintf('  RMS 誤差: %.2f%%\n', results(d_idx).theory.rms_error_percent);

    % 找出誤差最小的頻率
    [min_error, min_idx] = min(results(d_idx).theory.error_percent);
    fprintf('  最小誤差: %.2f%% @ %.1f Hz\n', min_error, frequencies(min_idx));

    fprintf('\n');
end

fprintf('════════════════════════════════════════════════════════════\n');

%% 品質統計報告
fprintf('\n【品質檢測統計】\n');
fprintf('════════════════════════════════════════════════════════════\n');

% 統計各通道的通過率（只針對 d=0）
d_idx = 1;
for ch = 1:6
    steady_pass_count = sum(results(d_idx).quality.steady_state(:, ch));
    thd_pass_count = sum(results(d_idx).quality.thd_pass(:, ch));
    dc_pass_count = sum(results(d_idx).quality.dc_pass(:, ch));

    steady_pass_rate = steady_pass_count / num_freq * 100;
    thd_pass_rate = thd_pass_count / num_freq * 100;
    dc_pass_rate = dc_pass_count / num_freq * 100;

    overall_pass = sum(results(d_idx).quality.steady_state(:, ch) & ...
                       results(d_idx).quality.thd_pass(:, ch) & ...
                       results(d_idx).quality.dc_pass(:, ch));
    overall_pass_rate = overall_pass / num_freq * 100;

    fprintf('\n【P%d】\n', ch);
    if ch == Channel
        fprintf('  (激勵通道)\n');
    end
    fprintf('  穩態檢測通過率: %d/%d (%.1f%%)\n', steady_pass_count, num_freq, steady_pass_rate);
    fprintf('  THD 檢測通過率: %d/%d (%.1f%%)\n', thd_pass_count, num_freq, thd_pass_rate);
    fprintf('  DC 檢測通過率:  %d/%d (%.1f%%)\n', dc_pass_count, num_freq, dc_pass_rate);
    fprintf('  整體通過率:     %d/%d (%.1f%%)\n', overall_pass, num_freq, overall_pass_rate);

    % THD 統計
    valid_thd = results(d_idx).quality.thd(~isnan(results(d_idx).quality.thd(:, ch)), ch);
    if ~isempty(valid_thd)
        fprintf('  THD 平均值: %.2f%% (最大: %.2f%%, 最小: %.2f%%)\n', ...
                mean(valid_thd), max(valid_thd), min(valid_thd));
    end

    % DC 誤差統計
    fprintf('  DC 誤差平均: %.4f V (最大: %.4f V)\n', ...
            mean(results(d_idx).quality.dc_error(:, ch)), max(results(d_idx).quality.dc_error(:, ch)));
end

fprintf('\n════════════════════════════════════════════════════════════\n');

% 顯示相位差統計（如果有 d=0 和 d=2）
if num_d == 2
    phase_d0 = results(1).phase_lag(:, Channel);
    phase_d2 = results(2).phase_lag(:, Channel);
    delta_phase = phase_d0 - phase_d2;

    fprintf('\n【相位差統計 (d=0 vs d=2)】\n');
    fprintf('────────────────────────────────────────────────────────\n');

    mean_delta = mean(delta_phase);
    max_delta = max(delta_phase);
    min_delta = min(delta_phase);

    [~, max_idx] = max(delta_phase);
    [~, min_idx] = min(delta_phase);

    fprintf('  平均 Δphase: %.2f°\n', mean_delta);
    fprintf('  最大 Δphase: %.2f° (at %.2f Hz)\n', max_delta, frequencies(max_idx));
    fprintf('  最小 Δphase: %.2f° (at %.2f Hz)\n', min_delta, frequencies(min_idx));
    fprintf('\n');

    if mean_delta < 0
        fprintf('  → d=0 的相位平均比 d=2 更負 %.2f°\n', abs(mean_delta));
    else
        fprintf('  → d=0 的相位平均比 d=2 更正 %.2f°\n', mean_delta);
    end

    fprintf('\n');
end

fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% SECTION 6: 儲存結果 

fprintf('【儲存結果】\n');
fprintf('────────────────────────\n');

% 檔案命名（簡化，因為已經在專屬資料夾中）
mat_filename = 'freq_sweep_data.mat';
png_bode_filename = 'bode_plot.png';

mat_path = fullfile(output_dir, mat_filename);
png_bode_path = fullfile(output_dir, png_bode_filename);

% 保存 .mat 檔案
save(mat_path, 'results', '-v7.3');
fprintf('  ✓ 數據已保存: %s\n', mat_filename);

% 保存 Bode Plot（300 DPI 高解析度）
export_resolution = 300;  % DPI
exportgraphics(figure(1), png_bode_path, 'Resolution', export_resolution);
fprintf('  ✓ Bode Plot 已保存: %s (%d DPI)\n', png_bode_filename, export_resolution);

fprintf('\n  📁 所有檔案保存至: %s\n', output_dir);
fprintf('  📁 診斷圖保存至: %s\n', diagnostic_dir);
fprintf('\n');

%% SECTION 7: 測試總結

fprintf('════════════════════════════════════════════════════════════\n');
fprintf('                     測試完成\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');

fprintf('【測試摘要】\n');
fprintf('  控制器: R Controller\n');
fprintf('  d 值: %d (測試版本)\n', d_values(1));
fprintf('  激勵通道: P%d\n', Channel);
fprintf('  控制器頻寬: %.1f kHz\n', fB_c/1000);
fprintf('  估測器頻寬: %.1f kHz\n', fB_e/1000);
fprintf('  頻率範圍: %.1f ~ %.1f Hz (%d 點)\n', ...
        frequencies(1), frequencies(end), num_freq);
fprintf('  總模擬時間: %.2f 分鐘\n', sum(sim_times)/60);
fprintf('  輸出位置: %s\n', output_dir);
fprintf('\n');

fprintf('測試腳本執行完畢！\n\n');
