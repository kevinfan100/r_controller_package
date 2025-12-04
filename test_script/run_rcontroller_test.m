% run_rcontroller_test.m

clear; clc; close all;

% 添加必要的路徑
script_dir_temp = fileparts(mfilename('fullpath'));
package_root_temp = fullfile(script_dir_temp, '..');
addpath(fullfile(package_root_temp, 'model'));

%% SECTION 1: 配置區域

test_name = 'test';    % 測試名稱（用於檔案命名）

%Vd Generator
signal_type_name = 'sine';      % 'step' 或 'sine'

% preview
d = 0;  
Channel = 1;                    % 激發通道 (1-6)
Amplitude = 0.5;               % 振幅 [V]
Frequency = 100;                % Sine 頻率 [Hz]
Phase = 0;                      % Sine 相位 [deg]
StepTime = 0;                 % Step 跳變時間 [s]
                             
% Step 模式
step_simulation_time = 0.5;     % Step 模式總模擬時間 [s]

% Sine 模式配置
total_cycles = 100;             % 總模擬週期數（參考 frequency_sweep）
skip_cycles = 60;               % 跳過前 N 個週期（暫態，參考 frequency_sweep）
fft_cycles = 40;                % FFT 分析使用的週期數（參考 frequency_sweep）
sine_display_cycles = 5;        % 顯示最後 N 個週期（圖形顯示用）
sine_min_sim_time = 0.1;        % 最小模擬時間 [s]
sine_max_sim_time = 50.0;       % 最大模擬時間 [s]

% 品質檢測參數（參考 frequency_sweep）
steady_state_threshold = 0.02;  % 穩態檢測閾值 (2% of Amplitude)
thd_threshold = 1.0;            % THD 閾值 (1%)
dc_tolerance = 0.01;            % DC 值容忍度 (1% of Amplitude)
freq_error_threshold = 0.1;     % 頻率誤差警告閾值 (0.1%)

% lambda corresponding bandwidth [Hz]
T = 1e-5;

fB_f = 3000;
fB_c = 500;
fB_e = 500;

lambda_f = exp(-fB_f*T*2*pi);
lambda_c = exp(-fB_c*T*2*pi);
lambda_e = exp(-fB_e*T*2*pi);
beta = sqrt(lambda_e * lambda_c);

% ==================== 計算控制器參數 ====================
params = r_controller_calc_params(fB_c, fB_e, fB_f);
% ======================================================


% ========== 顯示控制設定 ==========
DISPLAY_MODE = 'full';  % 'full' = 顯示所有圖, 'simplified' = 只顯示兩張圖
SAVE_ALL_FIGURES = true;      % 是否儲存所有圖形（即使不顯示）

% ========== 視窗位置設定 ==========
% [left, bottom, width, height] 單位是 pixels
% 可根據您的螢幕調整這些值
FIGURE_POSITIONS = struct();
FIGURE_POSITIONS.Fig1 = [50, 100, 900, 700];           % 圖 1 位置
FIGURE_POSITIONS.Fig2 = [980, 100, 1200, 800];         % 圖 2 位置
FIGURE_POSITIONS.Fig3 = [50, 100, 1000, 600];          % 圖 3 位置
FIGURE_POSITIONS.Fig4 = [980, 100, 1200, 800];         % 圖 4 位置
FIGURE_POSITIONS.Fig5 = [50, 100, 1200, 800];          % 圖 5 位置 (u_w1)
FIGURE_POSITIONS.Fig6 = [50, 100, 800, 500];           % 圖 6 位置 (激發通道 u)
FIGURE_POSITIONS.Fig7 = [900, 100, 800, 500];          % 圖 7 位置 (激發通道 u_w1)

% 如果想要在第二個螢幕顯示（如果有的話），可以用負值的 left
% 例如：[-1800, 100, 900, 700] 會在左邊的第二個螢幕

Ts = 1e-5;                      % 採樣時間 [s] (100 kHz)
solver = 'ode45';             % Simulink solver  ode23tb

model_name = 'r_controller_system_integrated';

script_dir = fileparts(mfilename('fullpath'));
package_root = fullfile(script_dir, '..');
model_path = fullfile(package_root, 'model', [model_name '.slx']);

colors = [
    0.0000, 0.0000, 0.5000;  % P1: 深藍色
    0.0000, 0.0000, 1.0000;  % P2: 藍色
    0.0000, 0.5000, 0.0000;  % P3: 綠色
    1.0000, 0.0000, 0.0000;  % P4: 紅色
    0.8000, 0.0000, 0.8000;  % P5: 粉紫色
    0.0000, 0.7500, 0.7500;  % P6: 青色
];

vm_vd_unified_axis = true;
measurement_linewidth = 3.0;     % Measurement 線粗細
reference_linewidth = 2.5;       % Reference 線粗細

% 圖形格式設定（新增）
axis_linewidth = 1.5;            % 座標軸線粗細
xlabel_fontsize = 14;            % X 軸標籤字體大小
ylabel_fontsize = 14;            % Y 軸標籤字體大小
title_fontsize = 15;             % 標題字體大小
tick_fontsize = 12;              % 刻度字體大小
legend_fontsize = 11;            % 圖例字體大小

% ┌─────────────────────────────────────────────────────────────┐
% │                   輸出控制                                   │
% └─────────────────────────────────────────────────────────────┘
ENABLE_PLOT = true;
SAVE_PNG = true;
SAVE_MAT = true;

% 根據測試類型選擇輸出資料夾
if strcmpi(signal_type_name, 'sine')
    output_dir = fullfile('test_results', 'sine_wave');
else
    output_dir = fullfile('test_results', 'step_response');
end

%% SECTION 2: 初始化與驗證

fprintf('\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('           R Controller 自動化測試\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');

% 轉換 SignalType（字串 → 數字，給 Simulink 使用）
if strcmpi(signal_type_name, 'sine')
    SignalType = 1;
else
    SignalType = 2;
end

% 驗證參數
if ~ismember(lower(signal_type_name), {'step', 'sine'})
    error('signal_type_name 必須是 ''step'' 或 ''sine''');
end

if Channel < 1 || Channel > 6
    error('Channel 必須在 1-6 之間');
end

% 顯示 Workspace 變數配置
fprintf('【Workspace 變數】\n');
fprintf('────────────────────────\n');
fprintf('  SignalType: %d (%s)\n', SignalType, signal_type_name);
fprintf('  Channel: %d\n', Channel);
fprintf('  Amplitude: %.3f V\n', Amplitude);
if strcmpi(signal_type_name, 'sine')
    fprintf('  Frequency: %.1f Hz\n', Frequency);
    fprintf('  Phase: %.1f deg\n', Phase);
else
    fprintf('  StepTime: %.3f s\n', StepTime);
end
fprintf('  d (preview): %d\n', d);
fprintf('  fB_f: %d Hz\n', fB_f);
fprintf('  fB_c: %d Hz\n', fB_c);
fprintf('  fB_e: %d Hz\n', fB_e);
fprintf('  lambda_f: %.6f\n', lambda_f);
fprintf('  lambda_c: %.6f\n', lambda_c);
fprintf('  lambda_e: %.6f\n', lambda_e);
fprintf('  beta: %.6f\n', beta);
fprintf('\n');

% 創建輸出目錄
if SAVE_PNG || SAVE_MAT
    output_dir = fullfile(package_root, output_dir);
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    test_dir = fullfile(output_dir, sprintf('%s_%s', test_name, timestamp));
    mkdir(test_dir);
    fprintf('📁 輸出目錄: %s\n\n', test_dir);
end

%% SECTION 3: 計算模擬時間

fprintf('【模擬時間計算】\n');
fprintf('────────────────────────\n');

if strcmpi(signal_type_name, 'sine')
    % Sine 模式：基於總週期數計算（參考 frequency_sweep）
    period = 1 / Frequency;
    sim_time = total_cycles * period;
    sim_time = max(sine_min_sim_time, min(sine_max_sim_time, sim_time));

    fprintf('  頻率: %.1f Hz\n', Frequency);
    fprintf('  週期: %.6f s\n', period);
    fprintf('  總模擬週期: %d (跳過 %d, 分析 %d)\n', ...
            total_cycles, skip_cycles, fft_cycles);
    fprintf('  實際模擬時間: %.4f s\n', sim_time);
else
    % Step 模式：固定時間
    sim_time = step_simulation_time;

    fprintf('  Step 跳變時間: %.3f s\n', StepTime);
    fprintf('  模擬時間: %.3f s\n', sim_time);
end

fprintf('\n');

%% SECTION 4: 開啟模型並配置模擬器

fprintf('【配置 Simulink 模型】\n');
fprintf('────────────────────────\n');
fprintf('  模型: %s\n', model_name);
fprintf('  模型路徑: %s\n', model_path);

% 檢查模型檔案
if ~exist(model_path, 'file')
    error('找不到模型檔案: %s', model_path);
end

% 開啟模型
if ~bdIsLoaded(model_name)
    open_system(model_path);
end
fprintf('  ✓ 模型已開啟\n');

% 設定模擬器參數
set_param(model_name, 'StopTime', num2str(sim_time));
set_param(model_name, 'Solver', solver);
set_param(model_name, 'MaxStep', num2str(Ts/10));
fprintf('  ✓ 模擬器參數已設定\n');
fprintf('    - StopTime: %.4f s\n', sim_time);
fprintf('    - Solver: %s\n', solver);
fprintf('    - MaxStep: %.2e s\n', Ts/10);

% 將 params 變數設定到模型工作區或基礎工作區
% 確保 Simulink 模型可以存取 params 變數
assignin('base', 'params', params);
fprintf('  ✓ 參數已載入至工作區\n');

fprintf('\n');

%% SECTION 5: 執行模擬

fprintf('【執行 Simulink 模擬】\n');
fprintf('────────────────────────\n');
fprintf('  採樣頻率: %.0f kHz\n', 1/Ts/1000);
fprintf('  ⏳ 模擬執行中...\n');

tic;
try
    out = sim(model_name);
    elapsed_time = toc;
    fprintf('  ✓ 模擬完成 (耗時 %.2f 秒)\n', elapsed_time);
catch ME
    fprintf('  ❌ 模擬失敗\n');
    fprintf('  錯誤訊息: %s\n', ME.message);

    % 顯示更詳細的錯誤資訊
    if ~isempty(ME.cause)
        fprintf('\n  詳細原因:\n');
        for i = 1:length(ME.cause)
            fprintf('  [%d] %s\n', i, ME.cause{i}.message);
        end
    end

    % 顯示錯誤堆疊
    fprintf('\n  錯誤堆疊:\n');
    for i = 1:min(3, length(ME.stack))
        fprintf('  - %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end

    rethrow(ME);
end

fprintf('\n');

%% SECTION 6: 提取數據

fprintf('【數據提取】\n');
fprintf('────────────────────────\n');

try
    Vd_data = out.Vd;
    Vm_data = out.Vm;
    u_data = out.u;
    u_w1_data = out.u_w1;
    disturbance_data = out.disturbance;

    % 根據採樣率生成時間軸（與數據同步）
    N = size(Vd_data, 1);
    t = (0:N-1)' * Ts;

    % 檢查 disturbance 維度，如果只有 1 通道，則複製到 6 個通道
    if size(disturbance_data, 2) == 1
        disturbance_data = repmat(disturbance_data, 1, 6);
        fprintf('  ⚠ disturbance 只有 1 通道，已複製到 6 通道\n');
    end

    fprintf('  ✓ 數據點數: %d (%.3f 秒)\n', N, t(end));
    fprintf('  ✓ Vd: [%d × %d]\n', size(Vd_data, 1), size(Vd_data, 2));
    fprintf('  ✓ Vm: [%d × %d]\n', size(Vm_data, 1), size(Vm_data, 2));
    fprintf('  ✓ u: [%d × %d]\n', size(u_data, 1), size(u_data, 2));
    fprintf('  ✓ u_w1: [%d × %d]\n', size(u_w1_data, 1), size(u_w1_data, 2));
    fprintf('  ✓ disturbance: [%d × %d]\n', size(disturbance_data, 1), size(disturbance_data, 2));
catch ME
    error('數據提取失敗: %s', ME.message);
end

fprintf('\n');

%% SECTION 7: 穩態數據選取與品質檢測（Sine 模式）

if strcmpi(signal_type_name, 'sine')
    fprintf('【穩態數據選取與品質檢測】\n');
    fprintf('────────────────────────\n');

    % ========================================
    % 1. 選取穩態數據段用於 FFT 分析（參考 frequency_sweep Line 209-228）
    % ========================================
    period = 1 / Frequency;
    skip_time = skip_cycles * period;
    fft_time = fft_cycles * period;

    t_start = skip_time;
    t_end = min(skip_time + fft_time, t(end));

    idx_steady = (t >= t_start) & (t <= t_end);

    if sum(idx_steady) < 100
        fprintf('  ✗ 穩態數據點不足 (%d 點)，跳過 FFT 分析\n', sum(idx_steady));
    else
        Vd_steady = Vd_data(idx_steady, :);
        Vm_steady = Vm_data(idx_steady, :);
        t_steady = t(idx_steady);

        actual_cycles = (t_end - t_start) / period;
        fprintf('  ✓ 穩態數據選取: %.2f ~ %.2f s (%.1f 個週期, %d 點)\n', ...
                t_start, t_end, actual_cycles, sum(idx_steady));

        % ========================================
        % 2. 品質檢測：週期重複性（參考 frequency_sweep Line 232-303）
        % ========================================
        fprintf('  🔍 執行品質檢測...\n');

        samples_per_cycle = round(period / Ts);
        num_cycles_to_check = min(fft_cycles, floor(length(t_steady) / samples_per_cycle));

        % 初始化品質檢測結果
        quality_steady_state = true(1, 6);
        quality_thd = zeros(1, 6);
        quality_dc_error = zeros(1, 6);
        quality_thd_pass = true(1, 6);
        quality_dc_pass = true(1, 6);

        fs = 1 / Ts;

        for ch = 1:6
            % --- 週期重複性檢測 ---
            cycle_diffs = [];

            for k = 2:num_cycles_to_check
                idx_start_prev = (k-2) * samples_per_cycle + 1;
                idx_end_prev = (k-1) * samples_per_cycle;
                idx_start_curr = (k-1) * samples_per_cycle + 1;
                idx_end_curr = k * samples_per_cycle;

                if idx_end_curr <= length(Vm_steady(:, ch))
                    cycle_prev = Vm_steady(idx_start_prev:idx_end_prev, ch);
                    cycle_curr = Vm_steady(idx_start_curr:idx_end_curr, ch);

                    max_diff = max(abs(cycle_curr - cycle_prev));
                    cycle_diffs = [cycle_diffs; max_diff];
                end
            end

            threshold = steady_state_threshold * Amplitude;
            if ~isempty(cycle_diffs)
                quality_steady_state(ch) = all(cycle_diffs < threshold);
            else
                quality_steady_state(ch) = false;
            end

            % --- THD 檢測 ---
            try
                thd_dB = thd(Vm_steady(:, ch), fs, 10);
                thd_percent = 10^(thd_dB/20) * 100;
                quality_thd(ch) = thd_percent;
                quality_thd_pass(ch) = (thd_percent < thd_threshold);
            catch
                quality_thd(ch) = NaN;
                quality_thd_pass(ch) = false;
            end

            % --- DC 偏移檢測 ---
            Vm_fft_temp = fft(Vm_steady(:, ch));
            N_fft_temp = length(Vm_fft_temp);
            DC_value = abs(Vm_fft_temp(1)) / N_fft_temp;
            quality_dc_error(ch) = abs(DC_value);
            quality_dc_pass(ch) = (quality_dc_error(ch) < dc_tolerance * Amplitude);
        end

        % ========================================
        % 3. 顯示品質檢測結果（參考 frequency_sweep Line 333-383）
        % ========================================
        fprintf('  ✓ 品質檢測完成\n');
        fprintf('    通道 | 穩態 | THD          | DC誤差  | 狀態\n');
        fprintf('    ─────┼──────┼──────────────┼─────────┼──────\n');

        for ch = 1:6
            steady_mark = '✓';
            if ~quality_steady_state(ch)
                steady_mark = '✗';
            end

            thd_mark = '✓';
            if ~quality_thd_pass(ch)
                thd_mark = '✗';
            end

            dc_mark = '✓';
            if ~quality_dc_pass(ch)
                dc_mark = '✗';
            end

            % 整體狀態判斷
            if ch == Channel
                % 激勵通道：必須全部通過
                if quality_steady_state(ch) && quality_thd_pass(ch) && quality_dc_pass(ch)
                    status = 'PASS';
                else
                    status = 'WARN';
                end
            else
                % 其他通道：標記但不影響 FFT
                if quality_steady_state(ch) && quality_thd_pass(ch) && quality_dc_pass(ch)
                    status = 'OK';
                else
                    status = 'FAIL';
                end
            end

            % THD 動態格式顯示
            thd_val = quality_thd(ch);
            if isnan(thd_val)
                thd_str = '    N/A    ';
            elseif thd_val < 0.01
                thd_str = sprintf('%10.2e%%', thd_val);
            else
                thd_str = sprintf('%10.4f%%', thd_val);
            end

            fprintf('     P%d  |  %s   | %s %s | %.4fV %s | %s\n', ...
                    ch, steady_mark, thd_str, thd_mark, ...
                    quality_dc_error(ch), dc_mark, status);
        end

        fprintf('\n');
    end

    % ========================================
    % 4. 選取圖形顯示數據（最後 N 個週期，保持原邏輯）
    % ========================================
    fprintf('  📊 準備圖形顯示數據...\n');

    t_display_start = t(end) - sine_display_cycles * period;
    t_display_start = max(0, t_display_start);

    idx_display = t >= t_display_start;
    t_display = t(idx_display);
    Vd_display = Vd_data(idx_display, :);
    Vm_display = Vm_data(idx_display, :);

    fprintf('    顯示範圍: %.4f - %.4f s\n', t_display(1), t_display(end));
    fprintf('    顯示週期數: %.1f\n', (t_display(end) - t_display(1)) / period);
    fprintf('    顯示數據點: %d\n', length(t_display));
    fprintf('\n');

    %% SECTION 7.5: FFT 頻率響應分析（使用穩態數據）

    if sum(idx_steady) >= 100
        fprintf('【FFT 頻率響應分析】\n');
        fprintf('────────────────────────\n');

        % ========================================
        % 使用穩態數據段進行 FFT（參考 frequency_sweep Line 389-438）
        % ========================================

        % 對激勵通道的 Vd 做 FFT
        Vd_fft = fft(Vd_steady(:, Channel));
        N_fft = length(Vd_fft);

        % 計算頻率軸
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

        % 提取激勵頻率的幅度與相位
        Vd_mag = abs(Vd_fft(freq_bin_idx)) * 2 / N_fft;
        Vd_phase = angle(Vd_fft(freq_bin_idx)) * 180 / pi;

        % 對每個 Vm 通道做 FFT
        magnitude_ratio = zeros(1, 6);
        phase_lag = zeros(1, 6);

        for ch = 1:6
            Vm_fft = fft(Vm_steady(:, ch));
            Vm_mag = abs(Vm_fft(freq_bin_idx)) * 2 / N_fft;
            Vm_phase = angle(Vm_fft(freq_bin_idx)) * 180 / pi;

            % 計算頻率響應
            magnitude_ratio(ch) = Vm_mag / Vd_mag;
            phase_lag(ch) = Vm_phase - Vd_phase;

            % 相位正規化到 [-180, 180]
            while phase_lag(ch) > 180
                phase_lag(ch) = phase_lag(ch) - 360;
            end
            while phase_lag(ch) < -180
                phase_lag(ch) = phase_lag(ch) + 360;
            end
        end

        % 找出除了激勵通道外，振幅比最大的通道
        other_channels = setdiff(1:6, Channel);
        [max_gain, max_idx] = max(magnitude_ratio(other_channels));
        max_gain_channel = other_channels(max_idx);

        % 顯示結果
        fprintf('  激勵頻率: %.1f Hz\n', Frequency);
        fprintf('  激勵通道: P%d\n', Channel);
        fprintf('  FFT 數據點數: %d (%.2f 個週期)\n', N_fft, actual_cycles);
        fprintf('\n');
        fprintf('  通道  |   振幅比    |   相位差    | 品質\n');
        fprintf('  ──────┼─────────────┼─────────────┼──────\n');
        for ch = 1:6
            marker = '';
            if ch == Channel
                marker = '← 激勵';
            elseif ch == max_gain_channel
                marker = '← 最大響應';
            end

            % 品質標記
            quality_mark = '';
            if ~quality_steady_state(ch)
                quality_mark = ' ⚠️未穩態';
            elseif ~quality_thd_pass(ch)
                quality_mark = ' ⚠️高THD';
            elseif ~quality_dc_pass(ch)
                quality_mark = ' ⚠️DC偏移';
            end

            fprintf('   P%d   |  %6.2f%%   |  %+7.2f°  | %s%s\n', ...
                    ch, magnitude_ratio(ch)*100, phase_lag(ch), marker, quality_mark);
        end
        fprintf('\n');
    end
end

%% SECTION 7.5: 性能指標計算（Step 模式）

if strcmpi(signal_type_name, 'step')
    fprintf('【性能指標計算】\n');
    fprintf('────────────────────────\n');
    fprintf('  激發通道: P%d\n', Channel);
    fprintf('  目標振幅: %.4f V\n\n', Amplitude);

    % === 提取激發通道數據 ===
    Vm_ch = Vm_data(:, Channel);
    Vd_ch = Vd_data(:, Channel);

    % === 1. 計算穩態值 ===
    % 使用最後 100ms 的平均值作為穩態值
    settling_window = 0.1;  % 100 ms
    n_samples_window = round(settling_window / Ts);
    final_value = mean(Vm_ch(end-n_samples_window:end));

    % === 2. 穩態誤差 (Steady-State Error) ===
    % 使用最後 100ms 的平均絕對誤差（手動計算 e = Vd - Vm）
    e_steady = Vd_ch(end-n_samples_window:end) - Vm_ch(end-n_samples_window:end);
    sse = mean(abs(e_steady));
    sse_percent = (sse / abs(Amplitude)) * 100;

    % === 3. 上升時間 (Rise Time, 10% to 90%) ===
    level_10 = final_value * 0.1;
    level_90 = final_value * 0.9;

    idx_10 = find(Vm_ch >= level_10, 1, 'first');
    idx_90 = find(Vm_ch >= level_90, 1, 'first');

    if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
        rise_time = t(idx_90) - t(idx_10);
    else
        rise_time = NaN;
    end

    % === 4. 安定時間 (Settling Time, 2% band) ===
    settling_band = 0.02;  % ±2%
    upper_bound = final_value * (1 + settling_band);
    lower_bound = final_value * (1 - settling_band);

    outside_band = (Vm_ch > upper_bound) | (Vm_ch < lower_bound);
    last_violation_idx = find(outside_band, 1, 'last');

    if isempty(last_violation_idx)
        settling_time = 0;  % 一直在範圍內
    else
        settling_time = t(last_violation_idx);
    end

    % === 5. 最大超越量 (Maximum Overshoot) ===
    % 只看 StepTime 之後的數據
    idx_after_step = t >= StepTime;
    t_after = t(idx_after_step);
    Vm_after = Vm_ch(idx_after_step);

    [peak_value, peak_idx_rel] = max(Vm_after);
    peak_idx = find(idx_after_step, 1, 'first') + peak_idx_rel - 1;
    peak_time = t(peak_idx);

    if final_value ~= 0
        overshoot_percent = ((peak_value - final_value) / abs(final_value)) * 100;
    else
        overshoot_percent = 0;
    end

    % 如果沒有超越（peak < final），設為 0
    if overshoot_percent < 0
        overshoot_percent = 0;
    end

    % === 顯示結果 ===
    fprintf('  時域響應特性:\n');
    fprintf('    ├─ 穩態值:                  %.6f V\n', final_value);
    fprintf('    ├─ 安定時間 (2%% band):      %.4f s (%.2f ms)\n', ...
            settling_time, settling_time*1000);
    fprintf('    ├─ 最大超越量:              %.2f %% (峰值: %.6f V)\n', ...
            overshoot_percent, peak_value);
    fprintf('    └─ 穩態誤差 (SSE):          %.6f V (%.4f %%)\n', ...
            sse, sse_percent);

    % === 保存到結構 ===
    performance.channel = Channel;
    performance.target_value = Amplitude;
    performance.final_value = final_value;
    performance.rise_time = rise_time;
    performance.peak_time = peak_time;
    performance.peak_value = peak_value;
    performance.settling_time_2pct = settling_time;
    performance.settling_band = settling_band;
    performance.overshoot_percent = overshoot_percent;
    performance.sse = sse;
    performance.sse_percent = sse_percent;

    fprintf('\n');
end

%% SECTION 8: 繪圖

if ENABLE_PLOT
    fprintf('【生成圖表】\n');
    fprintf('────────────────────────\n');

    if strcmpi(signal_type_name, 'sine')
        % === 圖 1: Vm_Vd ===
        fig1 = figure('Name', 'Vm_Vd', 'Position', FIGURE_POSITIONS.Fig1);

        hold on;
        grid off;  % 取消背景網格線

        % 繪製所有通道 (使用激發通道的 Vd)
        % 策略：非激發通道由粗到細（第1條=激發通道粗度），最後畫激發通道

        % 1. 激發通道的線寬
        excited_linewidth = measurement_linewidth * 1.5;

        % 2. 非激發通道線寬範圍
        % 最粗 = 激發通道線寬
        % 最細 = measurement_linewidth * 0.5 (加粗)
        base_thick = excited_linewidth;
        base_thin = measurement_linewidth * 0.5;

        % 3. 先畫非激發通道（由粗到細，保持通道編號順序）
        % 儲存 plot handle 以便正確對應圖例
        plot_handles = gobjects(6, 1);  % 預分配 handle 陣列

        draw_count = 0;
        for ch = 1:6
            if ch ~= Channel
                draw_count = draw_count + 1;
                % 線寬線性遞減（共5條，索引 1~5）
                lw = base_thick - (base_thick - base_thin) * (draw_count - 1) / 4;
                plot_handles(ch) = plot(Vd_display(:, Channel), Vm_display(:, ch), ...
                     'Color', colors(ch, :), 'LineWidth', lw);
            end
        end

        % 4. 最後畫激發通道（最粗的線）
        plot_handles(Channel) = plot(Vd_display(:, Channel), Vm_display(:, Channel), ...
             'Color', colors(Channel, :), 'LineWidth', excited_linewidth);

        xlabel('Vd (V)', 'FontSize', xlabel_fontsize+6, 'FontWeight', 'bold');  % 更大
        ylabel('Vm (V)', 'FontSize', ylabel_fontsize+6, 'FontWeight', 'bold');  % 更大
        % 取消標題
        % title(sprintf('Vm vs Vd[P%d]', Channel), 'FontSize', title_fontsize, 'FontWeight', 'bold');

        % 設定座標軸格式
        ax = gca;
        ax.LineWidth = 3.0;  % 座標軸線加粗（更粗）
        ax.FontSize = tick_fontsize+6;  % 刻度字體放大（更大）
        ax.FontWeight = 'bold';  % 刻度數字加粗
        ax.Box = 'on';  % 保留框線

        % 設定軸範圍和刻度（基於激發通道振幅）
        if vm_vd_unified_axis
            % 統一軸：使用所有數據的最大值
            max_val = max([max(abs(Vd_display(:))), max(abs(Vm_display(:)))]);
            axis_lim = [-max_val*1.1, max_val*1.1];
            xlim(axis_lim);
            ylim(axis_lim);
            axis square;

            % 刻度基於激發振幅的倍數
            % 生成以 Amplitude 為基準的刻度點
            tick_step = Amplitude / 2;  % 每個刻度間距 = 振幅的一半
            num_ticks = floor(max_val / tick_step);
            tick_values = (-num_ticks:num_ticks) * tick_step;
            % 限制刻度在軸範圍內
            tick_values = tick_values(tick_values >= axis_lim(1) & tick_values <= axis_lim(2));
            xticks(tick_values);
            yticks(tick_values);
        else
            % 非統一軸：貼近激發通道極值
            vd_excited = Vd_display(:, Channel);
            vm_excited = Vm_display(:, Channel);

            vd_range = [min(vd_excited), max(vd_excited)];
            vm_range = [min(vm_excited), max(vm_excited)];
            vd_margin = (vd_range(2) - vd_range(1)) * 0.1;
            vm_margin = (vm_range(2) - vm_range(1)) * 0.1;

            x_lim = [vd_range(1) - vd_margin, vd_range(2) + vd_margin];
            y_lim = [vm_range(1) - vm_margin, vm_range(2) + vm_margin];
            xlim(x_lim);
            ylim(y_lim);

            % 刻度基於激發振幅
            tick_step = Amplitude / 2;
            x_ticks = floor(x_lim(1)/tick_step):ceil(x_lim(2)/tick_step);
            y_ticks = floor(y_lim(1)/tick_step):ceil(y_lim(2)/tick_step);
            xticks(x_ticks * tick_step);
            yticks(y_ticks * tick_step);
        end

        % 添加圖例（使用 plot_handles 確保正確對應，位置改為右下）
        leg = legend(plot_handles, {'P1', 'P2', 'P3', 'P4', 'P5', 'P6'}, ...
               'Location', 'southeast', 'FontSize', legend_fontsize+2, 'FontWeight', 'bold');
        % 設定圖例外框
        leg.BoxFace.ColorType = 'truecoloralpha';
        leg.BoxFace.ColorData = uint8(255*[1; 1; 1; 0.9]);  % 白底
        leg.EdgeColor = [0 0 0];  % 黑色外框
        leg.LineWidth = 1.0;  % 外框線寬

        % 在左上角添加 FFT 頻率響應資訊
        if exist('magnitude_ratio', 'var') && exist('phase_lag', 'var')
            annotation_str = sprintf('Excited P%d Magnitude: %.2f%% Phase: %+.2f°', ...
                                     Channel, magnitude_ratio(Channel)*100, phase_lag(Channel));
        else
            annotation_str = sprintf('Excited P%d Magnitude: N/A Phase: N/A', Channel);
        end

        % 使用 text 在左上角添加標註（數據座標系統）
        x_range = xlim;
        y_range = ylim;
        x_pos = x_range(1) + 0.05 * (x_range(2) - x_range(1));  % 左邊 5%
        y_pos = y_range(2) - 0.08 * (y_range(2) - y_range(1));  % 上邊 8%

        text(x_pos, y_pos, annotation_str, ...
             'FontSize', 11, ...
             'FontName', 'Consolas', ...
             'FontWeight', 'bold', ...
             'BackgroundColor', [1 1 1 0.85], ...
             'EdgeColor', [0.3 0.3 0.3], ...
             'LineWidth', 1.2, ...
             'Margin', 5, ...
             'VerticalAlignment', 'top', ...
             'HorizontalAlignment', 'left');

        fprintf('  ✓ Figure 1: Vm_Vd (with FFT analysis)\n');

        % === 圖 2: 6 通道時域響應 ===
        fig2 = figure('Name', '6 Channels Time Response', 'Position', FIGURE_POSITIONS.Fig2);

        for ch = 1:6
            subplot(2, 3, ch);

            % Measurement (實線)
            plot(t_display*1000, Vm_display(:, ch), '-', ...
                 'Color', colors(ch, :), 'LineWidth', measurement_linewidth);
            hold on;

            % Reference (虛線)
            plot(t_display*1000, Vd_display(:, ch), '--', ...
                 'Color', [0.5, 0.5, 0.5], 'LineWidth', reference_linewidth);

            grid off;  % 去除網格線
            xlabel('Time (ms)', 'FontSize', xlabel_fontsize+2, 'FontWeight', 'bold');  % 更大
            ylabel('HsVm (V)', 'FontSize', ylabel_fontsize+2, 'FontWeight', 'bold');  % 更大
            title(sprintf('P%d', ch), 'FontSize', title_fontsize+2, 'FontWeight', 'bold');

            % 設定座標軸格式
            ax = gca;
            ax.LineWidth = axis_linewidth;
            ax.FontSize = tick_fontsize+2;  % 更大
            ax.FontWeight = 'bold';
            ax.Box = 'off';  % 去除背景框線

            % 添加圖例（只在第一個子圖）
            if ch == 1
                legend({'Measurement', 'Reference'}, ...
                       'Location', 'northeast', 'FontSize', legend_fontsize-2, 'FontWeight', 'bold');
            end
        end

        % 加入總標題
        sgtitle(sprintf('6 Channels Time Response - Excited Ch: P%d, Freq: %.1f Hz (Last %d cycles)', ...
                        Channel, Frequency, sine_display_cycles), ...
                'FontSize', title_fontsize, 'FontWeight', 'bold');

        fprintf('  ✓ Figure 2: 6 Channels Time Response\n');

        % === 計算最後 10 個週期的時間窗口 ===
        period = 1 / Frequency;
        detail_cycles = 10;  % 顯示最後 10 個週期

        % 取最後 10 個週期（穩態）
        t_start_detail = t(end) - detail_cycles * period;
        t_end_detail = t(end);

        % 確保時間範圍有效
        t_start_detail = max(0, t_start_detail);
        t_end_detail = min(t(end), t_end_detail);

        % 選取數據
        idx_detail = (t >= t_start_detail) & (t <= t_end_detail);
        t_detail = t(idx_detail);
        u_detail = u_data(idx_detail, :);

        % 顯示資訊
        actual_cycles = (t_end_detail - t_start_detail) / period;
        fprintf('  📊 詳細分析窗口: %.4f - %.4f s (%.1f 個週期, %d 點)\n', ...
                t_start_detail, t_end_detail, actual_cycles, sum(idx_detail));

        % === 圖 4: 控制輸入 u (Control Effort) ===
        fig4 = figure('Name', sprintf('Control Effort (Last %d cycles)', detail_cycles), ...
                      'Position', FIGURE_POSITIONS.Fig4);

        for ch = 1:6
            subplot(2, 3, ch);

            plot(t_detail*1000, u_detail(:, ch), '-', ...
                 'Color', colors(ch, :), 'LineWidth', measurement_linewidth);

            grid on;
            xlabel('Time (ms)', 'FontSize', xlabel_fontsize-2, 'FontWeight', 'bold');
            ylabel('Control Input u (V)', 'FontSize', ylabel_fontsize-2, 'FontWeight', 'bold');
            title(sprintf('P%d', ch), 'FontSize', title_fontsize-2, 'FontWeight', 'bold');

            % 設定座標軸格式
            ax = gca;
            ax.LineWidth = axis_linewidth;
            ax.FontSize = tick_fontsize-1;
            ax.FontWeight = 'bold';
        end

        % 加入總標題顯示控制參數
        sgtitle(sprintf('Control Effort - fB_f=%.0fHz, fB_c=%.0fHz, fB_e=%.0fHz', ...
                        fB_f, fB_c, fB_e), ...
                'FontSize', title_fontsize, 'FontWeight', 'bold');

        fprintf('  ✓ Figure 4: Control Input u (Last %d cycles)\n', detail_cycles);

        % === 圖 5: u_w1 ===
        u_w1_detail = u_w1_data(idx_detail, :);

        % 正規化時間軸（從 0 開始，單位：秒）
        t_detail_norm = t_detail - t_detail(1);

        fig5 = figure('Name', sprintf('u_w1 (Last %d cycles)', detail_cycles), ...
                      'Position', FIGURE_POSITIONS.Fig5);

        for ch = 1:6
            subplot(2, 3, ch);

            % 繪製 u_w1
            plot(t_detail_norm, u_w1_detail(:, ch), '-', ...
                 'Color', colors(ch, :), 'LineWidth', measurement_linewidth);

            grid on;
            xlabel('Time (s)', 'FontSize', xlabel_fontsize-2, 'FontWeight', 'bold');
            ylabel('(V)', 'FontSize', ylabel_fontsize-2, 'FontWeight', 'bold');
            title(sprintf('P%d', ch), ...
                  'FontSize', title_fontsize-2, 'FontWeight', 'bold');

            % 設定座標軸格式
            ax = gca;
            ax.LineWidth = axis_linewidth;
            ax.FontSize = tick_fontsize-1;
            ax.FontWeight = 'bold';

            % Y 軸緊貼波形範圍
            y_data = u_w1_detail(:, ch);
            ylim([min(y_data)*1.05, max(y_data)*1.05]);

            % X 軸刻度只顯示頭、尾、中間
            xticks(ax, [t_detail_norm(1), t_detail_norm(round(end/2)), t_detail_norm(end)]);
        end

        % 加入總標題
        sgtitle(sprintf('u_w1 - fB_f=%.0fHz, fB_c=%.0fHz, fB_e=%.0fHz', ...
                        fB_f, fB_c, fB_e), ...
                'FontSize', title_fontsize, 'FontWeight', 'bold');

        fprintf('  ✓ Figure 5: u_w1 (Last %d cycles)\n', detail_cycles);

    else
        % === Step 模式繪圖 ===

        % 使用完整時間段的數據
        t_step_full = t;
        Vm_step_full = Vm_data;
        Vd_step_full = Vd_data;
        u_step_full = u_data;

        % 選取 0~10ms 的數據用於 Vm 和 error 圖
        zoom_time = 0.01;  % 10 ms
        idx_zoom = t <= zoom_time;
        t_zoom = t(idx_zoom);
        Vm_zoom = Vm_data(idx_zoom, :);
        Vd_zoom = Vd_data(idx_zoom, :);
        % 手動計算誤差 e = Vd - Vm
        e_zoom = Vd_zoom - Vm_zoom;

        % 圖 1: 6 通道響應 (0~10ms)
        fig1 = figure('Name', 'Step Response - 6 Channels (0-10ms)', ...
                      'Position', FIGURE_POSITIONS.Fig1);

        for ch = 1:6
            subplot(2, 3, ch);

            % Measurement (實線)
            plot(t_zoom*1000, Vm_zoom(:, ch), '-', 'Color', colors(ch, :), ...
                 'LineWidth', measurement_linewidth);
            hold on;

            % Reference (虛線)
            plot(t_zoom*1000, Vd_zoom(:, ch), '--', 'Color', [0, 0, 0], ...
                 'LineWidth', reference_linewidth);

            grid off;  % 去除網格線
            xlabel('Time (ms)', 'FontSize', xlabel_fontsize+2, 'FontWeight', 'bold');  % 更大
            ylabel('HsVm (V)', 'FontSize', ylabel_fontsize+2, 'FontWeight', 'bold');  % 更大
            title(sprintf('P%d', ch), 'FontSize', title_fontsize+2, 'FontWeight', 'bold');

            % 設定座標軸格式
            ax = gca;
            ax.LineWidth = axis_linewidth;
            ax.FontSize = tick_fontsize+2;  % 更大
            ax.FontWeight = 'bold';
            ax.Box = 'off';  % 去除背景框線

            % 添加圖例（只在第一個子圖）
            if ch == 1
                legend({'Measurement', 'Reference'}, ...
                       'Location', 'best', 'FontSize', legend_fontsize-2, 'FontWeight', 'bold');
            end
        end

        % 加入總標題
        sgtitle(sprintf('Step Response - Excited Ch: P%d, Amplitude: %.2f V (0-10ms)', ...
                        Channel, Amplitude), ...
                'FontSize', title_fontsize, 'FontWeight', 'bold');

        fprintf('  ✓ Figure 1: Step Response (0-10ms)\n');

        % === 圖 2: 全時間範圍 Control Effort u ===
        fig2 = figure('Name', 'Control Effort - Full Response', ...
                      'Position', FIGURE_POSITIONS.Fig2);

        for ch = 1:6
            subplot(2, 3, ch);

            plot(t*1000, u_data(:, ch), '-', 'Color', colors(ch, :), ...
                 'LineWidth', measurement_linewidth);

            grid on;
            xlabel('Time (ms)', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
            ylabel('u (V)', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
            title(sprintf('P%d', ch), 'FontSize', title_fontsize, 'FontWeight', 'bold');

            % 設定座標軸格式
            ax = gca;
            ax.LineWidth = axis_linewidth;
            ax.FontSize = tick_fontsize;
            ax.FontWeight = 'bold';

            % 顯示統計資訊
            u_max = max(u_data(:, ch));
            u_min = min(u_data(:, ch));
            u_ss = mean(u_data(end-min(1000, size(u_data,1)-1):end, ch));

            text(0.95, 0.95, sprintf('Max: %.2f\nMin: %.2f\nSS: %.2f', u_max, u_min, u_ss), ...
                 'Units', 'normalized', ...
                 'HorizontalAlignment', 'right', ...
                 'VerticalAlignment', 'top', ...
                 'FontSize', legend_fontsize-2, ...
                 'BackgroundColor', [1 1 1 0.8], ...
                 'EdgeColor', [0.5 0.5 0.5]);
        end

        % 加入總標題
        sgtitle(sprintf('Control Effort (Full) - P%d, Amp: %.2fV, fB_c=%dHz, fB_e=%dHz', ...
                        Channel, Amplitude, fB_c, fB_e), ...
                'FontSize', title_fontsize, 'FontWeight', 'bold');

        fprintf('  ✓ Figure 2: Control Effort - Full Response\n');

        % === 提取穩態數據（最後 100ms）===
        steady_time = 0.1;  % 100 ms
        idx_steady = t >= (t(end) - steady_time);
        t_steady = t(idx_steady);
        u_steady = u_data(idx_steady, :);
        u_w1_steady = u_w1_data(idx_steady, :);

        % 正規化時間軸（從 0 開始）
        t_steady_norm = t_steady - t_steady(1);

        % === 圖 4: 控制輸入 u (Control Effort) ===
        fig4 = figure('Name', 'Control Effort (Steady State)', ...
                      'Position', FIGURE_POSITIONS.Fig4);

        for ch = 1:6
            subplot(2, 3, ch);

            plot(t_steady_norm*1000, u_steady(:, ch), '-', ...
                 'Color', colors(ch, :), 'LineWidth', measurement_linewidth);

            grid on;
            xlabel('Time (ms)', 'FontSize', xlabel_fontsize-2, 'FontWeight', 'bold');
            ylabel('Control Input u (V)', 'FontSize', ylabel_fontsize-2, 'FontWeight', 'bold');
            title(sprintf('P%d', ch), 'FontSize', title_fontsize-2, 'FontWeight', 'bold');

            % 設定座標軸格式
            ax = gca;
            ax.LineWidth = axis_linewidth;
            ax.FontSize = tick_fontsize-1;
            ax.FontWeight = 'bold';
        end

        % 加入總標題顯示控制參數
        sgtitle(sprintf('Control Effort - fB_f=%.0fHz, fB_c=%.0fHz, fB_e=%.0fHz', ...
                        fB_f, fB_c, fB_e), ...
                'FontSize', title_fontsize, 'FontWeight', 'bold');

        fprintf('  ✓ Figure 4: Control Input u (Steady State)\n');

        % === 圖 5: u_w1 ===
        fig5 = figure('Name', 'u_w1 (Steady State)', ...
                      'Position', FIGURE_POSITIONS.Fig5);

        for ch = 1:6
            subplot(2, 3, ch);

            % 繪製 u_w1
            plot(t_steady_norm, u_w1_steady(:, ch), '-', ...
                 'Color', colors(ch, :), 'LineWidth', measurement_linewidth);

            grid on;
            xlabel('Time (s)', 'FontSize', xlabel_fontsize-2, 'FontWeight', 'bold');
            ylabel('(V)', 'FontSize', ylabel_fontsize-2, 'FontWeight', 'bold');
            title(sprintf('P%d', ch), ...
                  'FontSize', title_fontsize-2, 'FontWeight', 'bold');

            % 設定座標軸格式
            ax = gca;
            ax.LineWidth = axis_linewidth;
            ax.FontSize = tick_fontsize-1;
            ax.FontWeight = 'bold';

            % Y 軸緊貼波形範圍
            y_data = u_w1_steady(:, ch);
            ylim([min(y_data)*1.05, max(y_data)*1.05]);

            % X 軸刻度只顯示頭、尾、中間
            xticks(ax, [t_steady_norm(1), t_steady_norm(round(end/2)), t_steady_norm(end)]);
        end

        % 加入總標題
        sgtitle(sprintf('u_w1 - fB_f=%.0fHz, fB_c=%.0fHz, fB_e=%.0fHz', ...
                        fB_f, fB_c, fB_e), ...
                'FontSize', title_fontsize, 'FontWeight', 'bold');

        fprintf('  ✓ Figure 5: u_w1 (Steady State)\n');
    end

    fprintf('\n');
end

%% SECTION 9: 保存結果

if SAVE_PNG || SAVE_MAT
    fprintf('【保存結果】\n');
    fprintf('────────────────────────\n');

    % 保存圖片
    if SAVE_PNG && ENABLE_PLOT
        if strcmpi(signal_type_name, 'sine')
            saveas(fig1, fullfile(test_dir, 'Vm_Vd.png'));
            saveas(fig2, fullfile(test_dir, '6ch_time_response.png'));
            saveas(fig4, fullfile(test_dir, 'control_input_u.png'));
            saveas(fig5, fullfile(test_dir, 'u_w1_estimated.png'));
        else
            saveas(fig1, fullfile(test_dir, 'step_response_6ch.png'));
            saveas(fig2, fullfile(test_dir, 'control_effort_full.png'));
            saveas(fig4, fullfile(test_dir, 'control_input_u_steady.png'));
            saveas(fig5, fullfile(test_dir, 'u_w1.png'));
        end
        fprintf('  ✓ Figures saved (.png)\n');
    end

    % 保存 MAT 數據
    if SAVE_MAT
        result = struct();
        result.config.test_name = test_name;
        result.config.signal_type_name = signal_type_name;
        result.config.SignalType = SignalType;
        result.config.Channel = Channel;
        result.config.Amplitude = Amplitude;
        result.config.d = d;
        result.config.fB_f = fB_f;
        result.config.fB_c = fB_c;
        result.config.fB_e = fB_e;
        result.config.lambda_f = lambda_f;
        result.config.lambda_c = lambda_c;
        result.config.lambda_e = lambda_e;
        result.config.beta = beta;
        result.config.sim_time = sim_time;
        result.config.Ts = Ts;

        if strcmpi(signal_type_name, 'sine')
            result.config.Frequency = Frequency;
            result.config.Phase = Phase;
            result.config.sine_display_cycles = sine_display_cycles;
        else
            result.config.StepTime = StepTime;
        end

        result.data.t = t;
        result.data.Vd = Vd_data;
        result.data.Vm = Vm_data;
        result.data.u = u_data;
        result.data.u_w1 = u_w1_data;

        if strcmpi(signal_type_name, 'sine')
            result.display.t = t_display;
            result.display.Vd = Vd_display;
            result.display.Vm = Vm_display;

            % FFT 分析結果
            result.analysis.magnitude_ratio = magnitude_ratio;
            result.analysis.phase_lag = phase_lag;
            result.analysis.excited_freq = Frequency;
            result.analysis.actual_freq = actual_freq;
            result.analysis.freq_error_percent = freq_error_percent;

            % 品質檢測結果（新增，參考 frequency_sweep）
            result.quality.steady_state = quality_steady_state;
            result.quality.thd = quality_thd;
            result.quality.dc_error = quality_dc_error;
            result.quality.thd_pass = quality_thd_pass;
            result.quality.dc_pass = quality_dc_pass;
            result.quality.steady_state_threshold = steady_state_threshold;
            result.quality.thd_threshold = thd_threshold;
            result.quality.dc_tolerance = dc_tolerance;

            % FFT 數據窗口資訊（新增）
            result.fft_window.t_start = t_start;
            result.fft_window.t_end = t_end;
            result.fft_window.cycles = actual_cycles;
            result.fft_window.data_points = N_fft;
        else
            % Step 模式：保存性能指標
            result.performance = performance;
        end

        result.meta.timestamp = datestr(now);
        result.meta.elapsed_time = elapsed_time;

        save(fullfile(test_dir, 'result.mat'), 'result', '-v7.3');
        fprintf('  ✓ 數據已保存 (.mat)\n');
    end

    fprintf('  📁 所有檔案保存至: %s\n\n', test_dir);
end

%% SECTION 10: 測試總結

fprintf('════════════════════════════════════════════════════════════\n');
fprintf('                     測試完成\n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('\n');

fprintf('【測試摘要】\n');
fprintf('  名稱: %s\n', test_name);
fprintf('  信號: %s, P%d, %.3f V\n', signal_type_name, Channel, Amplitude);
if strcmpi(signal_type_name, 'sine')
    fprintf('  頻率: %.1f Hz\n', Frequency);
end
fprintf('  R Controller 參數: d=%d, fB_c=%d Hz, fB_e=%d Hz\n', d, fB_c, fB_e);
fprintf('  執行時間: %.2f 秒\n', elapsed_time);

fprintf('\n');
