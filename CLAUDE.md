# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

R-Controller Package: A MATLAB/Simulink implementation of an R-Controller for magnetic tweezers, controlling 6-pole electromagnetic actuators for precision biological force measurement at piconewton scale.

## Directory Structure

```
r_controller_package/
├── model/
│   ├── inner_loop_ctrl/              # Inner loop controller components
│   │   ├── model_base_ctrl_calc_params.m  # Controller parameter calculator
│   │   └── model_base_ctrl_function.m     # Core difference equations
│   ├── flux_allocation/             # Force control components
│   │   ├── force_model.m             # Hall voltage → estimated force
│   │   ├── inverse_model.m           # Force → Hall voltage (LUT-based)
│   │   └── system_params.m           # System constants and matrices
│   └── main_system.slx  # Main Simulink model
├── test_script/
│   ├── inner_loop/                   # Inner loop controller tests
│   │   ├── run_inner_loop_test.m     # Single-frequency test (sine/step)
│   │   └── run_inner_loop_bode.m     # Frequency sweep (Bode plot)
│   ├── force_generation/                   # Force generation pipeline tests
│   │   ├── run_force_generation_test.m    # Force generation integration test
│   │   └── run_force_bode.m               # Force generation frequency sweep
│   └── utils/                        # Shared test utilities
│       ├── plot_styles.m             # Unified plot style settings
│       ├── fft_analysis.m            # FFT frequency response analysis
│       ├── quality_check.m           # Signal quality detection
│       └── test_config.m             # Unified test configuration
├── data/lut/                         # Lookup tables for inverse model
└── test_results/                     # Test output directory (gitignored)
```

## Development Commands

```matlab
% Add paths and load parameters
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, 'model'));
addpath(fullfile(script_dir, 'model', 'inner_loop_ctrl'));
addpath(fullfile(script_dir, 'model', 'flux_allocation'));

% Initialize controller parameters (fB_c, fB_e, fB_f in Hz)
ctrl_params = model_base_ctrl_calc_params(500, 500, 3000);

% Run inner loop tests
run('test_script/inner_loop/run_inner_loop_test.m')
run('test_script/inner_loop/run_inner_loop_bode.m')

% Run force generation tests
run('test_script/force_generation/run_force_generation_test.m')
run('test_script/force_generation/run_force_bode.m')
```

## Architecture

### Three-Phase Design

1. **Inner Loop Controller** (Phase 1, Complete): Discrete-time feedback control with feedforward filter, disturbance observer, and PI control law
2. **Physical Models** (Phase 2): `inverse_model.m` (force→voltage) and `force_model.m` (voltage→force)
3. **Full Integration** (Phase 3): Closed-loop force control pipeline

### Signal Flow

```
Desired Force (f_d) → Inverse Model → v_d → R-Controller → u (currents) → Plant → v_m → Force Model → f_m
```

### Key Files

| File | Description |
|------|-------------|
| `model/inner_loop_ctrl/model_base_ctrl_calc_params.m` | Computes controller coefficients from bandwidth parameters, creates ParamsBus |
| `model/inner_loop_ctrl/model_base_ctrl_function.m` | Core discrete-time difference equations (runs at 100 kHz) |
| `model/flux_allocation/inverse_model.m` | Force to Hall voltage conversion using LUT interpolation |
| `model/flux_allocation/force_model.m` | Hall voltage to estimated force using L-matrix calculations |
| `model/flux_allocation/system_params.m` | Centralized system constants (D_H matrix, coordinate transforms, LUT paths) |
| `model/main_system.slx` | Main Simulink model |

### Test Utilities

| Utility | Description |
|---------|-------------|
| `test_script/utils/plot_styles.m` | Unified color schemes, line widths, font sizes |
| `test_script/utils/fft_analysis.m` | FFT-based frequency response calculation |
| `test_script/utils/quality_check.m` | Steady-state, THD, DC offset detection |
| `test_script/utils/test_config.m` | Centralized test configuration parameters |

## Code Conventions

### Variable Naming for Pre-computed Operations

- Addition suffix: `_A_` (e.g., `one_A_beta = 1 + beta`)
- Subtraction suffix: `_S_` (e.g., `one_S_bc = 1 - bc`)
- Multiplication suffix: `_M_` (e.g., `b_M_lambda_c`)
- Division suffix: `_D_` (e.g., `a_D_b`)
- Negative prefix: `neg_` (e.g., `neg_beta = -beta`)

### Coordinate Systems

- **Measuring Coordinate**: Based on Hall sensor layout
- **Actuator Coordinate**: 6-pole configuration along X/Y/Z axes
- Transforms: `T_m2a` (Measuring→Actuator), `T_a2m` (Actuator→Measuring)

### Function Naming

- Model functions: `model_base_ctrl_*` prefix
- Test scripts: `run_*_test.m` or `run_*_bode.m`
- Utilities: Descriptive names (`plot_styles`, `fft_analysis`, etc.)

## Critical Constants

| Constant | Value | Description |
|----------|-------|-------------|
| Sampling rate | 100 kHz (Ts = 1e-5 s) | Controller execution rate |
| Normalization radius | R_norm = 550 µm | LUT address calculation |
| Force gain | FGain = 8.0 pN | Force model gain (g_H = g_I) |
| Position update rate | 1600 Hz | Hardware constraint for force control |

## Controller Parameters

### R-Controller Bandwidths

- `fB_f` - Feedforward filter bandwidth [Hz]
- `fB_c` - Control bandwidth [Hz]
- `fB_e` - Estimator bandwidth [Hz]

### PI Controller (Alternative)

- `Kp_value` - Proportional gain
- `Ki_value` - Integral gain (Ki = Kp × zc)
- `zc` - Zero location [rad/s]

## Test Output

Results are saved to timestamped directories:
```
test_results/
├── inner_loop/
│   ├── sine_wave/           # run_inner_loop_test.m (sine mode)
│   ├── step_response/       # run_inner_loop_test.m (step mode)
│   └── frequency_response/  # run_inner_loop_bode.m
└── force_generation/
    ├── force_control/       # run_force_generation_test.m
    └── force_bode/          # run_force_bode.m
```

Each test saves `.mat` data files and `.png` figures (300 DPI). All test results are gitignored.

## Utility Usage Examples

```matlab
% Using plot_styles
styles = plot_styles();
plot(x, y, 'Color', styles.channel_colors(1,:), ...
     'LineWidth', styles.measurement_linewidth);

% Using fft_analysis
results = fft_analysis(Vd_steady(:,1), Vm_steady, 100000, 100);
fprintf('Gain: %.2f%%, Phase: %.2f deg\n', ...
        results.magnitude_ratio*100, results.phase_lag_deg);

% Using quality_check
qc = quality_check(Vm, 100000, amplitude, 'Frequency', 100);
if qc.overall_pass(1)
    fprintf('Channel 1 passed all quality checks\n');
end

% Using test_config
config = test_config('Type', 'inner_loop');
params = model_base_ctrl_calc_params(config.fB_c, config.fB_e, config.fB_f);
```

---

## Plot Style Standards

統一的圖形樣式規範，確保所有測試輸出視覺一致性。

### Channel Colors (P1-P6)

| Channel | RGB Value | 說明 |
|---------|-----------|------|
| P1 | [0.0, 0.0, 0.5] | Dark Blue |
| P2 | [0.0, 0.0, 1.0] | Blue |
| P3 | [0.0, 0.5, 0.0] | Green |
| P4 | [1.0, 0.0, 0.0] | Red |
| P5 | [0.8, 0.0, 0.8] | Purple |
| P6 | [0.0, 0.75, 0.75] | Cyan |

### Force Axis Colors (X/Y/Z)

| Axis | RGB Value | 說明 |
|------|-----------|------|
| Fx | [0, 0, 1] | Blue |
| Fy | [0, 0.5, 0] | Green |
| Fz | [1, 0, 0] | Red |

### Line Width Standards

| 用途 | 線寬 | 說明 |
|------|------|------|
| 量測數據 | 3.0 | measurement_linewidth |
| 參考信號 | 2.5 | reference_linewidth |
| 理論曲線 | 3.5 | theory_linewidth |
| 座標軸 | 1.5 | axis_linewidth |
| Bode 圖 | 3.5 | bode_linewidth |

### Font Size Standards

| 元素 | 一般圖 | Bode 圖 |
|------|--------|---------|
| 標題 | 15 | 22 |
| 軸標籤 | 14 | 22 |
| 刻度 | 12 | 18 |
| 圖例 | 11 | 13 |

### Marker Standards

- Channel markers: `'o', 's', '^', 'd', 'v', 'p'` (依序對應 P1-P6)
- Force markers: `'o', 's', '^'` (依序對應 X/Y/Z)
- Marker size: 8 (一般), 9 (Bode)

### 圖檔輸出格式

- 格式: PNG
- 解析度: 300 DPI
- 命名: `{test_type}_{timestamp}.png`

---

## Frequency Response Validation

FFT 頻率響應分析方法論與驗證標準。

### FFT 分析方法論

1. **信號準備**
   - 跳過暫態週期（預設 60 週期）
   - 使用穩態週期進行分析（預設 40 週期）

2. **頻率分辨率**
   - 頻率 bin 誤差 < 0.1% 為合格
   - 若誤差過大，應調整模擬週期數

3. **相位計算**
   - 相位差歸一化至 [-180°, +180°]
   - 正值表示 lead，負值表示 lag

### 頻寬判定標準

| 指標 | 標準 |
|------|------|
| -3dB 頻寬 | 量測值與設計值誤差 < 10% |
| 相位 @-3dB | 應在 -30° ~ -60° 之間 |
| DC 增益 | 0 dB ± 0.5 dB |

### Inner Loop 頻率掃描點

建議使用以下頻率（確保整數週期）:

```
10, 50, 100, 125, 200, 250, 400, 500, 625, 800, 1000, 1250, 2000 Hz
```

### Force Control 頻率掃描點

受限於 1600 Hz 位置更新率，建議:

```
1, 10, 20, 50, 100, 125, 200, 250 Hz
```

---

## Signal Quality Check Standards

信號品質檢測規範，確保分析數據可靠性。

### 穩態判定（Steady-State Detection）

- 方法：連續週期差異比較
- 門檻：週期間最大差異 < 2% 振幅
- 比較週期數：最多 10 個週期

### THD 規範（Total Harmonic Distortion）

- 門檻：THD < 1%
- 諧波數：計算至第 10 次諧波
- 失敗處理：標記警告但不中斷測試

### DC 偏移規範

- 門檻：DC 成分 < 1% 振幅
- 計算：FFT 第 0 bin

### 綜合判定

- 所有檢測通過 → 數據可用
- 任一檢測失敗 → 輸出警告，標記需人工檢視

---

## Test Pass/Fail Criteria

測試結果判定標準。

### Inner Loop Test (run_inner_loop_test.m)

**Sine 模式：**

| 指標 | Pass 條件 |
|------|----------|
| 追蹤精度 | 振幅誤差 < 5% |
| 相位誤差 | < 10° @ 測試頻率 |
| 穩態收斂 | 60 週期內達穩態 |

**Step 模式：**

| 指標 | Pass 條件 |
|------|----------|
| 上升時間 | < 理論值 × 1.2 |
| 安定時間 | < 理論值 × 1.5 |
| 超越量 | < 10% |

### Inner Loop Bode (run_inner_loop_bode.m)

| 指標 | Pass 條件 |
|------|----------|
| 頻寬 | 實測/設計比 = 0.9 ~ 1.1 |
| 低頻增益 | 0 dB ± 0.5 dB |
| 相位交越 | 無正反饋跡象 |

### Force Generation Test (run_force_generation_test.m)

| 指標 | Pass 條件 |
|------|----------|
| 力量追蹤誤差 | RMS < 10% |
| 電壓追蹤誤差 | RMS < 5% |
| 串擾 | < -20 dB |

### Force Bode Test (run_force_bode.m)

| 指標 | Pass 條件 |
|------|----------|
| 頻寬 | > 100 Hz (受 1600 Hz 限制) |
| 主軸增益 | 0 dB ± 1 dB @ DC |
| 跨軸串擾 | < -15 dB |
