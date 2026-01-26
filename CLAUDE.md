# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ZPETC Package: A MATLAB/Simulink implementation of a Zero Phase Error Tracking Controller (ZPETC) for magnetic tweezers, controlling 6-pole electromagnetic actuators for precision biological force measurement at piconewton scale.

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
│   ├── force_ctrl/                   # Force control pipeline tests
│   │   ├── run_force_control_test.m  # Force control integration test
│   │   └── run_force_bode_test.m     # Force control frequency sweep
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

% Run force control tests
run('test_script/force_ctrl/run_force_control_test.m')
run('test_script/force_ctrl/run_force_bode_test.m')
```

## Architecture

### Three-Phase Design

1. **Inner Loop Controller** (Phase 1, Complete): Discrete-time feedback control with feedforward filter, disturbance observer, and PI control law
2. **Physical Models** (Phase 2): `inverse_model.m` (force→voltage) and `force_model.m` (voltage→force)
3. **Full Integration** (Phase 3): Closed-loop force control pipeline

### Signal Flow

```
Desired Force (f_d) → Inverse Model → v_d → ZPETC → u (currents) → Plant → v_m → Force Model → f_m
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

### ZPETC Bandwidths

- `fB_f` - Feedforward filter bandwidth [Hz]
- `fB_c` - Control bandwidth [Hz]
- `fB_e` - Estimator bandwidth [Hz]

### PI Controller (Alternative to ZPETC)

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
└── force_ctrl/
    ├── force_control/       # run_force_control_test.m
    └── force_bode/          # run_force_bode_test.m
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
