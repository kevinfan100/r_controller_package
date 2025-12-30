# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

R-Controller Package: A MATLAB/Simulink implementation of an R-Controller for magnetic tweezers, controlling 6-pole electromagnetic actuators for precision biological force measurement at piconewton scale.

## Development Commands

```matlab
% Add paths and load parameters
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'model'));

% Initialize controller parameters (fB_c, fB_e, fB_f in Hz)
ctrl_params = r_controller_calc_params(500, 500, 3000);

% Run primary R-Controller test
run('test_script/run_rcontroller_test.m')

% Run frequency sweep (Bode plot generation)
run('test_script/run_frequency_sweep.m')

% Run force control pipeline test
run('test_script/run_force_control_test.m')
```

## Architecture

**Three-Phase Design:**
1. **R-Controller** (Phase 1, Complete): Discrete-time feedback control with feedforward filter, disturbance observer, and PI control law
2. **Physical Models** (Phase 2): `inverse_model.m` (force→voltage) and `force_model.m` (voltage→force)
3. **Full Integration** (Phase 3): Closed-loop force control pipeline

**Signal Flow:**
```
Desired Force (f_d) → Inverse Model → v_d → R-Controller → u (currents) → Plant → v_m → Force Model → f_m
```

**Key Files:**
- `model/r_controller_calc_params.m` - Computes controller coefficients from bandwidth parameters, creates ParamsBus
- `model/r_controller_function_general.m` - Core discrete-time difference equations (runs at 100 kHz)
- `model/r_controller_system_integrated.slx` - Main Simulink model
- `model/system_params.m` - Centralized system constants (D_H matrix, coordinate transforms, LUT paths)
- `model/inverse_model.m` - Force to Hall voltage conversion using LUT interpolation
- `model/force_model.m` - Hall voltage to estimated force using L-matrix calculations
- `data/lut/` - Lookup tables (1891x20) for inverse model polynomial interpolation

## Code Conventions

**Variable Naming for Pre-computed Operations:**
- Addition suffix: `_A_` (e.g., `one_A_beta = 1 + beta`)
- Subtraction suffix: `_S_` (e.g., `one_S_bc = 1 - bc`)
- Multiplication suffix: `_M_` (e.g., `b_M_lambda_c`)
- Division suffix: `_D_` (e.g., `a_D_b`)
- Negative prefix: `neg_` (e.g., `neg_beta = -beta`)

**Coordinate Systems:**
- **Measuring Coordinate**: Based on Hall sensor layout
- **Actuator Coordinate**: 6-pole configuration along X/Y/Z axes
- Transforms: `T_m2a` (Measuring→Actuator), `T_a2m` (Actuator→Measuring)

## Critical Constants

- Sampling rate: 100 kHz (Ts = 1e-5 s)
- Normalization radius: R_norm = 550 µm
- Force gain: FGain = 4.0 pN
- Coupling matrix B: 6x6 (pre-compute inverse for efficiency)

## Test Output

Results are saved to timestamped directories under `test_results/` with `.mat` data files and `.png` figures. These are gitignored.
