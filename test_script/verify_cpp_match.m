%% verify_cpp_match.m
% Direct numerical verification against C++ test results
% This script replicates exactly what the C++ test program does
%
% C++ Test Results (reference):
%   Input Position (um): 0, 0, 0
%   Input Force    (pN): 5, 0, 0
%   Output Force   (pN): 5, 0, 0
%   Error          (pN): 1.89339e-07, 0, 0
%   Current I1-I6: 0.330523 -0.071266 0.0520834 0.0520834 -0.181712 -0.181712
%   Hall Voltage V_m: 0.744338 -0.19591 0.116979 0.119479 -0.514608 -0.417029
%   Hall Force (pN): 5, 0, -2.22045e-16
%   Difference (pN): 2.66454e-15, 0, -2.22045e-16

clear; clc;
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('        MATLAB vs C++ Numerical Verification\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

%% Add paths
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'model'));

%% Load system parameters
params = system_params();

%% Test inputs (same as C++)
pos_um = [0; 0; 0];      % Position in um (Measuring coordinate)
f_d_pN = [5; 0; 0];      % Desired force in pN (Measuring coordinate)

fprintf('=== Test Inputs ===\n');
fprintf('Position (um): [%.1f, %.1f, %.1f]\n', pos_um);
fprintf('Force (pN):    [%.1f, %.1f, %.1f]\n', f_d_pN);

%% C++ Reference Values
cpp_I1toI6 = [0.330523, -0.071266, 0.0520834, 0.0520834, -0.181712, -0.181712]';
cpp_V_m = [0.744338, -0.19591, 0.116979, 0.119479, -0.514608, -0.417029]';
cpp_F_curr = [5, 0, 0]';  % Current-based force output
cpp_F_hall = [5, 0, -2.22045e-16]';  % Hall sensor-based force output

fprintf('\n=== C++ Reference Values ===\n');
fprintf('C++ Current I1-I6: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', cpp_I1toI6);
fprintf('C++ Hall Voltage:  [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', cpp_V_m);

%% ═══════════════════════════════════════════════════════════════════════
%                    STEP 1: Inverse Model (f_d -> I_d -> v_d)
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('              STEP 1: Inverse Model Test\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

% Call inverse_model
v_d = inverse_model(f_d_pN, pos_um, params);

% Back-calculate I_d from v_d
% v_d = D_H^-1 * K_I * I_d
% So: I_d = pinv(D_H^-1 * K_I) * v_d
I_d_from_vd = pinv(params.DH_inv_KI) * v_d;

fprintf('\nMATLAB Results:\n');
fprintf('  v_d:        [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', v_d);
fprintf('  I_d (calc): [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', I_d_from_vd);

fprintf('\nComparison with C++:\n');
fprintf('  v_d diff:   [%.2e, %.2e, %.2e, %.2e, %.2e, %.2e]\n', v_d' - cpp_V_m');
fprintf('  I_d diff:   [%.2e, %.2e, %.2e, %.2e, %.2e, %.2e]\n', I_d_from_vd' - cpp_I1toI6');

%% ═══════════════════════════════════════════════════════════════════════
%                    STEP 2: Force Model Tests
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('              STEP 2: Force Model Tests\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

%% 2a. Current-based Force Model (C++ method: F = I^T * N * I)
fprintf('\n--- Method A: Current-based (F = I^T * N * I) ---\n');

% Transform position to actuator coordinate
pos_a = params.T_m2a * pos_um;

% Calculate L matrices at this position
[Lx, Ly, Lz] = calc_L_matrices_test(pos_a, params.R_norm);

% Calculate N = K_I^T * L * K_I
Nx = params.KI_theo' * Lx * params.KI_theo;
Ny = params.KI_theo' * Ly * params.KI_theo;
Nz = params.KI_theo' * Lz * params.KI_theo;

% Use C++ currents for fair comparison
I = cpp_I1toI6;

% F = FGain * I^T * N * I
Fx_curr = params.FGain * (I' * Nx * I);
Fy_curr = params.FGain * (I' * Ny * I);
Fz_curr = params.FGain * (I' * Nz * I);
F_curr_a = [Fx_curr; Fy_curr; Fz_curr];

% Transform back to measuring coordinate
F_curr_m = params.T_a2m * F_curr_a;

fprintf('  F_a (Actuator): [%.6f, %.6f, %.6f] pN\n', F_curr_a);
fprintf('  F_m (Measuring): [%.6f, %.6f, %.6f] pN\n', F_curr_m);
fprintf('  Error vs f_d:   [%.2e, %.2e, %.2e] pN\n', F_curr_m' - f_d_pN');

%% 2b. Hall Sensor-based Force Model (C++ method: F = Phi^T * L * Phi)
fprintf('\n--- Method B: Hall Sensor-based (F = Phi^T * L * Phi) ---\n');

% Use C++ Hall voltage
V_m = cpp_V_m;

% Phi = D_H * V_m
Phi = params.D_H * V_m;

fprintf('  Phi: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', Phi);

% F = FGain * Phi^T * L * Phi
Fx_hall = params.FGain * (Phi' * Lx * Phi);
Fy_hall = params.FGain * (Phi' * Ly * Phi);
Fz_hall = params.FGain * (Phi' * Lz * Phi);
F_hall_a = [Fx_hall; Fy_hall; Fz_hall];

% Transform back to measuring coordinate
F_hall_m = params.T_a2m * F_hall_a;

fprintf('  F_a (Actuator): [%.6f, %.6f, %.6f] pN\n', F_hall_a);
fprintf('  F_m (Measuring): [%.6f, %.6f, %.6f] pN\n', F_hall_m);
fprintf('  Error vs f_d:   [%.2e, %.2e, %.2e] pN\n', F_hall_m' - f_d_pN');

%% 2c. MATLAB current force_model.m (whatever it does now)
fprintf('\n--- Method C: Current MATLAB force_model.m ---\n');

% Use C++ Hall voltage as input
f_m_matlab = force_model(cpp_V_m, pos_um, params);

fprintf('  f_m (Measuring): [%.6f, %.6f, %.6f] pN\n', f_m_matlab);
fprintf('  Error vs f_d:   [%.2e, %.2e, %.2e] pN\n', f_m_matlab' - f_d_pN');
fprintf('  Ratio f_m/f_d:  [%.6f, %.6f, %.6f]\n', f_m_matlab' ./ max(f_d_pN', 1e-10));

%% ═══════════════════════════════════════════════════════════════════════
%                    STEP 3: Matrix Comparison
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('              STEP 3: Matrix Comparison at Origin\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

% C++ Nx_KItheo matrix (from test output)
cpp_Nx = [
    4,    5.55112e-17, -1, -1, -1, -1;
    -5.55112e-17, -4,  1,  1,  1,  1;
    -1,   1,          0,  0,  0,  0;
    -1,   1,          0,  0,  0,  0;
    -1,   1,          0,  0,  0,  0;
    -1,   1,          0,  0,  0,  0
];

% C++ Lx matrix (from test output)
cpp_Lx = [
    4,  0, -1, -1, -1, -1;
    0, -4,  1,  1,  1,  1;
   -1,  1,  0,  0,  0,  0;
   -1,  1,  0,  0,  0,  0;
   -1,  1,  0,  0,  0,  0;
   -1,  1,  0,  0,  0,  0
];

fprintf('\n--- Lx Matrix Comparison ---\n');
fprintf('MATLAB Lx:\n');
disp(round(Lx, 6));
fprintf('C++ Lx:\n');
disp(cpp_Lx);
fprintf('Max diff: %.2e\n', max(abs(Lx(:) - cpp_Lx(:))));

fprintf('\n--- Nx Matrix Comparison ---\n');
fprintf('MATLAB Nx = K_I^T * Lx * K_I:\n');
disp(round(Nx, 6));
fprintf('C++ Nx_KItheo:\n');
disp(round(cpp_Nx, 6));
fprintf('Max diff: %.2e\n', max(abs(Nx(:) - cpp_Nx(:))));

%% ═══════════════════════════════════════════════════════════════════════
%                    STEP 4: Full Round-Trip Test
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('              STEP 4: Full Round-Trip Test\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

fprintf('\nTest: f_d -> inverse_model -> v_d -> force_model -> f_m\n');

% Step 1: Inverse model
v_d_roundtrip = inverse_model(f_d_pN, pos_um, params);

% Step 2: Force model (current MATLAB implementation)
f_m_roundtrip = force_model(v_d_roundtrip, pos_um, params);

fprintf('\n  f_d (input):   [%.6f, %.6f, %.6f] pN\n', f_d_pN);
fprintf('  v_d (inverse): [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', v_d_roundtrip);
fprintf('  f_m (force):   [%.6f, %.6f, %.6f] pN\n', f_m_roundtrip);
fprintf('  Error:         [%.6f, %.6f, %.6f] pN\n', f_m_roundtrip - f_d_pN);
fprintf('  Relative:      %.2f%%\n', norm(f_m_roundtrip - f_d_pN) / norm(f_d_pN) * 100);

%% ═══════════════════════════════════════════════════════════════════════
%                    STEP 5: Diagnose Current MATLAB Force Model
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('              STEP 5: Force Model Diagnosis\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

% What does the current MATLAB force_model do?
% According to force_model.m (2025-12-29 fix):
%   Phi = D_H * v_m
%   I = pinv(K_I) * Phi
%   F = g_I * I^T * L * I

V_m_test = v_d_roundtrip;
Phi_test = params.D_H * V_m_test;
I_recovered = params.KI_pinv * Phi_test;

fprintf('\nStep-by-step trace of current force_model.m:\n');
fprintf('  V_m (input): [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', V_m_test);
fprintf('  Phi = D_H * V_m: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', Phi_test);
fprintf('  I = pinv(K_I) * Phi: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', I_recovered);

% Current MATLAB formula: F = g_I * I^T * L * I (WRONG!)
Fx_wrong = params.g_I * (I_recovered' * Lx * I_recovered);
Fy_wrong = params.g_I * (I_recovered' * Ly * I_recovered);
Fz_wrong = params.g_I * (I_recovered' * Lz * I_recovered);

fprintf('\n  Current MATLAB: F = g_I * I^T * L * I\n');
fprintf('    F_a: [%.6f, %.6f, %.6f]\n', [Fx_wrong, Fy_wrong, Fz_wrong]);

% Correct Method A: F = g_I * I^T * N * I (N = K_I^T * L * K_I)
Fx_A = params.g_I * (I_recovered' * Nx * I_recovered);
Fy_A = params.g_I * (I_recovered' * Ny * I_recovered);
Fz_A = params.g_I * (I_recovered' * Nz * I_recovered);

fprintf('\n  Correct Method A: F = g_I * I^T * N * I\n');
fprintf('    F_a: [%.6f, %.6f, %.6f]\n', [Fx_A, Fy_A, Fz_A]);

% Correct Method B: F = g_I * Phi^T * L * Phi (original)
Fx_B = params.g_I * (Phi_test' * Lx * Phi_test);
Fy_B = params.g_I * (Phi_test' * Ly * Phi_test);
Fz_B = params.g_I * (Phi_test' * Lz * Phi_test);

fprintf('\n  Correct Method B: F = g_I * Phi^T * L * Phi\n');
fprintf('    F_a: [%.6f, %.6f, %.6f]\n', [Fx_B, Fy_B, Fz_B]);

%% Summary
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('                        SUMMARY\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\nExpected f_a (Actuator frame): Depends on T_m2a transform\n');
fprintf('f_d in Actuator frame: [%.6f, %.6f, %.6f]\n', params.T_m2a * f_d_pN);
fprintf('\n');

%% ════════════════════════════════════════════════════════════════════════
%                    Helper Function: L Matrix Calculation
%% ════════════════════════════════════════════════════════════════════════
function [Lx, Ly, Lz] = calc_L_matrices_test(pos_um, R_norm)
% Copied from force_model.m calc_L_matrices_full for standalone use

    x = pos_um(1) / R_norm;
    y = pos_um(2) / R_norm;
    z = pos_um(3) / R_norm;

    Dx1_x = -x; Dx2_x = -x; Dx3_x = -x; Dx4_x = -x; Dx5_x = -x; Dx6_x = -x;
    Dy1_y = -y; Dy2_y = -y; Dy3_y = -y; Dy4_y = -y; Dy5_y = -y; Dy6_y = -y;
    Dz1_z = -z; Dz2_z = -z; Dz3_z = -z; Dz4_z = -z; Dz5_z = -z; Dz6_z = -z;

    Dx1_xPlus1 = -x + 1;
    x_Dx2Plus1 = x + 1;
    Dy3_yPlus1 = -y + 1;
    y_Dy4Plus1 = y + 1;
    Dz5_zPlus1 = -z + 1;
    z_Dz6Plus1 = z + 1;

    Sq_r1 = Dx1_xPlus1^2 + Dy1_y^2 + Dz1_z^2;
    Sq_r2 = x_Dx2Plus1^2 + Dy2_y^2 + Dz2_z^2;
    Sq_r3 = Dx3_x^2 + Dy3_yPlus1^2 + Dz3_z^2;
    Sq_r4 = Dx4_x^2 + y_Dy4Plus1^2 + Dz4_z^2;
    Sq_r5 = Dx5_x^2 + Dy5_y^2 + Dz5_zPlus1^2;
    Sq_r6 = Dx6_x^2 + Dy6_y^2 + z_Dz6Plus1^2;

    % Lx
    Lx = zeros(6, 6);
    Lx(1,1) = 4*Dx1_xPlus1 / Sq_r1^3;
    Lx(1,2) = (((3*Dy1_y*Dy2_y - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz1_z*Dz2_z)/Sq_r1 - 1)*Dx1_xPlus1 ...
             - ((3*Dy1_y*Dy2_y - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz1_z*Dz2_z)/Sq_r2 - 1)*x_Dx2Plus1) ...
             / (Sq_r1^1.5 * Sq_r2^1.5);
    Lx(1,3) = (((3*Dx3_x*Dx1_xPlus1 + 3*Dy1_y*Dy3_yPlus1 + 3*Dz1_z*Dz3_z)/Sq_r1 - 1)*Dx1_xPlus1 ...
             + Dx3_x*((3*Dx3_x*Dx1_xPlus1 + 3*Dy1_y*Dy3_yPlus1 + 3*Dz1_z*Dz3_z)/Sq_r3 - 1)) ...
             / (Sq_r1^1.5 * Sq_r3^1.5);
    Lx(1,4) = (((3*Dx4_x*Dx1_xPlus1 - 3*Dy1_y*y_Dy4Plus1 + 3*Dz1_z*Dz4_z)/Sq_r1 - 1)*Dx1_xPlus1 ...
             + Dx4_x*((3*Dx4_x*Dx1_xPlus1 - 3*Dy1_y*y_Dy4Plus1 + 3*Dz1_z*Dz4_z)/Sq_r4 - 1)) ...
             / (Sq_r1^1.5 * Sq_r4^1.5);
    Lx(1,5) = (((3*Dx5_x*Dx1_xPlus1 + 3*Dy1_y*Dy5_y + 3*Dz1_z*Dz5_zPlus1)/Sq_r1 - 1)*Dx1_xPlus1 ...
             + Dx5_x*((3*Dx5_x*Dx1_xPlus1 + 3*Dy1_y*Dy5_y + 3*Dz1_z*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r1^1.5 * Sq_r5^1.5);
    Lx(1,6) = (((3*Dx6_x*Dx1_xPlus1 + 3*Dy1_y*Dy6_y - 3*Dz1_z*z_Dz6Plus1)/Sq_r1 - 1)*Dx1_xPlus1 ...
             + Dx6_x*((3*Dx6_x*Dx1_xPlus1 + 3*Dy1_y*Dy6_y - 3*Dz1_z*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r1^1.5 * Sq_r6^1.5);

    Lx(2,1) = Lx(1,2);
    Lx(2,2) = -4*x_Dx2Plus1 / Sq_r2^3;
    Lx(2,3) = -(((3*Dy2_y*Dy3_yPlus1 - 3*Dx3_x*x_Dx2Plus1 + 3*Dz2_z*Dz3_z)/Sq_r2 - 1)*x_Dx2Plus1 ...
              - Dx3_x*((3*Dy2_y*Dy3_yPlus1 - 3*Dx3_x*x_Dx2Plus1 + 3*Dz2_z*Dz3_z)/Sq_r3 - 1)) ...
              / (Sq_r2^1.5 * Sq_r3^1.5);
    Lx(2,4) = (((3*Dx4_x*x_Dx2Plus1 + 3*Dy2_y*y_Dy4Plus1 - 3*Dz2_z*Dz4_z)/Sq_r2 + 1)*x_Dx2Plus1 ...
             - Dx4_x*((3*Dx4_x*x_Dx2Plus1 + 3*Dy2_y*y_Dy4Plus1 - 3*Dz2_z*Dz4_z)/Sq_r4 + 1)) ...
             / (Sq_r2^1.5 * Sq_r4^1.5);
    Lx(2,5) = -(((3*Dy2_y*Dy5_y - 3*Dx5_x*x_Dx2Plus1 + 3*Dz2_z*Dz5_zPlus1)/Sq_r2 - 1)*x_Dx2Plus1 ...
              - Dx5_x*((3*Dy2_y*Dy5_y - 3*Dx5_x*x_Dx2Plus1 + 3*Dz2_z*Dz5_zPlus1)/Sq_r5 - 1)) ...
              / (Sq_r2^1.5 * Sq_r5^1.5);
    Lx(2,6) = (((3*Dx6_x*x_Dx2Plus1 - 3*Dy2_y*Dy6_y + 3*Dz2_z*z_Dz6Plus1)/Sq_r2 + 1)*x_Dx2Plus1 ...
             - Dx6_x*((3*Dx6_x*x_Dx2Plus1 - 3*Dy2_y*Dy6_y + 3*Dz2_z*z_Dz6Plus1)/Sq_r6 + 1)) ...
             / (Sq_r2^1.5 * Sq_r6^1.5);

    Lx(3,1) = Lx(1,3); Lx(3,2) = Lx(2,3);
    Lx(3,3) = 4*Dx3_x / Sq_r3^3;
    Lx(3,4) = (Dx3_x*((3*Dx3_x*Dx4_x - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz3_z*Dz4_z)/Sq_r3 - 1) ...
             + Dx4_x*((3*Dx3_x*Dx4_x - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz3_z*Dz4_z)/Sq_r4 - 1)) ...
             / (Sq_r3^1.5 * Sq_r4^1.5);
    Lx(3,5) = (Dx3_x*((3*Dx3_x*Dx5_x + 3*Dy5_y*Dy3_yPlus1 + 3*Dz3_z*Dz5_zPlus1)/Sq_r3 - 1) ...
             + Dx5_x*((3*Dx3_x*Dx5_x + 3*Dy5_y*Dy3_yPlus1 + 3*Dz3_z*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r3^1.5 * Sq_r5^1.5);
    Lx(3,6) = (Dx3_x*((3*Dx3_x*Dx6_x + 3*Dy6_y*Dy3_yPlus1 - 3*Dz3_z*z_Dz6Plus1)/Sq_r3 - 1) ...
             + Dx6_x*((3*Dx3_x*Dx6_x + 3*Dy6_y*Dy3_yPlus1 - 3*Dz3_z*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r3^1.5 * Sq_r6^1.5);

    Lx(4,1) = Lx(1,4); Lx(4,2) = Lx(2,4); Lx(4,3) = Lx(3,4);
    Lx(4,4) = 4*Dx4_x / Sq_r4^3;
    Lx(4,5) = (Dx4_x*((3*Dx4_x*Dx5_x - 3*Dy5_y*y_Dy4Plus1 + 3*Dz4_z*Dz5_zPlus1)/Sq_r4 - 1) ...
             + Dx5_x*((3*Dx4_x*Dx5_x - 3*Dy5_y*y_Dy4Plus1 + 3*Dz4_z*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r4^1.5 * Sq_r5^1.5);
    Lx(4,6) = -(Dx4_x*((3*Dy6_y*y_Dy4Plus1 - 3*Dx4_x*Dx6_x + 3*Dz4_z*z_Dz6Plus1)/Sq_r4 + 1) ...
              + Dx6_x*((3*Dy6_y*y_Dy4Plus1 - 3*Dx4_x*Dx6_x + 3*Dz4_z*z_Dz6Plus1)/Sq_r6 + 1)) ...
              / (Sq_r4^1.5 * Sq_r6^1.5);

    Lx(5,1) = Lx(1,5); Lx(5,2) = Lx(2,5); Lx(5,3) = Lx(3,5); Lx(5,4) = Lx(4,5);
    Lx(5,5) = 4*Dx5_x / Sq_r5^3;
    Lx(5,6) = (Dx5_x*((3*Dx5_x*Dx6_x + 3*Dy5_y*Dy6_y - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r5 - 1) ...
             + Dx6_x*((3*Dx5_x*Dx6_x + 3*Dy5_y*Dy6_y - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r5^1.5 * Sq_r6^1.5);

    Lx(6,1) = Lx(1,6); Lx(6,2) = Lx(2,6); Lx(6,3) = Lx(3,6); Lx(6,4) = Lx(4,6); Lx(6,5) = Lx(5,6);
    Lx(6,6) = 4*Dx6_x / Sq_r6^3;

    % Ly (simplified for origin)
    Ly = zeros(6, 6);
    Ly(1,1) = 4*Dy1_y / Sq_r1^3;
    Ly(1,2) = (Dy1_y*((3*Dy1_y*Dy2_y - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz1_z*Dz2_z)/Sq_r1 - 1) ...
             + Dy2_y*((3*Dy1_y*Dy2_y - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz1_z*Dz2_z)/Sq_r2 - 1)) ...
             / (Sq_r1^1.5 * Sq_r2^1.5);
    Ly(2,1) = Ly(1,2);
    Ly(2,2) = 4*Dy2_y / Sq_r2^3;
    Ly(3,3) = 4*Dy3_yPlus1 / Sq_r3^3;
    Ly(4,4) = -4*y_Dy4Plus1 / Sq_r4^3;
    Ly(5,5) = 4*Dy5_y / Sq_r5^3;
    Ly(6,6) = 4*Dy6_y / Sq_r6^3;

    % Lz (simplified for origin)
    Lz = zeros(6, 6);
    Lz(1,1) = 4*Dz1_z / Sq_r1^3;
    Lz(2,2) = 4*Dz2_z / Sq_r2^3;
    Lz(3,3) = 4*Dz3_z / Sq_r3^3;
    Lz(4,4) = 4*Dz4_z / Sq_r4^3;
    Lz(5,5) = 4*Dz5_zPlus1 / Sq_r5^3;
    Lz(6,6) = -4*z_Dz6Plus1 / Sq_r6^3;
end
