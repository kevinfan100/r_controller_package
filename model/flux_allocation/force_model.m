function f_m = force_model(v_m, pos_m, params)
% FORCE_MODEL Hall sensor voltage -> Estimated force
%
% Formula: F = g_H * Phi_hat' * L * Phi_hat
%   Phi_hat = D_H_hat * V_m (normalized flux)
%   L = position-dependent gradient matrices
%
% Inputs:
%   v_m    - Hall sensor voltage (6x1) [V]
%   pos_m  - Bead position (3x1), Measuring coordinate [um]
%   params - System parameters from system_params()
%
% Output:
%   f_m    - Estimated force (3x1), Measuring coordinate [pN]

    % ========================================
    % STEP 1: V_m -> Phi_hat
    % ========================================
    Phi = params.D_H_hat * v_m;

    % ========================================
    % STEP 2: POSITION TRANSFORM
    % ========================================
    pos_a = params.T_m2a * pos_m;

    % ========================================
    % STEP 3: L MATRICES
    % ========================================
    [Lx, Ly, Lz] = calc_L_matrices(pos_a, params.R_norm);

    % ========================================
    % STEP 4: QUADRATIC FORM
    % ========================================
    % F = g_H * Phi' * L * Phi
    Fx_a = params.g_H * (Phi' * Lx * Phi);
    Fy_a = params.g_H * (Phi' * Ly * Phi);
    Fz_a = params.g_H * (Phi' * Lz * Phi);

    % ========================================
    % STEP 5: ACTUATOR -> MEASURING
    % ========================================
    f_m = params.T_a2m * [Fx_a; Fy_a; Fz_a];

end


% ========================================
% L MATRIX CALCULATION
% ========================================
function [Lx, Ly, Lz] = calc_L_matrices(pos_um, R_norm)
% CALC_L_MATRICES Position-dependent gradient matrices
%
% Ported from CALCULATION.cpp Calc_Pre_L, Calc_Lx, Calc_Ly, Calc_Lz
%
% Inputs:
%   pos_um - Bead position [x, y, z] in um (Actuator Coordinate)
%   R_norm - Normalization radius (550 um)
%
% Outputs:
%   Lx, Ly, Lz - 6x6 symmetric gradient matrices

    % ========================================
    % NORMALIZE POSITION
    % ========================================
    x = pos_um(1) / R_norm;
    y = pos_um(2) / R_norm;
    z = pos_um(3) / R_norm;

    % ========================================
    % POLE DISTANCE TERMS
    % ========================================
    % 6 poles at normalized distance 1 from origin
    % Dx/Dy/Dz terms for position relative to each pole
    Dx1_xPlus1 = 1 - x;   % Pole 1: +X
    x_Dx2Plus1 = 1 + x;   % Pole 2: -X
    Dy3_yPlus1 = 1 - y;   % Pole 3: +Y
    y_Dy4Plus1 = 1 + y;   % Pole 4: -Y
    Dz5_zPlus1 = 1 - z;   % Pole 5: +Z
    z_Dz6Plus1 = 1 + z;   % Pole 6: -Z

    % Difference terms (with Bias = 0, all are simply -pos)
    Dx_all = -x;
    Dy_all = -y;
    Dz_all = -z;

    % ========================================
    % SQUARED DISTANCES
    % ========================================
    Sq_r1 = Dx1_xPlus1^2 + y^2 + z^2;
    Sq_r2 = x_Dx2Plus1^2 + y^2 + z^2;
    Sq_r3 = x^2 + Dy3_yPlus1^2 + z^2;
    Sq_r4 = x^2 + y_Dy4Plus1^2 + z^2;
    Sq_r5 = x^2 + y^2 + Dz5_zPlus1^2;
    Sq_r6 = x^2 + y^2 + z_Dz6Plus1^2;

    % ========================================
    % Lx MATRIX
    % ========================================
    Lx = zeros(6, 6);

    % Row 1
    Lx(1,1) = 4*Dx1_xPlus1 / Sq_r1^3;
    Lx(1,2) = (((3*Dy_all*Dy_all - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r1 - 1)*Dx1_xPlus1 ...
             - ((3*Dy_all*Dy_all - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r2 - 1)*x_Dx2Plus1) ...
             / (Sq_r1^1.5 * Sq_r2^1.5);
    Lx(1,3) = (((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz_all)/Sq_r1 - 1)*Dx1_xPlus1 ...
             + Dx_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz_all)/Sq_r3 - 1)) ...
             / (Sq_r1^1.5 * Sq_r3^1.5);
    Lx(1,4) = (((3*Dx_all*Dx1_xPlus1 - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r1 - 1)*Dx1_xPlus1 ...
             + Dx_all*((3*Dx_all*Dx1_xPlus1 - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r4 - 1)) ...
             / (Sq_r1^1.5 * Sq_r4^1.5);
    Lx(1,5) = (((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all + 3*Dz_all*Dz5_zPlus1)/Sq_r1 - 1)*Dx1_xPlus1 ...
             + Dx_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r1^1.5 * Sq_r5^1.5);
    Lx(1,6) = (((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all - 3*Dz_all*z_Dz6Plus1)/Sq_r1 - 1)*Dx1_xPlus1 ...
             + Dx_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all - 3*Dz_all*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r1^1.5 * Sq_r6^1.5);

    % Row 2
    Lx(2,1) = Lx(1,2);
    Lx(2,2) = -4*x_Dx2Plus1 / Sq_r2^3;
    Lx(2,3) = -(((3*Dy_all*Dy3_yPlus1 - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r2 - 1)*x_Dx2Plus1 ...
              - Dx_all*((3*Dy_all*Dy3_yPlus1 - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r3 - 1)) ...
              / (Sq_r2^1.5 * Sq_r3^1.5);
    Lx(2,4) = (((3*Dx_all*x_Dx2Plus1 + 3*Dy_all*y_Dy4Plus1 - 3*Dz_all*Dz_all)/Sq_r2 + 1)*x_Dx2Plus1 ...
             - Dx_all*((3*Dx_all*x_Dx2Plus1 + 3*Dy_all*y_Dy4Plus1 - 3*Dz_all*Dz_all)/Sq_r4 + 1)) ...
             / (Sq_r2^1.5 * Sq_r4^1.5);
    Lx(2,5) = -(((3*Dy_all*Dy_all - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r2 - 1)*x_Dx2Plus1 ...
              - Dx_all*((3*Dy_all*Dy_all - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)) ...
              / (Sq_r2^1.5 * Sq_r5^1.5);
    Lx(2,6) = (((3*Dx_all*x_Dx2Plus1 - 3*Dy_all*Dy_all + 3*Dz_all*z_Dz6Plus1)/Sq_r2 + 1)*x_Dx2Plus1 ...
             - Dx_all*((3*Dx_all*x_Dx2Plus1 - 3*Dy_all*Dy_all + 3*Dz_all*z_Dz6Plus1)/Sq_r6 + 1)) ...
             / (Sq_r2^1.5 * Sq_r6^1.5);

    % Row 3
    Lx(3,1) = Lx(1,3);
    Lx(3,2) = Lx(2,3);
    Lx(3,3) = 4*Dx_all / Sq_r3^3;
    Lx(3,4) = (Dx_all*((3*Dx_all*Dx_all - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r3 - 1) ...
             + Dx_all*((3*Dx_all*Dx_all - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r4 - 1)) ...
             / (Sq_r3^1.5 * Sq_r4^1.5);
    Lx(3,5) = (Dx_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r3 - 1) ...
             + Dx_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r3^1.5 * Sq_r5^1.5);
    Lx(3,6) = (Dx_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 - 3*Dz_all*z_Dz6Plus1)/Sq_r3 - 1) ...
             + Dx_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 - 3*Dz_all*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r3^1.5 * Sq_r6^1.5);

    % Row 4
    Lx(4,1) = Lx(1,4);
    Lx(4,2) = Lx(2,4);
    Lx(4,3) = Lx(3,4);
    Lx(4,4) = 4*Dx_all / Sq_r4^3;
    Lx(4,5) = (Dx_all*((3*Dx_all*Dx_all - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r4 - 1) ...
             + Dx_all*((3*Dx_all*Dx_all - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r4^1.5 * Sq_r5^1.5);
    Lx(4,6) = -(Dx_all*((3*Dy_all*y_Dy4Plus1 - 3*Dx_all*Dx_all + 3*Dz_all*z_Dz6Plus1)/Sq_r4 + 1) ...
              + Dx_all*((3*Dy_all*y_Dy4Plus1 - 3*Dx_all*Dx_all + 3*Dz_all*z_Dz6Plus1)/Sq_r6 + 1)) ...
              / (Sq_r4^1.5 * Sq_r6^1.5);

    % Row 5
    Lx(5,1) = Lx(1,5);
    Lx(5,2) = Lx(2,5);
    Lx(5,3) = Lx(3,5);
    Lx(5,4) = Lx(4,5);
    Lx(5,5) = 4*Dx_all / Sq_r5^3;
    Lx(5,6) = (Dx_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy_all - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r5 - 1) ...
             + Dx_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy_all - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r5^1.5 * Sq_r6^1.5);

    % Row 6
    Lx(6,1) = Lx(1,6);
    Lx(6,2) = Lx(2,6);
    Lx(6,3) = Lx(3,6);
    Lx(6,4) = Lx(4,6);
    Lx(6,5) = Lx(5,6);
    Lx(6,6) = 4*Dx_all / Sq_r6^3;

    % ========================================
    % Ly MATRIX
    % ========================================
    Ly = zeros(6, 6);

    % Row 1
    Ly(1,1) = 4*Dy_all / Sq_r1^3;
    Ly(1,2) = (Dy_all*((3*Dy_all*Dy_all - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r1 - 1) ...
             + Dy_all*((3*Dy_all*Dy_all - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r2 - 1)) ...
             / (Sq_r1^1.5 * Sq_r2^1.5);
    Ly(1,3) = (Dy_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz_all)/Sq_r1 - 1) ...
             + ((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz_all)/Sq_r3 - 1)*Dy3_yPlus1) ...
             / (Sq_r1^1.5 * Sq_r3^1.5);
    Ly(1,4) = (Dy_all*((3*Dx_all*Dx1_xPlus1 - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r1 - 1) ...
             - ((3*Dx_all*Dx1_xPlus1 - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r4 - 1)*y_Dy4Plus1) ...
             / (Sq_r1^1.5 * Sq_r4^1.5);
    Ly(1,5) = (Dy_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all + 3*Dz_all*Dz5_zPlus1)/Sq_r1 - 1) ...
             + Dy_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r1^1.5 * Sq_r5^1.5);
    Ly(1,6) = (Dy_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all - 3*Dz_all*z_Dz6Plus1)/Sq_r1 - 1) ...
             + Dy_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all - 3*Dz_all*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r1^1.5 * Sq_r6^1.5);

    % Row 2
    Ly(2,1) = Ly(1,2);
    Ly(2,2) = 4*Dy_all / Sq_r2^3;
    Ly(2,3) = (Dy_all*((3*Dy_all*Dy3_yPlus1 - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r2 - 1) ...
             + ((3*Dy_all*Dy3_yPlus1 - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r3 - 1)*Dy3_yPlus1) ...
             / (Sq_r2^1.5 * Sq_r3^1.5);
    Ly(2,4) = -(Dy_all*((3*Dx_all*x_Dx2Plus1 + 3*Dy_all*y_Dy4Plus1 - 3*Dz_all*Dz_all)/Sq_r2 + 1) ...
              - ((3*Dx_all*x_Dx2Plus1 + 3*Dy_all*y_Dy4Plus1 - 3*Dz_all*Dz_all)/Sq_r4 + 1)*y_Dy4Plus1) ...
              / (Sq_r2^1.5 * Sq_r4^1.5);
    Ly(2,5) = (Dy_all*((3*Dy_all*Dy_all - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r2 - 1) ...
             + Dy_all*((3*Dy_all*Dy_all - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r2^1.5 * Sq_r5^1.5);
    Ly(2,6) = -(Dy_all*((3*Dx_all*x_Dx2Plus1 - 3*Dy_all*Dy_all + 3*Dz_all*z_Dz6Plus1)/Sq_r2 + 1) ...
              + Dy_all*((3*Dx_all*x_Dx2Plus1 - 3*Dy_all*Dy_all + 3*Dz_all*z_Dz6Plus1)/Sq_r6 + 1)) ...
              / (Sq_r2^1.5 * Sq_r6^1.5);

    % Row 3
    Ly(3,1) = Ly(1,3);
    Ly(3,2) = Ly(2,3);
    Ly(3,3) = 4*Dy3_yPlus1 / Sq_r3^3;
    Ly(3,4) = (((3*Dx_all*Dx_all - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r3 - 1)*Dy3_yPlus1 ...
             - ((3*Dx_all*Dx_all - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r4 - 1)*y_Dy4Plus1) ...
             / (Sq_r3^1.5 * Sq_r4^1.5);
    Ly(3,5) = (((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r3 - 1)*Dy3_yPlus1 ...
             + Dy_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r3^1.5 * Sq_r5^1.5);
    Ly(3,6) = (((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 - 3*Dz_all*z_Dz6Plus1)/Sq_r3 - 1)*Dy3_yPlus1 ...
             + Dy_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 - 3*Dz_all*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r3^1.5 * Sq_r6^1.5);

    % Row 4
    Ly(4,1) = Ly(1,4);
    Ly(4,2) = Ly(2,4);
    Ly(4,3) = Ly(3,4);
    Ly(4,4) = -4*y_Dy4Plus1 / Sq_r4^3;
    Ly(4,5) = -(((3*Dx_all*Dx_all - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r4 - 1)*y_Dy4Plus1 ...
              - Dy_all*((3*Dx_all*Dx_all - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)) ...
              / (Sq_r4^1.5 * Sq_r5^1.5);
    Ly(4,6) = (((3*Dy_all*y_Dy4Plus1 - 3*Dx_all*Dx_all + 3*Dz_all*z_Dz6Plus1)/Sq_r4 + 1)*y_Dy4Plus1 ...
             - Dy_all*((3*Dy_all*y_Dy4Plus1 - 3*Dx_all*Dx_all + 3*Dz_all*z_Dz6Plus1)/Sq_r6 + 1)) ...
             / (Sq_r4^1.5 * Sq_r6^1.5);

    % Row 5
    Ly(5,1) = Ly(1,5);
    Ly(5,2) = Ly(2,5);
    Ly(5,3) = Ly(3,5);
    Ly(5,4) = Ly(4,5);
    Ly(5,5) = 4*Dy_all / Sq_r5^3;
    Ly(5,6) = (Dy_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy_all - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r5 - 1) ...
             + Dy_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy_all - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r5^1.5 * Sq_r6^1.5);

    % Row 6
    Ly(6,1) = Ly(1,6);
    Ly(6,2) = Ly(2,6);
    Ly(6,3) = Ly(3,6);
    Ly(6,4) = Ly(4,6);
    Ly(6,5) = Ly(5,6);
    Ly(6,6) = 4*Dy_all / Sq_r6^3;

    % ========================================
    % Lz MATRIX
    % ========================================
    Lz = zeros(6, 6);

    % Row 1
    Lz(1,1) = 4*Dz_all / Sq_r1^3;
    Lz(1,2) = (Dz_all*((3*Dy_all*Dy_all - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r1 - 1) ...
             + Dz_all*((3*Dy_all*Dy_all - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r2 - 1)) ...
             / (Sq_r1^1.5 * Sq_r2^1.5);
    Lz(1,3) = (Dz_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz_all)/Sq_r1 - 1) ...
             + Dz_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz_all)/Sq_r3 - 1)) ...
             / (Sq_r1^1.5 * Sq_r3^1.5);
    Lz(1,4) = (Dz_all*((3*Dx_all*Dx1_xPlus1 - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r1 - 1) ...
             + Dz_all*((3*Dx_all*Dx1_xPlus1 - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r4 - 1)) ...
             / (Sq_r1^1.5 * Sq_r4^1.5);
    Lz(1,5) = (Dz_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all + 3*Dz_all*Dz5_zPlus1)/Sq_r1 - 1) ...
             + ((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)*Dz5_zPlus1) ...
             / (Sq_r1^1.5 * Sq_r5^1.5);
    Lz(1,6) = (Dz_all*((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all - 3*Dz_all*z_Dz6Plus1)/Sq_r1 - 1) ...
             - ((3*Dx_all*Dx1_xPlus1 + 3*Dy_all*Dy_all - 3*Dz_all*z_Dz6Plus1)/Sq_r6 - 1)*z_Dz6Plus1) ...
             / (Sq_r1^1.5 * Sq_r6^1.5);

    % Row 2
    Lz(2,1) = Lz(1,2);
    Lz(2,2) = 4*Dz_all / Sq_r2^3;
    Lz(2,3) = (Dz_all*((3*Dy_all*Dy3_yPlus1 - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r2 - 1) ...
             + Dz_all*((3*Dy_all*Dy3_yPlus1 - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz_all)/Sq_r3 - 1)) ...
             / (Sq_r2^1.5 * Sq_r3^1.5);
    Lz(2,4) = -(Dz_all*((3*Dx_all*x_Dx2Plus1 + 3*Dy_all*y_Dy4Plus1 - 3*Dz_all*Dz_all)/Sq_r2 + 1) ...
              + Dz_all*((3*Dx_all*x_Dx2Plus1 + 3*Dy_all*y_Dy4Plus1 - 3*Dz_all*Dz_all)/Sq_r4 + 1)) ...
              / (Sq_r2^1.5 * Sq_r4^1.5);
    Lz(2,5) = (Dz_all*((3*Dy_all*Dy_all - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r2 - 1) ...
             + ((3*Dy_all*Dy_all - 3*Dx_all*x_Dx2Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)*Dz5_zPlus1) ...
             / (Sq_r2^1.5 * Sq_r5^1.5);
    Lz(2,6) = -(Dz_all*((3*Dx_all*x_Dx2Plus1 - 3*Dy_all*Dy_all + 3*Dz_all*z_Dz6Plus1)/Sq_r2 + 1) ...
              - ((3*Dx_all*x_Dx2Plus1 - 3*Dy_all*Dy_all + 3*Dz_all*z_Dz6Plus1)/Sq_r6 + 1)*z_Dz6Plus1) ...
              / (Sq_r2^1.5 * Sq_r6^1.5);

    % Row 3
    Lz(3,1) = Lz(1,3);
    Lz(3,2) = Lz(2,3);
    Lz(3,3) = 4*Dz_all / Sq_r3^3;
    Lz(3,4) = (Dz_all*((3*Dx_all*Dx_all - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r3 - 1) ...
             + Dz_all*((3*Dx_all*Dx_all - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz_all*Dz_all)/Sq_r4 - 1)) ...
             / (Sq_r3^1.5 * Sq_r4^1.5);
    Lz(3,5) = (Dz_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r3 - 1) ...
             + ((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)*Dz5_zPlus1) ...
             / (Sq_r3^1.5 * Sq_r5^1.5);
    Lz(3,6) = (Dz_all*((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 - 3*Dz_all*z_Dz6Plus1)/Sq_r3 - 1) ...
             - ((3*Dx_all*Dx_all + 3*Dy_all*Dy3_yPlus1 - 3*Dz_all*z_Dz6Plus1)/Sq_r6 - 1)*z_Dz6Plus1) ...
             / (Sq_r3^1.5 * Sq_r6^1.5);

    % Row 4
    Lz(4,1) = Lz(1,4);
    Lz(4,2) = Lz(2,4);
    Lz(4,3) = Lz(3,4);
    Lz(4,4) = 4*Dz_all / Sq_r4^3;
    Lz(4,5) = (Dz_all*((3*Dx_all*Dx_all - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r4 - 1) ...
             + ((3*Dx_all*Dx_all - 3*Dy_all*y_Dy4Plus1 + 3*Dz_all*Dz5_zPlus1)/Sq_r5 - 1)*Dz5_zPlus1) ...
             / (Sq_r4^1.5 * Sq_r5^1.5);
    Lz(4,6) = -(Dz_all*((3*Dy_all*y_Dy4Plus1 - 3*Dx_all*Dx_all + 3*Dz_all*z_Dz6Plus1)/Sq_r4 + 1) ...
              - ((3*Dy_all*y_Dy4Plus1 - 3*Dx_all*Dx_all + 3*Dz_all*z_Dz6Plus1)/Sq_r6 + 1)*z_Dz6Plus1) ...
              / (Sq_r4^1.5 * Sq_r6^1.5);

    % Row 5
    Lz(5,1) = Lz(1,5);
    Lz(5,2) = Lz(2,5);
    Lz(5,3) = Lz(3,5);
    Lz(5,4) = Lz(4,5);
    Lz(5,5) = 4*Dz5_zPlus1 / Sq_r5^3;
    Lz(5,6) = (((3*Dx_all*Dx_all + 3*Dy_all*Dy_all - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r5 - 1)*Dz5_zPlus1 ...
             - ((3*Dx_all*Dx_all + 3*Dy_all*Dy_all - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r6 - 1)*z_Dz6Plus1) ...
             / (Sq_r5^1.5 * Sq_r6^1.5);

    % Row 6
    Lz(6,1) = Lz(1,6);
    Lz(6,2) = Lz(2,6);
    Lz(6,3) = Lz(3,6);
    Lz(6,4) = Lz(4,6);
    Lz(6,5) = Lz(5,6);
    Lz(6,6) = -4*z_Dz6Plus1 / Sq_r6^3;

end
