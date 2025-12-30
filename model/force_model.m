function f_m = force_model(v_m, pos_m, params)
% FORCE_MODEL Yellow Block: Hall sensor voltage -> Estimated force
%
% Implements the Hall sensor-based Force Model for magnetic tweezers.
%
% Formula: F = g_H * Φ̂^T * L * Φ̂
% where:
%   Φ̂ = D̂_H * V_m (normalized magnetic flux)
%   L = position-dependent gradient matrices (Lx, Ly, Lz)
%   g_H = 4.741 * d_H1^2 ≈ 0.935 pN/V² (experimental calibration)
%
% Inputs:
%   v_m    - Hall sensor voltage (6x1) [V]
%   pos_m  - Bead position [x; y; z] (3x1), Measuring coordinate [um]
%   params - System parameters from system_params()
%
% Output:
%   f_m    - Estimated force [Fx; Fy; Fz] (3x1), Measuring coordinate [pN]
%
% References:
%   - CALCULATION.cpp lines 1963-1970 (Calc_FNor_FromSixCurr_KItheo)
%   - CALCULATION.cpp lines 1712-1919 (L matrix calculation)

    %% ════════════════════════════════════════════════════════════════════
    %                   Step 1: V_m -> Φ̂ (Normalized Flux)
    %% ════════════════════════════════════════════════════════════════════
    % Φ̂ = D̂_H * V_m (using normalized D̂_H)
    Phi = params.D_H_hat * v_m;

    %% ════════════════════════════════════════════════════════════════════
    %                   Step 2: Coordinate Transform (Position)
    %% ════════════════════════════════════════════════════════════════════
    % Transform position to Actuator coordinate
    pos_a = params.T_m2a * pos_m;

    %% ════════════════════════════════════════════════════════════════════
    %                   Step 3: Calculate L Matrices (Full Implementation)
    %% ════════════════════════════════════════════════════════════════════
    % Position-dependent gradient matrices from magnetic dipole interaction
    % Ported from CALCULATION.cpp Calc_Pre_L(), Calc_Lx(), Calc_Ly(), Calc_Lz()
    [Lx, Ly, Lz] = calc_L_matrices_full(pos_a, params.R_norm);

    %% ════════════════════════════════════════════════════════════════════
    %                   Step 4: Φ̂ -> F (Quadratic Form)
    %% ════════════════════════════════════════════════════════════════════
    % F = g_H * Φ̂' * L * Φ̂
    % Note: K_I cancels out in the derivation
    Fx_a = params.g_H * (Phi' * Lx * Phi);
    Fy_a = params.g_H * (Phi' * Ly * Phi);
    Fz_a = params.g_H * (Phi' * Lz * Phi);
    f_a = [Fx_a; Fy_a; Fz_a];

    %% ════════════════════════════════════════════════════════════════════
    %                   Step 5: Actuator -> Measuring
    %% ════════════════════════════════════════════════════════════════════
    f_m = params.T_a2m * f_a;

end


%% ════════════════════════════════════════════════════════════════════════
%           Full L Matrix Calculation (Ported from CALCULATION.cpp)
%% ════════════════════════════════════════════════════════════════════════
function [Lx, Ly, Lz] = calc_L_matrices_full(pos_um, R_norm)
% CALC_L_MATRICES_FULL Calculate position-dependent L matrices
%
% Complete port of CALCULATION.cpp:
%   - Calc_Pre_L() lines 1712-1780
%   - Calc_Lx() lines 1783-1826
%   - Calc_Ly() lines 1830-1873
%   - Calc_Lz() lines 1877-1919
%
% Inputs:
%   pos_um - Bead position [x, y, z] in um (Actuator Coordinate)
%   R_norm - Normalization radius (550 um)
%
% Outputs:
%   Lx, Ly, Lz - 6x6 symmetric gradient matrices

    %% ════════════════════════════════════════════════════════════════════
    %                   Calc_Pre_L (lines 1712-1780)
    %% ════════════════════════════════════════════════════════════════════

    % Normalize position
    x = pos_um(1) / R_norm;
    y = pos_um(2) / R_norm;
    z = pos_um(3) / R_norm;

    % Pole positions via special adjustment terms (+1 encoding)
    % In C++, Bias_umOa_P1toP6[i] = 0.0 for all i (CALCULATION.cpp line 18)
    % The pole positions are encoded in the "+1" terms:
    %   Dx1_xPlus1 = -x + 1  -> Pole 1 at +X (normalized distance 1)
    %   x_Dx2Plus1 = x + 1   -> Pole 2 at -X (normalized distance 1)
    %   etc.

    % With Bias = 0, all difference vectors are simply -position
    % Dx1_x to Dx6_x (lines 1746-1751 in C++)
    Dx1_x = -x;   % Bias[0]/R_norm - x = 0 - x = -x
    Dx2_x = -x;   % Bias[3]/R_norm - x = 0 - x = -x
    Dx3_x = -x;   % Bias[6]/R_norm - x = 0 - x = -x
    Dx4_x = -x;   % Bias[9]/R_norm - x = 0 - x = -x
    Dx5_x = -x;   % Bias[12]/R_norm - x = 0 - x = -x
    Dx6_x = -x;   % Bias[15]/R_norm - x = 0 - x = -x

    % Dy1_y to Dy6_y (lines 1753-1758)
    Dy1_y = -y;   % = 0 - y = -y
    Dy2_y = -y;   % = 0 - y = -y
    Dy3_y = -y;   % = 0 - y = -y
    Dy4_y = -y;   % = 0 - y = -y
    Dy5_y = -y;   % = 0 - y = -y
    Dy6_y = -y;   % = 0 - y = -y

    % Dz1_z to Dz6_z (lines 1760-1765)
    Dz1_z = -z;   % = 0 - z = -z
    Dz2_z = -z;   % = 0 - z = -z
    Dz3_z = -z;   % = 0 - z = -z
    Dz4_z = -z;   % = 0 - z = -z
    Dz5_z = -z;   % = 0 - z = -z
    Dz6_z = -z;   % = 0 - z = -z

    % Special adjustment terms for opposing poles (lines 1767-1772)
    % These encode the pole positions with the +1 offset
    Dx1_xPlus1 = -x + 1;   % Pole 1 at +X: 0 - x + 1 = 1 - x
    x_Dx2Plus1 = x + 1;    % Pole 2 at -X: x - 0 + 1 = x + 1
    Dy3_yPlus1 = -y + 1;   % Pole 3 at +Y: 0 - y + 1 = 1 - y
    y_Dy4Plus1 = y + 1;    % Pole 4 at -Y: y - 0 + 1 = y + 1
    Dz5_zPlus1 = -z + 1;   % Pole 5 at +Z: 0 - z + 1 = 1 - z
    z_Dz6Plus1 = z + 1;    % Pole 6 at -Z: z - 0 + 1 = z + 1

    % Squared distances (lines 1774-1779)
    Sq_r1 = Dx1_xPlus1^2 + Dy1_y^2 + Dz1_z^2;
    Sq_r2 = x_Dx2Plus1^2 + Dy2_y^2 + Dz2_z^2;
    Sq_r3 = Dx3_x^2 + Dy3_yPlus1^2 + Dz3_z^2;
    Sq_r4 = Dx4_x^2 + y_Dy4Plus1^2 + Dz4_z^2;
    Sq_r5 = Dx5_x^2 + Dy5_y^2 + Dz5_zPlus1^2;
    Sq_r6 = Dx6_x^2 + Dy6_y^2 + z_Dz6Plus1^2;

    %% ════════════════════════════════════════════════════════════════════
    %                   Calc_Lx (lines 1783-1826)
    %% ════════════════════════════════════════════════════════════════════
    Lx = zeros(6, 6);

    % Row 0 (MATLAB index 1)
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

    % Row 1 (MATLAB index 2)
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

    % Row 2 (MATLAB index 3)
    Lx(3,1) = Lx(1,3);
    Lx(3,2) = Lx(2,3);
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

    % Row 3 (MATLAB index 4)
    Lx(4,1) = Lx(1,4);
    Lx(4,2) = Lx(2,4);
    Lx(4,3) = Lx(3,4);
    Lx(4,4) = 4*Dx4_x / Sq_r4^3;
    Lx(4,5) = (Dx4_x*((3*Dx4_x*Dx5_x - 3*Dy5_y*y_Dy4Plus1 + 3*Dz4_z*Dz5_zPlus1)/Sq_r4 - 1) ...
             + Dx5_x*((3*Dx4_x*Dx5_x - 3*Dy5_y*y_Dy4Plus1 + 3*Dz4_z*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r4^1.5 * Sq_r5^1.5);
    Lx(4,6) = -(Dx4_x*((3*Dy6_y*y_Dy4Plus1 - 3*Dx4_x*Dx6_x + 3*Dz4_z*z_Dz6Plus1)/Sq_r4 + 1) ...
              + Dx6_x*((3*Dy6_y*y_Dy4Plus1 - 3*Dx4_x*Dx6_x + 3*Dz4_z*z_Dz6Plus1)/Sq_r6 + 1)) ...
              / (Sq_r4^1.5 * Sq_r6^1.5);

    % Row 4 (MATLAB index 5)
    Lx(5,1) = Lx(1,5);
    Lx(5,2) = Lx(2,5);
    Lx(5,3) = Lx(3,5);
    Lx(5,4) = Lx(4,5);
    Lx(5,5) = 4*Dx5_x / Sq_r5^3;
    Lx(5,6) = (Dx5_x*((3*Dx5_x*Dx6_x + 3*Dy5_y*Dy6_y - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r5 - 1) ...
             + Dx6_x*((3*Dx5_x*Dx6_x + 3*Dy5_y*Dy6_y - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r5^1.5 * Sq_r6^1.5);

    % Row 5 (MATLAB index 6)
    Lx(6,1) = Lx(1,6);
    Lx(6,2) = Lx(2,6);
    Lx(6,3) = Lx(3,6);
    Lx(6,4) = Lx(4,6);
    Lx(6,5) = Lx(5,6);
    Lx(6,6) = 4*Dx6_x / Sq_r6^3;

    %% ════════════════════════════════════════════════════════════════════
    %                   Calc_Ly (lines 1830-1873)
    %% ════════════════════════════════════════════════════════════════════
    Ly = zeros(6, 6);

    % Row 0 (MATLAB index 1)
    Ly(1,1) = 4*Dy1_y / Sq_r1^3;
    Ly(1,2) = (Dy1_y*((3*Dy1_y*Dy2_y - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz1_z*Dz2_z)/Sq_r1 - 1) ...
             + Dy2_y*((3*Dy1_y*Dy2_y - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz1_z*Dz2_z)/Sq_r2 - 1)) ...
             / (Sq_r1^1.5 * Sq_r2^1.5);
    Ly(1,3) = (Dy1_y*((3*Dx3_x*Dx1_xPlus1 + 3*Dy1_y*Dy3_yPlus1 + 3*Dz1_z*Dz3_z)/Sq_r1 - 1) ...
             + ((3*Dx3_x*Dx1_xPlus1 + 3*Dy1_y*Dy3_yPlus1 + 3*Dz1_z*Dz3_z)/Sq_r3 - 1)*Dy3_yPlus1) ...
             / (Sq_r1^1.5 * Sq_r3^1.5);
    Ly(1,4) = (Dy1_y*((3*Dx4_x*Dx1_xPlus1 - 3*Dy1_y*y_Dy4Plus1 + 3*Dz1_z*Dz4_z)/Sq_r1 - 1) ...
             - ((3*Dx4_x*Dx1_xPlus1 - 3*Dy1_y*y_Dy4Plus1 + 3*Dz1_z*Dz4_z)/Sq_r4 - 1)*y_Dy4Plus1) ...
             / (Sq_r1^1.5 * Sq_r4^1.5);
    Ly(1,5) = (Dy1_y*((3*Dx5_x*Dx1_xPlus1 + 3*Dy1_y*Dy5_y + 3*Dz1_z*Dz5_zPlus1)/Sq_r1 - 1) ...
             + Dy5_y*((3*Dx5_x*Dx1_xPlus1 + 3*Dy1_y*Dy5_y + 3*Dz1_z*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r1^1.5 * Sq_r5^1.5);
    Ly(1,6) = (Dy1_y*((3*Dx6_x*Dx1_xPlus1 + 3*Dy1_y*Dy6_y - 3*Dz1_z*z_Dz6Plus1)/Sq_r1 - 1) ...
             + Dy6_y*((3*Dx6_x*Dx1_xPlus1 + 3*Dy1_y*Dy6_y - 3*Dz1_z*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r1^1.5 * Sq_r6^1.5);

    % Row 1 (MATLAB index 2)
    Ly(2,1) = Ly(1,2);
    Ly(2,2) = 4*Dy2_y / Sq_r2^3;
    Ly(2,3) = (Dy2_y*((3*Dy2_y*Dy3_yPlus1 - 3*Dx3_x*x_Dx2Plus1 + 3*Dz2_z*Dz3_z)/Sq_r2 - 1) ...
             + ((3*Dy2_y*Dy3_yPlus1 - 3*Dx3_x*x_Dx2Plus1 + 3*Dz2_z*Dz3_z)/Sq_r3 - 1)*Dy3_yPlus1) ...
             / (Sq_r2^1.5 * Sq_r3^1.5);
    Ly(2,4) = -(Dy2_y*((3*Dx4_x*x_Dx2Plus1 + 3*Dy2_y*y_Dy4Plus1 - 3*Dz2_z*Dz4_z)/Sq_r2 + 1) ...
              - ((3*Dx4_x*x_Dx2Plus1 + 3*Dy2_y*y_Dy4Plus1 - 3*Dz2_z*Dz4_z)/Sq_r4 + 1)*y_Dy4Plus1) ...
              / (Sq_r2^1.5 * Sq_r4^1.5);
    Ly(2,5) = (Dy2_y*((3*Dy2_y*Dy5_y - 3*Dx5_x*x_Dx2Plus1 + 3*Dz2_z*Dz5_zPlus1)/Sq_r2 - 1) ...
             + Dy5_y*((3*Dy2_y*Dy5_y - 3*Dx5_x*x_Dx2Plus1 + 3*Dz2_z*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r2^1.5 * Sq_r5^1.5);
    Ly(2,6) = -(Dy2_y*((3*Dx6_x*x_Dx2Plus1 - 3*Dy2_y*Dy6_y + 3*Dz2_z*z_Dz6Plus1)/Sq_r2 + 1) ...
              + Dy6_y*((3*Dx6_x*x_Dx2Plus1 - 3*Dy2_y*Dy6_y + 3*Dz2_z*z_Dz6Plus1)/Sq_r6 + 1)) ...
              / (Sq_r2^1.5 * Sq_r6^1.5);

    % Row 2 (MATLAB index 3)
    Ly(3,1) = Ly(1,3);
    Ly(3,2) = Ly(2,3);
    Ly(3,3) = 4*Dy3_yPlus1 / Sq_r3^3;
    Ly(3,4) = (((3*Dx3_x*Dx4_x - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz3_z*Dz4_z)/Sq_r3 - 1)*Dy3_yPlus1 ...
             - ((3*Dx3_x*Dx4_x - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz3_z*Dz4_z)/Sq_r4 - 1)*y_Dy4Plus1) ...
             / (Sq_r3^1.5 * Sq_r4^1.5);
    Ly(3,5) = (((3*Dx3_x*Dx5_x + 3*Dy5_y*Dy3_yPlus1 + 3*Dz3_z*Dz5_zPlus1)/Sq_r3 - 1)*Dy3_yPlus1 ...
             + Dy5_y*((3*Dx3_x*Dx5_x + 3*Dy5_y*Dy3_yPlus1 + 3*Dz3_z*Dz5_zPlus1)/Sq_r5 - 1)) ...
             / (Sq_r3^1.5 * Sq_r5^1.5);
    Ly(3,6) = (((3*Dx3_x*Dx6_x + 3*Dy6_y*Dy3_yPlus1 - 3*Dz3_z*z_Dz6Plus1)/Sq_r3 - 1)*Dy3_yPlus1 ...
             + Dy6_y*((3*Dx3_x*Dx6_x + 3*Dy6_y*Dy3_yPlus1 - 3*Dz3_z*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r3^1.5 * Sq_r6^1.5);

    % Row 3 (MATLAB index 4)
    Ly(4,1) = Ly(1,4);
    Ly(4,2) = Ly(2,4);
    Ly(4,3) = Ly(3,4);
    Ly(4,4) = -4*y_Dy4Plus1 / Sq_r4^3;
    Ly(4,5) = -(((3*Dx4_x*Dx5_x - 3*Dy5_y*y_Dy4Plus1 + 3*Dz4_z*Dz5_zPlus1)/Sq_r4 - 1)*y_Dy4Plus1 ...
              - Dy5_y*((3*Dx4_x*Dx5_x - 3*Dy5_y*y_Dy4Plus1 + 3*Dz4_z*Dz5_zPlus1)/Sq_r5 - 1)) ...
              / (Sq_r4^1.5 * Sq_r5^1.5);
    Ly(4,6) = (((3*Dy6_y*y_Dy4Plus1 - 3*Dx4_x*Dx6_x + 3*Dz4_z*z_Dz6Plus1)/Sq_r4 + 1)*y_Dy4Plus1 ...
             - Dy6_y*((3*Dy6_y*y_Dy4Plus1 - 3*Dx4_x*Dx6_x + 3*Dz4_z*z_Dz6Plus1)/Sq_r6 + 1)) ...
             / (Sq_r4^1.5 * Sq_r6^1.5);

    % Row 4 (MATLAB index 5)
    Ly(5,1) = Ly(1,5);
    Ly(5,2) = Ly(2,5);
    Ly(5,3) = Ly(3,5);
    Ly(5,4) = Ly(4,5);
    Ly(5,5) = 4*Dy5_y / Sq_r5^3;
    Ly(5,6) = (Dy5_y*((3*Dx5_x*Dx6_x + 3*Dy5_y*Dy6_y - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r5 - 1) ...
             + Dy6_y*((3*Dx5_x*Dx6_x + 3*Dy5_y*Dy6_y - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r6 - 1)) ...
             / (Sq_r5^1.5 * Sq_r6^1.5);

    % Row 5 (MATLAB index 6)
    Ly(6,1) = Ly(1,6);
    Ly(6,2) = Ly(2,6);
    Ly(6,3) = Ly(3,6);
    Ly(6,4) = Ly(4,6);
    Ly(6,5) = Ly(5,6);
    Ly(6,6) = 4*Dy6_y / Sq_r6^3;

    %% ════════════════════════════════════════════════════════════════════
    %                   Calc_Lz (lines 1877-1919)
    %% ════════════════════════════════════════════════════════════════════
    Lz = zeros(6, 6);

    % Row 0 (MATLAB index 1)
    Lz(1,1) = 4*Dz1_z / Sq_r1^3;
    Lz(1,2) = (Dz1_z*((3*Dy1_y*Dy2_y - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz1_z*Dz2_z)/Sq_r1 - 1) ...
             + Dz2_z*((3*Dy1_y*Dy2_y - 3*Dx1_xPlus1*x_Dx2Plus1 + 3*Dz1_z*Dz2_z)/Sq_r2 - 1)) ...
             / (Sq_r1^1.5 * Sq_r2^1.5);
    Lz(1,3) = (Dz1_z*((3*Dx3_x*Dx1_xPlus1 + 3*Dy1_y*Dy3_yPlus1 + 3*Dz1_z*Dz3_z)/Sq_r1 - 1) ...
             + Dz3_z*((3*Dx3_x*Dx1_xPlus1 + 3*Dy1_y*Dy3_yPlus1 + 3*Dz1_z*Dz3_z)/Sq_r3 - 1)) ...
             / (Sq_r1^1.5 * Sq_r3^1.5);
    Lz(1,4) = (Dz1_z*((3*Dx4_x*Dx1_xPlus1 - 3*Dy1_y*y_Dy4Plus1 + 3*Dz1_z*Dz4_z)/Sq_r1 - 1) ...
             + Dz4_z*((3*Dx4_x*Dx1_xPlus1 - 3*Dy1_y*y_Dy4Plus1 + 3*Dz1_z*Dz4_z)/Sq_r4 - 1)) ...
             / (Sq_r1^1.5 * Sq_r4^1.5);
    Lz(1,5) = (Dz1_z*((3*Dx5_x*Dx1_xPlus1 + 3*Dy1_y*Dy5_y + 3*Dz1_z*Dz5_zPlus1)/Sq_r1 - 1) ...
             + ((3*Dx5_x*Dx1_xPlus1 + 3*Dy1_y*Dy5_y + 3*Dz1_z*Dz5_zPlus1)/Sq_r5 - 1)*Dz5_zPlus1) ...
             / (Sq_r1^1.5 * Sq_r5^1.5);
    Lz(1,6) = (Dz1_z*((3*Dx6_x*Dx1_xPlus1 + 3*Dy1_y*Dy6_y - 3*Dz1_z*z_Dz6Plus1)/Sq_r1 - 1) ...
             - ((3*Dx6_x*Dx1_xPlus1 + 3*Dy1_y*Dy6_y - 3*Dz1_z*z_Dz6Plus1)/Sq_r6 - 1)*z_Dz6Plus1) ...
             / (Sq_r1^1.5 * Sq_r6^1.5);

    % Row 1 (MATLAB index 2)
    Lz(2,1) = Lz(1,2);
    Lz(2,2) = 4*Dz2_z / Sq_r2^3;
    Lz(2,3) = (Dz2_z*((3*Dy2_y*Dy3_yPlus1 - 3*Dx3_x*x_Dx2Plus1 + 3*Dz2_z*Dz3_z)/Sq_r2 - 1) ...
             + Dz3_z*((3*Dy2_y*Dy3_yPlus1 - 3*Dx3_x*x_Dx2Plus1 + 3*Dz2_z*Dz3_z)/Sq_r3 - 1)) ...
             / (Sq_r2^1.5 * Sq_r3^1.5);
    Lz(2,4) = -(Dz2_z*((3*Dx4_x*x_Dx2Plus1 + 3*Dy2_y*y_Dy4Plus1 - 3*Dz2_z*Dz4_z)/Sq_r2 + 1) ...
              + Dz4_z*((3*Dx4_x*x_Dx2Plus1 + 3*Dy2_y*y_Dy4Plus1 - 3*Dz2_z*Dz4_z)/Sq_r4 + 1)) ...
              / (Sq_r2^1.5 * Sq_r4^1.5);
    Lz(2,5) = (Dz2_z*((3*Dy2_y*Dy5_y - 3*Dx5_x*x_Dx2Plus1 + 3*Dz2_z*Dz5_zPlus1)/Sq_r2 - 1) ...
             + ((3*Dy2_y*Dy5_y - 3*Dx5_x*x_Dx2Plus1 + 3*Dz2_z*Dz5_zPlus1)/Sq_r5 - 1)*Dz5_zPlus1) ...
             / (Sq_r2^1.5 * Sq_r5^1.5);
    Lz(2,6) = -(Dz2_z*((3*Dx6_x*x_Dx2Plus1 - 3*Dy2_y*Dy6_y + 3*Dz2_z*z_Dz6Plus1)/Sq_r2 + 1) ...
              - ((3*Dx6_x*x_Dx2Plus1 - 3*Dy2_y*Dy6_y + 3*Dz2_z*z_Dz6Plus1)/Sq_r6 + 1)*z_Dz6Plus1) ...
              / (Sq_r2^1.5 * Sq_r6^1.5);

    % Row 2 (MATLAB index 3)
    Lz(3,1) = Lz(1,3);
    Lz(3,2) = Lz(2,3);
    Lz(3,3) = 4*Dz3_z / Sq_r3^3;
    Lz(3,4) = (Dz3_z*((3*Dx3_x*Dx4_x - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz3_z*Dz4_z)/Sq_r3 - 1) ...
             + Dz4_z*((3*Dx3_x*Dx4_x - 3*Dy3_yPlus1*y_Dy4Plus1 + 3*Dz3_z*Dz4_z)/Sq_r4 - 1)) ...
             / (Sq_r3^1.5 * Sq_r4^1.5);
    Lz(3,5) = (Dz3_z*((3*Dx3_x*Dx5_x + 3*Dy5_y*Dy3_yPlus1 + 3*Dz3_z*Dz5_zPlus1)/Sq_r3 - 1) ...
             + ((3*Dx3_x*Dx5_x + 3*Dy5_y*Dy3_yPlus1 + 3*Dz3_z*Dz5_zPlus1)/Sq_r5 - 1)*Dz5_zPlus1) ...
             / (Sq_r3^1.5 * Sq_r5^1.5);
    Lz(3,6) = (Dz3_z*((3*Dx3_x*Dx6_x + 3*Dy6_y*Dy3_yPlus1 - 3*Dz3_z*z_Dz6Plus1)/Sq_r3 - 1) ...
             - ((3*Dx3_x*Dx6_x + 3*Dy6_y*Dy3_yPlus1 - 3*Dz3_z*z_Dz6Plus1)/Sq_r6 - 1)*z_Dz6Plus1) ...
             / (Sq_r3^1.5 * Sq_r6^1.5);

    % Row 3 (MATLAB index 4)
    Lz(4,1) = Lz(1,4);
    Lz(4,2) = Lz(2,4);
    Lz(4,3) = Lz(3,4);
    Lz(4,4) = 4*Dz4_z / Sq_r4^3;
    Lz(4,5) = (Dz4_z*((3*Dx4_x*Dx5_x - 3*Dy5_y*y_Dy4Plus1 + 3*Dz4_z*Dz5_zPlus1)/Sq_r4 - 1) ...
             + ((3*Dx4_x*Dx5_x - 3*Dy5_y*y_Dy4Plus1 + 3*Dz4_z*Dz5_zPlus1)/Sq_r5 - 1)*Dz5_zPlus1) ...
             / (Sq_r4^1.5 * Sq_r5^1.5);
    Lz(4,6) = -(Dz4_z*((3*Dy6_y*y_Dy4Plus1 - 3*Dx4_x*Dx6_x + 3*Dz4_z*z_Dz6Plus1)/Sq_r4 + 1) ...
              - ((3*Dy6_y*y_Dy4Plus1 - 3*Dx4_x*Dx6_x + 3*Dz4_z*z_Dz6Plus1)/Sq_r6 + 1)*z_Dz6Plus1) ...
              / (Sq_r4^1.5 * Sq_r6^1.5);

    % Row 4 (MATLAB index 5)
    Lz(5,1) = Lz(1,5);
    Lz(5,2) = Lz(2,5);
    Lz(5,3) = Lz(3,5);
    Lz(5,4) = Lz(4,5);
    Lz(5,5) = 4*Dz5_zPlus1 / Sq_r5^3;
    Lz(5,6) = (((3*Dx5_x*Dx6_x + 3*Dy5_y*Dy6_y - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r5 - 1)*Dz5_zPlus1 ...
             - ((3*Dx5_x*Dx6_x + 3*Dy5_y*Dy6_y - 3*Dz5_zPlus1*z_Dz6Plus1)/Sq_r6 - 1)*z_Dz6Plus1) ...
             / (Sq_r5^1.5 * Sq_r6^1.5);

    % Row 5 (MATLAB index 6)
    Lz(6,1) = Lz(1,6);
    Lz(6,2) = Lz(2,6);
    Lz(6,3) = Lz(3,6);
    Lz(6,4) = Lz(4,6);
    Lz(6,5) = Lz(5,6);
    Lz(6,6) = -4*z_Dz6Plus1 / Sq_r6^3;

end
