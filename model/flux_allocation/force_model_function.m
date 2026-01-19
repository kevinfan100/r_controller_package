function f_m = force_model_function(v_m, pos_m_ext, params)
% FORCE_MODEL_FUNCTION Hall voltage -> Estimated force (Simulink version)
%
% Simulink-compatible force model. Supports both static pos_m (from params)
% and dynamic pos_m (from external) for closed-loop motion control.
%
% Formula: F = g_H * Phi_hat' * L * Phi_hat
%   where Phi_hat = D_H_hat * V_m (normalized flux)
%   and L = position-dependent gradient matrices
%
% Inputs:
%   v_m        - Measured Hall sensor voltage (6x1) [V]
%   pos_m_ext  - External bead position (3x1) [um], used when pos_m_source=1
%   params     - Parameters from force_model_allocation_params('Simulink', true)
%                Required fields:
%                .pos_m        - Bead position (3x1) [um] (used when pos_m_source=0)
%                .pos_m_source - 0=static (from params), 1=dynamic (from pos_m_ext)
%                .D_H_hat      - Normalized Hall gain matrix (6x6)
%                .T_m2a        - Measuring to Actuator transform (3x3)
%                .T_a2m        - Actuator to Measuring transform (3x3)
%                .g_H          - Force gain [pN/V^2]
%                .R_norm       - Normalization radius [um]
%
% Output:
%   f_m    - Estimated force (3x1), Measuring coordinate [pN]
%
% Dynamic pos_m Mode (pos_m_source=1):
%   When integrated with Particle Dynamics, pos_m_ext comes from the
%   particle position, enabling position-dependent force estimation
%   in closed-loop motion control.
%
% See also: force_model, force_model_allocation_params, CLAUDE.md

%#codegen

    %% Extract position (select source based on pos_m_source)
    % 0=static (from params), 1=dynamic (from external)
    if params.pos_m_source > 0.5
        pos_m = pos_m_ext;
    else
        pos_m = params.pos_m;
    end

    %% Step 1: V_m -> Phi_hat (normalized flux)
    Phi_hat = params.D_H_hat * v_m;

    %% Step 2: Position Transform to Actuator Coordinate
    pos_a = params.T_m2a * pos_m;

    %% Step 3: Calculate L Matrices
    [Lx, Ly, Lz] = calc_L_matrices(pos_a, params.R_norm);

    %% Step 4: Quadratic Form (Force calculation in Actuator Coordinate)
    Fx_a = params.g_H * (Phi_hat' * Lx * Phi_hat);
    Fy_a = params.g_H * (Phi_hat' * Ly * Phi_hat);
    Fz_a = params.g_H * (Phi_hat' * Lz * Phi_hat);

    %% Step 5: Transform back to Measuring Coordinate
    f_m = params.T_a2m * [Fx_a; Fy_a; Fz_a];

end


% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function [Lx, Ly, Lz] = calc_L_matrices(pos_um, R_norm)
% CALC_L_MATRICES Position-dependent gradient matrices
%
% Ported from force_model.m (originally from CALCULATION.cpp)
%
% Inputs:
%   pos_um - Bead position [x, y, z] in um (Actuator Coordinate)
%   R_norm - Normalization radius (550 um)
%
% Outputs:
%   Lx, Ly, Lz - 6x6 symmetric gradient matrices

    %% Normalize Position
    x = pos_um(1) / R_norm;
    y = pos_um(2) / R_norm;
    z = pos_um(3) / R_norm;

    %% Pole Distance Terms
    % 6 poles at normalized distance 1 from origin
    Dx1_xPlus1 = 1 - x;   % Pole 1: +X
    x_Dx2Plus1 = 1 + x;   % Pole 2: -X
    Dy3_yPlus1 = 1 - y;   % Pole 3: +Y
    y_Dy4Plus1 = 1 + y;   % Pole 4: -Y
    Dz5_zPlus1 = 1 - z;   % Pole 5: +Z
    z_Dz6Plus1 = 1 + z;   % Pole 6: -Z

    % Difference terms
    Dx_all = -x;
    Dy_all = -y;
    Dz_all = -z;

    %% Squared Distances
    Sq_r1 = Dx1_xPlus1^2 + y^2 + z^2;
    Sq_r2 = x_Dx2Plus1^2 + y^2 + z^2;
    Sq_r3 = x^2 + Dy3_yPlus1^2 + z^2;
    Sq_r4 = x^2 + y_Dy4Plus1^2 + z^2;
    Sq_r5 = x^2 + y^2 + Dz5_zPlus1^2;
    Sq_r6 = x^2 + y^2 + z_Dz6Plus1^2;

    %% Lx Matrix
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

    %% Ly Matrix
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

    %% Lz Matrix
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
