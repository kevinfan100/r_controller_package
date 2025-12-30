function params = system_params()
% SYSTEM_PARAMS Initialize system parameters for Phase 2 models
%
% This function centralizes all parameters needed by:
%   - inverse_model.m (Green block: f_d -> v_d)
%   - force_model.m (Yellow block: v_m -> f_m)
%
% Output:
%   params - Structure containing all system parameters
%
% References:
%   - CALCULATION.cpp (MotionControl_KalmanFilter_Simu)
%   - InverseModel_Math_Derivation.pdf

    %% ════════════════════════════════════════════════════════════════════
    %                           Basic Constants
    %% ════════════════════════════════════════════════════════════════════
    params.R_norm = 550.0;           % [um] Normalization radius for LUT
    params.force_scale = 10.0;       % Force scaling factor

    %% ════════════════════════════════════════════════════════════════════
    %                   D_H and D_H^-1 Matrices
    %% ════════════════════════════════════════════════════════════════════
    % From InverseModel_Math_Derivation.pdf Section 1.7.3
    % D_H: Hall sensor gain matrix (V_m = D_H^-1 * Phi)
    % Note: D_H^-1 is NOT normalized by Phi_max

    params.D_H_inv = diag([2.252, 2.749, 2.246, 2.294, 2.832, 2.295]);
    params.D_H = diag([1/2.252, 1/2.749, 1/2.246, 1/2.294, 1/2.832, 1/2.295]);

    %% ════════════════════════════════════════════════════════════════════
    %                   D̂_H Normalized Matrices
    %% ════════════════════════════════════════════════════════════════════
    % Normalize D_H by first element: D̂_H = D_H / d_H1
    % This makes D̂_H(1,1) = 1
    d_H1 = params.D_H(1,1);                     % = 1/2.252 ≈ 0.4441
    params.D_H_hat = params.D_H / d_H1;         % First element = 1
    params.D_H_hat_inv = params.D_H_inv * d_H1; % First element = 1

    %% ════════════════════════════════════════════════════════════════════
    %                   Force Gain g_H (Hall sensor-based)
    %% ════════════════════════════════════════════════════════════════════
    % g_H = 4.741 * d_H1^2 (experimental calibration with normalization)
    % 4.741 is the raw calibration value, multiplied by d_H1² to absorb
    % the normalization factor from D̂_H
    params.g_H_raw = 4.741;                     % [pN/V²] Raw calibration value
    params.g_H = params.g_H_raw * d_H1^2;       % ≈ 0.935 pN/V²

    %% ════════════════════════════════════════════════════════════════════
    %                   K_I Matrix (Theoretical)
    %% ════════════════════════════════════════════════════════════════════
    % K_I: Flux allocation matrix (Phi = K_I * I)
    % From C++ CALCULATION.cpp lines 24-29:
    %   Diagonal = 5/6, Off-diagonal = -1/6
    %   Row sum = 0 (important property)
    %
    % Correct formula: K_I = I - (1/6)*J where J is all-ones matrix
    % NOT (5/6)*I - (1/6)*J which gives wrong diagonal = 2/3

    params.KI_theo = eye(6) - (1/6)*ones(6,6);

    % pinv(K_I): Pseudo-inverse for recovering current from flux
    % Note: Not used in Force Model (K_I cancels out in Phi^T*L*Phi formula)
    % Kept for verification/debugging purposes
    params.KI_pinv = pinv(params.KI_theo);

    %% ════════════════════════════════════════════════════════════════════
    %                   Coordinate Transform Matrices
    %% ════════════════════════════════════════════════════════════════════
    % T_m2a: Measuring to Actuator coordinate transform
    % From CALCULATION.cpp lines 88-91

    params.T_m2a = [
        -1/sqrt(6),    1/sqrt(2),   -1/sqrt(3);
        -1/sqrt(6),   -1/sqrt(2),   -1/sqrt(3);
        -sqrt(2/3),    0,            1/sqrt(3)
    ];

    % T_a2m: Actuator to Measuring (inverse = transpose for orthogonal matrix)
    params.T_a2m = params.T_m2a';

    %% ════════════════════════════════════════════════════════════════════
    %                   Pole Positions (Actuator Coordinate)
    %% ════════════════════════════════════════════════════════════════════
    % Orthogonal 6-pole configuration: opposing poles along each axis
    % From CALCULATION.cpp lines 1739-1779 (when Bias = 0)

    params.pole_positions = [
        +550,    0,    0;   % Pole 1: +X direction
        -550,    0,    0;   % Pole 2: -X direction
           0, +550,    0;   % Pole 3: +Y direction
           0, -550,    0;   % Pole 4: -Y direction
           0,    0, +550;   % Pole 5: +Z direction
           0,    0, -550    % Pole 6: -Z direction
    ];  % 6x3 matrix, units: um

    %% ════════════════════════════════════════════════════════════════════
    %                   Octant Current Remap Table
    %% ════════════════════════════════════════════════════════════════════
    % Hard-coded octant current remap table
    % Sign indicates current direction (positive/negative)
    % From InverseModel_Math_Derivation.pdf

    params.octant_remap = [
        +1, +2, +3, +4, +5, +6;   % Octant 0 (+++): Fx+, Fy+, Fz+
        -4, -5, +3, +1, +2, +6;   % Octant 1 (-++): Fx-, Fy+, Fz+
        +2, +1, -6, +5, +4, -3;   % Octant 2 (+-+): Fx+, Fy-, Fz+
        -5, -4, -6, -2, -1, -3;   % Octant 3 (--+): Fx-, Fy-, Fz+
        -1, -2, +6, -4, -5, +3;   % Octant 4 (++-): Fx+, Fy+, Fz-
        +4, +5, +6, -1, -2, +3;   % Octant 5 (-+-): Fx-, Fy+, Fz-
        -2, -1, -3, -5, -4, -6;   % Octant 6 (+--): Fx+, Fy-, Fz-
        +5, +4, -3, +2, +1, -6    % Octant 7 (---): Fx-, Fy-, Fz-
    ];

    %% ════════════════════════════════════════════════════════════════════
    %                           LUT Path
    %% ════════════════════════════════════════════════════════════════════
    params.lut_path = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'lut');

    %% ════════════════════════════════════════════════════════════════════
    %                   Pre-compute Combined Matrices
    %% ════════════════════════════════════════════════════════════════════
    % For Inverse Model Stage 7-8: v_d = D̂_H^-1 * K_I * I_d
    params.DH_hat_inv_KI = params.D_H_hat_inv * params.KI_theo;

end
