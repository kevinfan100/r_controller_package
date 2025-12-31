function params = system_params()
% SYSTEM_PARAMS System parameters for Phase 2 physical models
%
% Provides parameters for:
%   - inverse_model.m (f_d -> v_d)
%   - force_model.m (v_m -> f_m)
%
% Output:
%   params - Structure containing all system parameters

    % ========================================
    % BASIC CONSTANTS
    % ========================================
    params.R_norm = 550.0;           % [um] Normalization radius
    params.force_scale = 10.0;       % Force scaling factor

    % ========================================
    % HALL SENSOR MATRICES
    % ========================================
    % D_H: Hall sensor gain matrix (diagonal)
    % D_H_hat: Normalized by first element (D_H_hat(1,1) = 1)
    d_H = [1/2.252, 1/2.749, 1/2.246, 1/2.294, 1/2.832, 1/2.295];
    d_H1 = d_H(1);                            % Normalization factor

    params.D_H_hat = diag(d_H / d_H1);        % Normalized D_H
    params.D_H_hat_inv = diag(1./d_H * d_H1); % Normalized D_H^-1

    % ========================================
    % FORCE GAIN
    % ========================================
    % g_H: Experimental calibration with normalization absorbed
    % g_H = 4.741 * d_H1^2 (approx 0.935 pN/V^2)
    params.g_H = 4.741 * d_H1^2;

    % ========================================
    % FLUX ALLOCATION MATRIX
    % ========================================
    % K_I = I - (1/6)*J, where J is all-ones matrix
    % Property: row sum = 0, diagonal = 5/6, off-diagonal = -1/6
    params.KI_theo = eye(6) - (1/6)*ones(6,6);

    % ========================================
    % COORDINATE TRANSFORMS
    % ========================================
    % T_m2a: Measuring -> Actuator coordinate
    params.T_m2a = [
        -1/sqrt(6),    1/sqrt(2),   -1/sqrt(3);
        -1/sqrt(6),   -1/sqrt(2),   -1/sqrt(3);
        -sqrt(2/3),    0,            1/sqrt(3)
    ];
    params.T_a2m = params.T_m2a';  % Inverse (orthogonal matrix)

    % ========================================
    % LUT PATH
    % ========================================
    params.lut_path = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'lut');

    % ========================================
    % PRE-COMPUTED MATRICES
    % ========================================
    % For inverse_model Stage 7-8: v_d = D_H_hat^-1 * K_I * I_d
    params.DH_hat_inv_KI = params.D_H_hat_inv * params.KI_theo;

    % ========================================
    % DEBUG / VERIFICATION ONLY
    % ========================================
    % These are NOT used by force_model or inverse_model
    % Kept for verify_cpp_match.m compatibility
    params.D_H = diag(d_H);
    params.KI_pinv = pinv(params.KI_theo);

end
