function alloc_params = force_model_allocation_params()
% FORCE_MODEL_ALLOCATION_PARAMS System parameters for force model and inverse model
%
% Provides parameters for:
%   - inverse_model.m (f_d -> v_d)
%   - force_model.m (v_m -> f_m)
%
% Output:
%   alloc_params - Structure containing all system parameters
%
% Example:
%   alloc_params = force_model_allocation_params();
%
% See also: force_model, inverse_model, CLAUDE.md

    %% Basic Constants
    alloc_params.R_norm = 550.0;           % [um] Normalization radius
    alloc_params.force_scale = 10.0;       % Force scaling factor

    %% Hall Sensor Matrices
    % D_H: Hall sensor gain matrix (diagonal)
    % D_H_hat: Normalized by first element (D_H_hat(1,1) = 1)
    d_H = [1/2.252, 1/2.749, 1/2.246, 1/2.294, 1/2.832, 1/2.295];
    d_H1 = d_H(1);                            % Normalization factor

    alloc_params.D_H_hat = diag(d_H / d_H1);        % Normalized D_H
    alloc_params.D_H_hat_inv = diag(1./d_H * d_H1); % Normalized D_H^-1

    %% Force Gain
    % g_H: Experimental calibration with normalization absorbed
    % g_H = 4.741 * d_H1^2 (approx 0.935 pN/V^2)
    alloc_params.g_H = 4.741 * d_H1^2;

    %% Flux Allocation Matrix
    % K_I = I - (1/6)*J, where J is all-ones matrix
    % Property: row sum = 0, diagonal = 5/6, off-diagonal = -1/6
    alloc_params.KI_theo = eye(6) - (1/6)*ones(6,6);

    %% Coordinate Transforms
    % T_m2a: Measuring -> Actuator coordinate
    alloc_params.T_m2a = [
        -1/sqrt(6),    1/sqrt(2),   -1/sqrt(3);
        -1/sqrt(6),   -1/sqrt(2),   -1/sqrt(3);
        -sqrt(2/3),    0,            1/sqrt(3)
    ];
    alloc_params.T_a2m = alloc_params.T_m2a';  % Inverse (orthogonal matrix)

    %% LUT Path
    alloc_params.lut_path = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'data', 'lut');

    %% Pre-computed Matrices
    % For inverse_model Stage 7-8: v_d = D_H_hat^-1 * K_I * I_d
    alloc_params.DH_hat_inv_KI = alloc_params.D_H_hat_inv * alloc_params.KI_theo;

    %% Debug / Verification Only
    % These are NOT used by force_model or inverse_model
    % Kept for verify_cpp_match.m compatibility
    alloc_params.D_H = diag(d_H);
    alloc_params.KI_pinv = pinv(alloc_params.KI_theo);

end
