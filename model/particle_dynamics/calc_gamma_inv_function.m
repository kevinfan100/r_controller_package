function [Gamma_inv, h_bar] = calc_gamma_inv_function(p, params)
% CALC_GAMMA_INV_FUNCTION Calculate inverse drag matrix with Wall Effect
%
% Computes the position-dependent mobility matrix (inverse drag coefficient)
% accounting for the Wall Effect near a planar boundary.
%
% Formula:
%   Gamma_inv = (1/(gamma_N * c_para)) * (I - coeff * W)
%   where coeff = (c_perp - c_para) / c_perp
%         W = w_hat * w_hat' (wall projection matrix)
%
% Inputs:
%   p      - Particle position (3x1) [um], Measuring coordinate
%   params - ParticleDynamicsParamsBus
%
% Outputs:
%   Gamma_inv - 3x3 inverse drag matrix [um/(pN*s)]
%   h_bar     - Normalized distance h/R (for diagnostics)
%
% Physical Interpretation:
%   - For displacement d, velocity is: v = Gamma_inv * F
%   - Parallel to wall: v_para = F_para / (gamma_N * c_para)
%   - Perpendicular to wall: v_perp = F_perp / (gamma_N * c_perp)
%   - Far from wall (h_bar -> inf): Gamma_inv -> I / gamma_N
%   - Near wall: anisotropic mobility (harder to move perpendicular)
%
% Mathematical Derivation:
%   Let gamma_para = gamma_N * c_para and gamma_perp = gamma_N * c_perp
%   For any direction d = d_para + d_perp (parallel + perpendicular):
%     Gamma_inv * F = F_para/gamma_para + F_perp/gamma_perp
%
%   Using projection W = w_hat * w_hat':
%     F_perp = W * F,  F_para = (I - W) * F
%
%   Gamma_inv = (I-W)/gamma_para + W/gamma_perp
%             = (1/gamma_para) * [I - ((c_perp-c_para)/c_perp)*W]
%
% Example:
%   params = particle_dynamics_params();
%   p = [0; 0; 5];
%   [Gamma_inv, h_bar] = calc_gamma_inv_function(p, params.Value);
%
% See also: calc_correction_functions, particle_dynamics_params, CLAUDE.md

%#codegen

    % Extract parameters
    w_hat = params.w_hat;
    pz = params.pz;
    R = params.R;
    gamma_N = params.gamma_N;

    % Calculate distance from wall
    % h = (p . w_hat) - pz (signed distance along wall normal)
    h = dot(p, w_hat) - pz;

    % Normalize by particle radius
    h_bar = h / R;

    % Safety: ensure h_bar > 1 (particle not touching wall)
    h_bar = max(h_bar, 1.01);

    % Calculate correction coefficients
    [c_para, c_perp] = calc_correction_functions(h_bar);

    % Build Gamma_inv matrix
    % W = w_hat * w_hat' is the projection onto wall normal direction
    W = w_hat * w_hat';

    % coeff = (c_perp - c_para) / c_perp
    coeff = (c_perp - c_para) / c_perp;

    % Gamma_inv = (1/(gamma_N * c_para)) * (I - coeff * W)
    Gamma_inv = (1 / (gamma_N * c_para)) * (eye(3) - coeff * W);

end
