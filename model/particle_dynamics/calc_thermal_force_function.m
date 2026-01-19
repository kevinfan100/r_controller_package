function F_th = calc_thermal_force_function(p, params)
% CALC_THERMAL_FORCE_FUNCTION Generate random thermal force (Brownian motion)
%
% Generates a random thermal force based on the particle position,
% accounting for Wall Effect on the diffusion coefficient.
%
% Based on Einstein relation: F_th ~ N(0, Sigma)
% With Wall Effect: direction-dependent variance
%
% Input:
%   p      - Particle position (3x1) [um], Measuring coordinate
%   params - ThermalForceParamsBus
%
% Output:
%   F_th   - Random thermal force (3x1) [pN]
%
% Physical Background:
%   Einstein relation: D = k_B*T / gamma
%   Force variance: <F_th^2> = 4*k_B*T*gamma / Ts
%
%   With Wall Effect (direction-dependent):
%     C = c_para*(u_hat + v_hat) + c_perp*w_hat
%     Variance = variance_coeff * abs(C)
%     F_th = sqrt(Variance) .* randn(3,1)
%
% Note:
%   - Random seed is initialized on first call
%   - Returns zeros when enable = 0
%
% Example:
%   params = thermal_force_params('Enable', 1, 'Seed', 42);
%   F_th = calc_thermal_force_function([0;0;5], params.Value);
%
% See also: thermal_force_params, calc_correction_functions, CLAUDE.md

%#codegen

    persistent rng_initialized

    % Initialize random seed on first call
    if isempty(rng_initialized)
        if params.seed >= 0
            rng(params.seed);
        end
        rng_initialized = true;
    end

    % Check if thermal force is enabled
    if params.enable < 0.5
        F_th = zeros(3, 1);
        return;
    end

    % Extract parameters
    w_hat = params.w_hat;
    u_hat = params.u_hat;
    v_hat = params.v_hat;
    pz = params.pz;
    R = params.R;
    variance_coeff = params.variance_coeff;  % 4*k_B*T*gamma_N/Ts

    % Calculate normalized distance from wall
    h = dot(p, w_hat) - pz;
    h_bar = h / R;

    % Safety: ensure h_bar > 1
    h_bar = max(h_bar, 1.01);

    % Calculate correction coefficients
    [c_para, c_perp] = calc_correction_functions(h_bar);

    % Direction-dependent variance (MotionControl_Simu original formula)
    % C = c_para * (u_hat + v_hat) + c_perp * w_hat
    C = c_para * (u_hat + v_hat) + c_perp * w_hat;
    Variance = variance_coeff * abs(C);

    % Generate random thermal force
    % F_th ~ N(0, Variance) => F_th = sqrt(Variance) .* randn(3,1)
    F_th = sqrt(Variance) .* randn(3, 1);

end
