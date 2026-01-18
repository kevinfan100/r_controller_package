function v_d = inverse_model_function(f_d, params)
% INVERSE_MODEL_FUNCTION Desired force -> Desired Hall voltage with rate transition
%
% Simulink-compatible inverse model with built-in rate transition.
% All parameters (including pos_m, sample_rate_mode) come from params.
% Phase counter is managed internally using persistent variables.
%
% This function implements the 8-stage Inverse Model pipeline plus
% rate transition logic for simulating hardware constraints.
%
% Inputs:
%   f_d    - Desired force (3x1), Measuring coordinate [pN]
%   params - Parameters from force_model_allocation_params('Simulink', true)
%            Required fields:
%            .pos_m            - Bead position (3x1) [um]
%            .sample_rate_mode - 1=ZOH, 2=Linear, 3=Direct
%            .T_m2a            - Measuring to Actuator transform (3x3)
%            .g_H              - Force gain [pN/V^2]
%            .force_scale      - Force scaling factor
%            .R_norm           - Normalization radius [um]
%            .DH_hat_inv_KI    - Pre-computed D_H_hat^-1 * K_I (6x6)
%            .LUT              - Pre-loaded LUT data (6 x 1891 x 10)
%
% Output:
%   v_d    - Desired Hall sensor voltage (6x1) [V]
%
% Rate Transition Modes:
%   1 (ZOH)    - Zero-order hold at 1600 Hz
%   2 (Linear) - Linear interpolation at 1600 Hz (default)
%   3 (Direct) - Direct execution at 100 kHz
%
% See also: force_model_allocation_params, inverse_model, CLAUDE.md

%#codegen

    persistent vd_prev vd_curr phase_counter initialized

    if isempty(initialized)
        vd_prev = zeros(6, 1);
        vd_curr = zeros(6, 1);
        phase_counter = 0;
        initialized = true;
    end

    %% Extract parameters from Bus
    pos_m = params.pos_m;
    sample_rate_mode = params.sample_rate_mode;

    %% Rate Transition Constants
    % 100 kHz / 1600 Hz = 62.5, use 63 for integer cycle
    PHASE_PERIOD = 63;
    INTERP_RATIO = 62.5;

    %% Check if at 1600 Hz boundary
    is_boundary = (phase_counter == 0);

    %% Calculate vd at 1600 Hz boundary or Direct mode
    if sample_rate_mode == 3 || is_boundary
        vd_raw = inverse_model_core(f_d, pos_m, params);

        if sample_rate_mode ~= 3
            vd_prev = vd_curr;
            vd_curr = vd_raw;
        end
    end

    %% Rate Transition Output
    if sample_rate_mode == 1
        % ZOH mode: hold current value
        v_d = vd_curr;

    elseif sample_rate_mode == 2
        % Linear Interpolation mode
        t_frac = double(phase_counter) / INTERP_RATIO;
        v_d = vd_prev + t_frac * (vd_curr - vd_prev);

    else
        % Direct mode (100 kHz): compute every step
        v_d = inverse_model_core(f_d, pos_m, params);
    end

    %% Increment phase counter for next call
    phase_counter = mod(phase_counter + 1, PHASE_PERIOD);

end


% ========================================================================
% LOCAL FUNCTIONS (Code Generation Compatible)
% ========================================================================

function v_d = inverse_model_core(f_d, pos_m, params)
% INVERSE_MODEL_CORE 8-stage inverse model pipeline
%
% Ported from inverse_model.m with LUT data from params instead of dlmread.

    %% Stage 1: Coordinate Transform
    f_a = params.T_m2a * f_d;
    pos_a = params.T_m2a * pos_m;

    %% Force Magnitude and Angles
    F_mag = sqrt(f_a(1)^2 + f_a(2)^2 + f_a(3)^2);

    if F_mag < 1e-10
        v_d = zeros(6, 1);
        return;
    end

    AngleThe = atan2(f_a(2), f_a(1));  % Azimuth
    AnglePhi = asin(f_a(3) / F_mag);   % Elevation

    %% Stage 2: Octant Detection
    x = pos_a(1);
    y = pos_a(2);
    z = pos_a(3);

    if x >= 0 && y >= 0 && z >= 0
        octant = 1;
        TheOct1 = AngleThe;
        PhiOct1 = AnglePhi;
    elseif x < 0 && y >= 0 && z >= 0
        octant = 2;
        if AngleThe >= 0
            TheOct1 = pi - AngleThe;
        else
            TheOct1 = -pi - AngleThe;
        end
        PhiOct1 = AnglePhi;
    elseif x <= 0 && y < 0 && z >= 0
        octant = 3;
        if AngleThe >= 0
            TheOct1 = -pi + AngleThe;
        else
            TheOct1 = pi + AngleThe;
        end
        PhiOct1 = AnglePhi;
    elseif x > 0 && y < 0 && z >= 0
        octant = 4;
        TheOct1 = -AngleThe;
        PhiOct1 = AnglePhi;
    elseif x >= 0 && y >= 0 && z < 0
        octant = 5;
        TheOct1 = AngleThe;
        PhiOct1 = -AnglePhi;
    elseif x < 0 && y >= 0 && z < 0
        octant = 6;
        if AngleThe >= 0
            TheOct1 = pi - AngleThe;
        else
            TheOct1 = -pi - AngleThe;
        end
        PhiOct1 = -AnglePhi;
    elseif x <= 0 && y < 0 && z < 0
        octant = 7;
        if AngleThe >= 0
            TheOct1 = -pi + AngleThe;
        else
            TheOct1 = pi + AngleThe;
        end
        PhiOct1 = -AnglePhi;
    else  % x > 0 && y < 0 && z < 0
        octant = 8;
        TheOct1 = -AngleThe;
        PhiOct1 = -AnglePhi;
    end

    %% Stage 3: Address Calculation
    CalcAngStep = 2*pi / 60;  % = pi/30

    IntThe = floor((TheOct1 + pi) / CalcAngStep);
    IntPhi = floor((PhiOct1 + pi/2) / CalcAngStep);

    IntThe = max(0, min(60, IntThe));
    IntPhi = max(0, min(30, IntPhi));

    FracThe = (TheOct1 + pi) / CalcAngStep - IntThe;
    FracPhi = (PhiOct1 + pi/2) / CalcAngStep - IntPhi;

    % LUT addresses (4 corners for bilinear interpolation)
    addr_00 = IntThe * 31 + IntPhi + 1;
    addr_10 = min(IntThe + 1, 60) * 31 + IntPhi + 1;
    addr_01 = IntThe * 31 + min(IntPhi + 1, 30) + 1;
    addr_11 = min(IntThe + 1, 60) * 31 + min(IntPhi + 1, 30) + 1;

    %% Stage 4: Polynomial Expansion
    R_norm = params.R_norm;

    P1 = abs(pos_a(1)) / R_norm;
    P2 = abs(pos_a(2)) / R_norm;
    P3 = abs(pos_a(3)) / R_norm;

    PosCoeff = [1; P1; P2; P3; P1^2; P2^2; P3^2; P1*P2; P1*P3; P2*P3];

    %% Stage 5: Bilinear Interpolation
    % LUT is stored as (6, 1891, 10) array
    I_ind = zeros(6, 4);
    addresses = [addr_00, addr_01, addr_10, addr_11];

    for corner = 1:4
        addr = addresses(corner);
        for k = 1:6
            % params.LUT(k, addr, :) is (1,1,10), squeeze to (10,1)
            lut_row = reshape(params.LUT(k, addr, :), [10, 1]);
            I_ind(k, corner) = lut_row' * PosCoeff;
        end
    end

    s = FracThe;
    t = FracPhi;

    % Current scaling: I = I_lut * sqrt(F_mag * force_scale / g_H)
    current_scale = sqrt(F_mag / (params.g_H / params.force_scale));

    I_interp = zeros(6, 1);
    if s ~= 0 && t ~= 0
        for k = 1:6
            mid13 = I_ind(k, 1) + s * (I_ind(k, 3) - I_ind(k, 1));
            mid24 = I_ind(k, 2) + s * (I_ind(k, 4) - I_ind(k, 2));
            I_interp(k) = (mid13 + t * (mid24 - mid13)) * current_scale;
        end
    elseif s == 0 && t ~= 0
        for k = 1:6
            I_interp(k) = (I_ind(k, 1) + t * (I_ind(k, 2) - I_ind(k, 1))) * current_scale;
        end
    elseif s ~= 0 && t == 0
        for k = 1:6
            I_interp(k) = (I_ind(k, 1) + s * (I_ind(k, 3) - I_ind(k, 1))) * current_scale;
        end
    else
        for k = 1:6
            I_interp(k) = I_ind(k, 1) * current_scale;
        end
    end

    %% Stage 6: Octant Current Remap
    I_d = zeros(6, 1);
    switch octant
        case 1
            I_d = I_interp([1, 2, 3, 4, 5, 6]);
        case 2
            I_d = I_interp([2, 1, 3, 4, 5, 6]);
        case 3
            I_d = I_interp([2, 1, 4, 3, 5, 6]);
        case 4
            I_d = I_interp([1, 2, 4, 3, 5, 6]);
        case 5
            I_d = I_interp([1, 2, 3, 4, 6, 5]);
        case 6
            I_d = I_interp([2, 1, 3, 4, 6, 5]);
        case 7
            I_d = I_interp([2, 1, 4, 3, 6, 5]);
        case 8
            I_d = I_interp([1, 2, 4, 3, 6, 5]);
    end

    %% Stage 7-8: Current to Voltage
    v_d = params.DH_hat_inv_KI * I_d;

end
