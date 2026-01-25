function [v_d, f_d_interp] = inverse_model_function(f_d, pos_m_ext, params)
% INVERSE_MODEL_FUNCTION Desired force -> Desired Hall voltage with rate transition
%
% Simulink-compatible inverse model with built-in rate transition.
% This version interpolates f_d (instead of vd) to match Ideal Tracking behavior.
% Uses fixed 625 µs period (62.5 steps @ 100kHz) to match FPGA Vd_Interpolator.
%
% Inputs:
%   f_d        - Desired force (3x1), Measuring coordinate [pN]
%   pos_m_ext  - External bead position (3x1) [um], used when pos_m_source=1
%   params     - Parameters from force_model_allocation_params('Simulink', true)
%                Required fields:
%                .pos_m            - Bead position (3x1) [um] (used when pos_m_source=0)
%                .pos_m_source     - 0=static (from params), 1=dynamic (from external)
%                .sample_rate_mode - 1=ZOH, 2=Linear, 3=Direct
%                .T_m2a            - Measuring to Actuator transform (3x3)
%                .g_H              - Force gain [pN/V^2]
%                .force_scale      - Force scaling factor
%                .R_norm           - Normalization radius [um]
%                .DH_hat_inv_KI    - Pre-computed D_H_hat^-1 * K_I (6x6)
%                .LUT              - Pre-loaded LUT data (6 x 1891 x 10)
%
% Outputs:
%   v_d        - Desired Hall sensor voltage (6x1) [V]
%   f_d_interp - Interpolated desired force (3x1) [pN], for verification
%
% Rate Transition Modes:
%   1 (ZOH)    - Zero-order hold f_d at 1600 Hz, compute vd every step
%   2 (Linear) - Linear interpolation of f_d at 1600 Hz, compute vd every step
%   3 (Direct) - Direct execution at 100 kHz (no rate transition)
%
% Interpolation Method (FPGA-compatible):
%   - Fixed frame_period = 62.5 steps (matches FPGA: 62500 @ 100MHz / 1000 divider)
%   - Alpha = phase_counter / 62.5, range [0, ~0.992]
%   - Boundary detection uses fractional accumulator for precise 1600 Hz
%
% See also: force_model_allocation_params, inverse_model, CLAUDE.md

%#codegen

    persistent fd_prev fd_curr phase_counter initialized first_boundary
    persistent phase_accumulator  % Fractional accumulator for precise 1600 Hz boundary

    if isempty(initialized)
        fd_prev = zeros(3, 1);
        fd_curr = zeros(3, 1);
        phase_counter = int32(0);
        phase_accumulator = 0.0;  % Accumulates fractional part
        first_boundary = true;
        initialized = true;
    end

    %% Extract parameters from Bus
    % Select pos_m source: 0=static (from params), 1=dynamic (from external)
    if params.pos_m_source > 0.5
        pos_m = pos_m_ext;
    else
        pos_m = params.pos_m;
    end
    sample_rate_mode = params.sample_rate_mode;

    %% Rate Transition Constants (FPGA-compatible)
    % 100 kHz / 1600 Hz = 62.5 steps per frame
    % FPGA uses: frame_period = 62500 @ 100MHz, output_divider = 1000
    % Equivalent: 62500 / 1000 = 62.5 steps @ 100kHz
    FRAME_PERIOD = 62.5;

    %% Check if at 1600 Hz boundary (using fractional accumulator)
    % Boundary occurs when phase_accumulator crosses integer threshold
    is_boundary = (phase_counter == int32(0));

    %% Update f_d boundary values at 1600 Hz rate
    if sample_rate_mode ~= 3 && is_boundary
        if first_boundary
            % First boundary: eliminate startup transient
            % Set both prev and curr to current f_d to avoid zero-start ramp
            fd_prev = f_d;
            fd_curr = f_d;
            first_boundary = false;
        else
            fd_prev = fd_curr;
            fd_curr = f_d;
        end
    end

    %% Interpolate f_d based on sample rate mode
    if sample_rate_mode == 1
        % ZOH mode: hold current boundary value
        f_d_interp = fd_curr;

    elseif sample_rate_mode == 2
        % Linear Interpolation mode: interpolate f_d
        % Alpha calculation matches FPGA: alpha = frame_counter / frame_period
        t_frac = double(phase_counter) / FRAME_PERIOD;
        if t_frac > 1.0
            t_frac = 1.0;  % Clamp to [0, 1]
        end
        f_d_interp = fd_prev + t_frac * (fd_curr - fd_prev);

    else
        % Direct mode (100 kHz): use input f_d directly
        f_d_interp = f_d;
    end

    %% Compute v_d using interpolated f_d (every step!)
    v_d = inverse_model_core(f_d_interp, pos_m, params);

    %% Increment phase counter with fractional accumulator (FPGA-compatible)
    % This achieves precise 1600 Hz average by accumulating the 0.5 remainder
    phase_accumulator = phase_accumulator + 1.0;
    phase_counter = phase_counter + int32(1);

    % Check for frame boundary: when accumulator reaches FRAME_PERIOD
    if phase_accumulator >= FRAME_PERIOD
        phase_accumulator = phase_accumulator - FRAME_PERIOD;  % Keep fractional remainder
        phase_counter = int32(0);
    end

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
