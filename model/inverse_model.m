function v_d = inverse_model(f_d, pos_m, params)
% INVERSE_MODEL Desired force -> Desired Hall sensor voltage
%
% Implements the 8-stage Inverse Model for magnetic tweezers:
%   Stage 1: Coordinate Transform (Measuring -> Actuator)
%   Stage 2: Octant Detection (based on position)
%   Stage 3: Angle Transformation + Address Calculation
%   Stage 4: Polynomial Expansion (10 terms)
%   Stage 5: Bilinear Interpolation (LUT lookup)
%   Stage 6: Octant Current Remap
%   Stage 7-8: v_d = D_H_hat^-1 * K_I * I_d
%
% Inputs:
%   f_d    - Desired force (3x1), Measuring coordinate [pN]
%   pos_m  - Bead position (3x1), Measuring coordinate [um]
%   params - System parameters from system_params()
%
% Output:
%   v_d    - Desired Hall sensor voltage (6x1) [V]

    % ========================================
    % PERSISTENT LUT DATA
    % ========================================
    persistent LUT_loaded LUT_data

    if isempty(LUT_loaded)
        LUT_data = load_lut_data(params.lut_path);
        LUT_loaded = true;
    end

    % ========================================
    % STAGE 1: COORDINATE TRANSFORM
    % ========================================
    f_a = params.T_m2a * f_d;
    pos_a = params.T_m2a * pos_m;

    % ========================================
    % FORCE MAGNITUDE AND ANGLES
    % ========================================
    F_mag = sqrt(f_a(1)^2 + f_a(2)^2 + f_a(3)^2);

    if F_mag < 1e-10
        v_d = zeros(6, 1);
        return;
    end

    AngleThe = atan2(f_a(2), f_a(1));  % Azimuth
    AnglePhi = asin(f_a(3) / F_mag);   % Elevation

    % ========================================
    % STAGE 2: OCTANT DETECTION
    % ========================================
    % Octant determined by POSITION signs, not force
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

    % ========================================
    % STAGE 3: ADDRESS CALCULATION
    % ========================================
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

    % ========================================
    % STAGE 4: POLYNOMIAL EXPANSION
    % ========================================
    R_norm = params.R_norm;

    P1 = abs(pos_a(1)) / R_norm;
    P2 = abs(pos_a(2)) / R_norm;
    P3 = abs(pos_a(3)) / R_norm;

    PosCoeff = [1; P1; P2; P3; P1^2; P2^2; P3^2; P1*P2; P1*P3; P2*P3];

    % ========================================
    % STAGE 5: BILINEAR INTERPOLATION
    % ========================================
    I_ind = zeros(6, 4);
    for corner = 1:4
        if corner == 1
            addr = addr_00;
        elseif corner == 2
            addr = addr_01;
        elseif corner == 3
            addr = addr_10;
        else
            addr = addr_11;
        end

        for k = 1:6
            I_ind(k, corner) = LUT_data{k}(addr, :) * PosCoeff;
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

    % ========================================
    % STAGE 6: OCTANT CURRENT REMAP
    % ========================================
    % Index swapping based on position octant
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

    % ========================================
    % STAGE 7-8: CURRENT TO VOLTAGE
    % ========================================
    v_d = params.DH_hat_inv_KI * I_d;

end


% ========================================
% HELPER: LOAD LUT DATA
% ========================================
function LUT_data = load_lut_data(lut_path)
% LOAD_LUT_DATA Load LUT coefficient files
%
% Returns cell array of 6 LUT matrices (one per current channel)
% Each matrix is 1891 x 10 (61x31 grid, 10 polynomial coefficients)

    LUT_data = cell(6, 1);

    for k = 1:6
        filename = sprintf('Feb28_2013 Coeff2nd_0CenterErr_I%d_1891x20.txt', k);
        filepath = fullfile(lut_path, filename);

        if ~exist(filepath, 'file')
            error('LUT file not found: %s', filepath);
        end

        LUT_data{k} = dlmread(filepath, '\t');

        [rows, cols] = size(LUT_data{k});
        if rows ~= 1891
            warning('LUT file %s has %d rows (expected 1891)', filename, rows);
        end
        if cols > 10
            LUT_data{k} = LUT_data{k}(:, 1:10);
        end
    end
end
