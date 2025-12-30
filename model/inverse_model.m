function v_d = inverse_model(f_d, pos_m, params)
% INVERSE_MODEL Green Block: Desired force -> Desired Hall sensor voltage
%
% Implements the complete 8-stage Inverse Model for magnetic tweezers:
%   Stage 1: Coordinate Transform (Measuring -> Actuator)
%   Stage 2: Octant Detection (based on POSITION)
%   Stage 3: Angle Transformation + Address Calculation
%   Stage 4: Polynomial Expansion (10 terms)
%   Stage 5: Bilinear Interpolation (LUT lookup)
%   Stage 6: Octant Current Remap (based on POSITION)
%   Stage 7-8: v_d = D_H^-1 * K_I * I_d
%
% Inputs:
%   f_d    - Desired force [Fx; Fy; Fz] (3x1), Measuring coordinate [pN]
%   pos_m  - Bead position [x; y; z] (3x1), Measuring coordinate [um]
%   params - System parameters from system_params()
%
% Output:
%   v_d    - Desired Hall sensor voltage (6x1) [V]
%
% References:
%   - InverseModel_Math_Derivation.pdf
%   - CALCULATION.cpp (MotionControl_KalmanFilter_Simu) lines 2081-2372

    %% ════════════════════════════════════════════════════════════════════
    %                   Persistent LUT Data
    %% ════════════════════════════════════════════════════════════════════
    persistent LUT_loaded LUT_data

    if isempty(LUT_loaded)
        LUT_data = load_lut_data(params.lut_path);
        LUT_loaded = true;
    end

    %% ════════════════════════════════════════════════════════════════════
    %                   Stage 1: Coordinate Transform
    %% ════════════════════════════════════════════════════════════════════
    % Transform from Measuring to Actuator coordinate system
    f_a = params.T_m2a * f_d;
    pos_a = params.T_m2a * pos_m;

    %% ════════════════════════════════════════════════════════════════════
    %                   Force Magnitude and Angles
    %% ════════════════════════════════════════════════════════════════════
    % Use original (unscaled) force for angle calculation
    % Scaling is applied later in current output: I = I_lut * sqrt(F_mag / (g_H/10))
    F_mag = sqrt(f_a(1)^2 + f_a(2)^2 + f_a(3)^2);

    if F_mag < 1e-10
        % Zero force: return zero voltage
        v_d = zeros(6, 1);
        return;
    end

    % Spherical angles from FORCE direction (with signs!)
    AngleThe = atan2(f_a(2), f_a(1));  % theta: azimuth
    AnglePhi = asin(f_a(3) / F_mag);   % phi: elevation

    %% ════════════════════════════════════════════════════════════════════
    %                   Stage 2: Octant Detection (based on POSITION!)
    %% ════════════════════════════════════════════════════════════════════
    % From CALCULATION.cpp lines 2092-2159
    % Octant is determined by POSITION signs, NOT force signs!
    %
    % C++ Octant numbering (1-8):
    %   Octant1: Pos_x >= 0, Pos_y >= 0, Pos_z >= 0
    %   Octant2: Pos_x <  0, Pos_y >= 0, Pos_z >= 0
    %   Octant3: Pos_x <= 0, Pos_y <  0, Pos_z >= 0
    %   Octant4: Pos_x >  0, Pos_y <  0, Pos_z >= 0
    %   Octant5: Pos_x >= 0, Pos_y >= 0, Pos_z <  0
    %   Octant6: Pos_x <  0, Pos_y >= 0, Pos_z <  0
    %   Octant7: Pos_x <= 0, Pos_y <  0, Pos_z <  0
    %   Octant8: Pos_x >  0, Pos_y <  0, Pos_z <  0

    x = pos_a(1);
    y = pos_a(2);
    z = pos_a(3);

    % Determine position octant and apply angle transformation
    if x >= 0 && y >= 0 && z >= 0
        % Octant1: No transformation
        octant = 1;
        TheOct1 = AngleThe;
        PhiOct1 = AnglePhi;
    elseif x < 0 && y >= 0 && z >= 0
        % Octant2
        octant = 2;
        if AngleThe >= 0
            TheOct1 = pi - AngleThe;
        else
            TheOct1 = -pi - AngleThe;
        end
        PhiOct1 = AnglePhi;
    elseif x <= 0 && y < 0 && z >= 0
        % Octant3
        octant = 3;
        if AngleThe >= 0
            TheOct1 = -pi + AngleThe;
        else
            TheOct1 = pi + AngleThe;
        end
        PhiOct1 = AnglePhi;
    elseif x > 0 && y < 0 && z >= 0
        % Octant4
        octant = 4;
        TheOct1 = -AngleThe;
        PhiOct1 = AnglePhi;
    elseif x >= 0 && y >= 0 && z < 0
        % Octant5
        octant = 5;
        TheOct1 = AngleThe;
        PhiOct1 = -AnglePhi;
    elseif x < 0 && y >= 0 && z < 0
        % Octant6
        octant = 6;
        if AngleThe >= 0
            TheOct1 = pi - AngleThe;
        else
            TheOct1 = -pi - AngleThe;
        end
        PhiOct1 = -AnglePhi;
    elseif x <= 0 && y < 0 && z < 0
        % Octant7
        octant = 7;
        if AngleThe >= 0
            TheOct1 = -pi + AngleThe;
        else
            TheOct1 = pi + AngleThe;
        end
        PhiOct1 = -AnglePhi;
    else  % x > 0 && y < 0 && z < 0
        % Octant8
        octant = 8;
        TheOct1 = -AngleThe;
        PhiOct1 = -AnglePhi;
    end

    %% ════════════════════════════════════════════════════════════════════
    %                   Stage 3: Address Calculation
    %% ════════════════════════════════════════════════════════════════════
    % From CALCULATION.cpp lines 2161-2169
    CalcAngStep = 2*pi / 60;  % = pi/30

    IntThe = floor((TheOct1 + pi) / CalcAngStep);
    IntPhi = floor((PhiOct1 + pi/2) / CalcAngStep);

    % Clamp to valid range
    IntThe = max(0, min(60, IntThe));
    IntPhi = max(0, min(30, IntPhi));

    % Fractional parts for bilinear interpolation
    FracThe = (TheOct1 + pi) / CalcAngStep - IntThe;
    FracPhi = (PhiOct1 + pi/2) / CalcAngStep - IntPhi;

    % Calculate LUT addresses (4 corners for bilinear interpolation)
    % LUT layout: 61 theta points x 31 phi points = 1891 entries
    % C++ uses 0-indexed: ind = IntThe*31 + IntPhi
    addr_00 = IntThe * 31 + IntPhi + 1;           % MATLAB 1-indexed
    addr_10 = min(IntThe + 1, 60) * 31 + IntPhi + 1;
    addr_01 = IntThe * 31 + min(IntPhi + 1, 30) + 1;
    addr_11 = min(IntThe + 1, 60) * 31 + min(IntPhi + 1, 30) + 1;

    %% ════════════════════════════════════════════════════════════════════
    %                   Stage 4: Polynomial Expansion
    %% ════════════════════════════════════════════════════════════════════
    % Position-dependent polynomial terms (10 terms)
    % From CALCULATION.cpp lines 2178-2187
    R_norm = params.R_norm;

    P1 = abs(pos_a(1)) / R_norm;  % |Mx| / R_norm
    P2 = abs(pos_a(2)) / R_norm;  % |My| / R_norm
    P3 = abs(pos_a(3)) / R_norm;  % |Mz| / R_norm

    PosCoeff = [
        1;           % P0: constant term
        P1;          % P1: |Mx| / R_norm
        P2;          % P2: |My| / R_norm
        P3;          % P3: |Mz| / R_norm
        P1^2;        % P4: P1^2
        P2^2;        % P5: P2^2
        P3^2;        % P6: P3^2
        P1 * P2;     % P7: P1 * P2
        P1 * P3;     % P8: P1 * P3
        P2 * P3      % P9: P2 * P3
    ];

    %% ════════════════════════════════════════════════════════════════════
    %                   Stage 5: Bilinear Interpolation
    %% ════════════════════════════════════════════════════════════════════
    % From CALCULATION.cpp lines 2189-2297

    % Compute currents at each LUT index
    I_ind = zeros(6, 4);  % 6 currents x 4 corners
    for corner = 1:4
        if corner == 1
            addr = addr_00;
        elseif corner == 2
            addr = addr_01;  % Note: C++ ind2 = ind1 + 1 (phi direction)
        elseif corner == 3
            addr = addr_10;  % Note: C++ ind3 = ind1 + 31 (theta direction)
        else
            addr = addr_11;
        end

        for k = 1:6
            I_ind(k, corner) = LUT_data{k}(addr, :) * PosCoeff;
        end
    end

    % Bilinear interpolation
    % C++ uses: mid13 = ind1 + FracThe*(ind3 - ind1)
    %           mid24 = ind2 + FracThe*(ind4 - ind2)
    %           interp = mid13 + FracPhi*(mid24 - mid13)
    s = FracThe;  % theta fraction
    t = FracPhi;  % phi fraction

    % Current scaling: I = I_lut * sqrt(F_mag / (g_H/10))
    %                    = I_lut * sqrt(F_mag * 10/g_H)
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

    %% ════════════════════════════════════════════════════════════════════
    %                   Stage 6: Octant Current Remap (based on POSITION!)
    %% ════════════════════════════════════════════════════════════════════
    % From CALCULATION.cpp lines 2300-2371
    % This is INDEX SWAPPING only, no sign changes!
    %
    % The remap table shows which I_interp index maps to each output:
    %   [I1_out, I2_out, I3_out, I4_out, I5_out, I6_out] = I_interp([remap indices])

    I_d = zeros(6, 1);
    switch octant
        case 1  % Octant1: Identity
            I_d = I_interp([1, 2, 3, 4, 5, 6]);
        case 2  % Octant2: Swap I1/I2
            I_d = I_interp([2, 1, 3, 4, 5, 6]);
        case 3  % Octant3: Swap I1/I2, I3/I4
            I_d = I_interp([2, 1, 4, 3, 5, 6]);
        case 4  % Octant4: Swap I3/I4
            I_d = I_interp([1, 2, 4, 3, 5, 6]);
        case 5  % Octant5: Swap I5/I6
            I_d = I_interp([1, 2, 3, 4, 6, 5]);
        case 6  % Octant6: Swap I1/I2, I5/I6
            I_d = I_interp([2, 1, 3, 4, 6, 5]);
        case 7  % Octant7: Swap I1/I2, I3/I4, I5/I6
            I_d = I_interp([2, 1, 4, 3, 6, 5]);
        case 8  % Octant8: Swap I3/I4, I5/I6
            I_d = I_interp([1, 2, 4, 3, 6, 5]);
    end

    %% ════════════════════════════════════════════════════════════════════
    %                   Stage 7-8: Current to Voltage
    %% ════════════════════════════════════════════════════════════════════
    % v_d = D̂_H^-1 * K_I * I_d (using normalized D̂_H^-1)
    v_d = params.DH_hat_inv_KI * I_d;

end

%% ════════════════════════════════════════════════════════════════════════
%                   Helper Function: Load LUT Data
%% ════════════════════════════════════════════════════════════════════════
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

        % Load tab-separated file
        LUT_data{k} = dlmread(filepath, '\t');

        % Verify dimensions (should be 1891 x 10)
        [rows, cols] = size(LUT_data{k});
        if rows ~= 1891
            warning('LUT file %s has %d rows (expected 1891)', filename, rows);
        end
        if cols ~= 10
            % File name says 1891x20 but actual content is 10 columns
            if cols > 10
                LUT_data{k} = LUT_data{k}(:, 1:10);
            end
        end
    end
end
