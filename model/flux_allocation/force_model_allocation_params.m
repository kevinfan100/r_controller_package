function alloc_params = force_model_allocation_params(varargin)
% FORCE_MODEL_ALLOCATION_PARAMS System parameters for force model and inverse model
%
% Provides parameters for:
%   - inverse_model.m / inverse_model_function.m (f_d -> v_d)
%   - force_model.m / force_model_function.m (v_m -> f_m)
%
% Usage:
%   alloc_params = force_model_allocation_params()
%       Returns struct (for offline MATLAB scripts)
%
%   alloc_params = force_model_allocation_params('Simulink', true, ...)
%       Returns Simulink.Parameter with ForceModelAllocationParamsBus
%
% Simulink-specific Parameters (Name-Value):
%   'Simulink'       - Enable Simulink mode (default: false)
%   'pos_m'          - Bead position [um] (default: [0;0;0])
%   'SampleRateMode' - 1=ZOH, 2=Linear, 3=Direct (default: 2)
%   'PosMSource'     - 0=static (from Bus), 1=dynamic (from external) (default: 0)
%
% Output (offline mode):
%   alloc_params - Structure with all system parameters
%
% Output (Simulink mode):
%   alloc_params - Simulink.Parameter with ForceModelAllocationParamsBus type
%
% Example:
%   % Offline usage
%   params = force_model_allocation_params();
%   v_d = inverse_model(f_d, pos_m, params);
%
%   % Simulink usage
%   params = force_model_allocation_params('Simulink', true, 'pos_m', [100;50;0]);
%   assignin('base', 'alloc_params', params);
%
% See also: inverse_model, force_model, inverse_model_function, force_model_function, CLAUDE.md

    %% Parse Input Parameters
    p = inputParser;
    addParameter(p, 'Simulink', false, @islogical);
    addParameter(p, 'pos_m', [0; 0; 0], @(x) isnumeric(x) && numel(x) == 3);
    addParameter(p, 'SampleRateMode', 2, @(x) ismember(x, [1, 2, 3]));
    addParameter(p, 'PosMSource', 0, @(x) ismember(x, [0, 1]));
    parse(p, varargin{:});

    use_simulink = p.Results.Simulink;

    %% Basic Constants
    params.R_norm = 550.0;           % [um] Normalization radius
    params.force_scale = 10.0;       % Force scaling factor

    %% Hall Sensor Matrices
    % D_H: Hall sensor gain matrix (diagonal)
    % D_H_hat: Normalized by first element (D_H_hat(1,1) = 1)
    d_H = [1/2.252, 1/2.749, 1/2.246, 1/2.294, 1/2.832, 1/2.295];
    d_H1 = d_H(1);                            % Normalization factor

    params.D_H_hat = diag(d_H / d_H1);        % Normalized D_H
    params.D_H_hat_inv = diag(1./d_H * d_H1); % Normalized D_H^-1

    %% Force Gain
    % g_H: Experimental calibration with normalization absorbed
    % g_H = 4.741 * d_H1^2 (approx 0.935 pN/V^2)
    params.g_H = 4.741 * d_H1^2;

    %% Flux Allocation Matrix
    % K_I = I - (1/6)*J, where J is all-ones matrix
    % Property: row sum = 0, diagonal = 5/6, off-diagonal = -1/6
    params.KI_theo = eye(6) - (1/6)*ones(6,6);

    %% Coordinate Transforms
    % T_m2a: Measuring -> Actuator coordinate
    params.T_m2a = [
        -1/sqrt(6),    1/sqrt(2),   -1/sqrt(3);
        -1/sqrt(6),   -1/sqrt(2),   -1/sqrt(3);
        -sqrt(2/3),    0,            1/sqrt(3)
    ];
    params.T_a2m = params.T_m2a';  % Inverse (orthogonal matrix)

    %% LUT Path
    params.lut_path = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'data', 'lut');

    %% Pre-computed Matrices
    % For inverse_model Stage 7-8: v_d = D_H_hat^-1 * K_I * I_d
    params.DH_hat_inv_KI = params.D_H_hat_inv * params.KI_theo;

    %% Debug / Verification Only
    % These are NOT used by force_model or inverse_model
    % Kept for verify_cpp_match.m compatibility
    params.D_H = diag(d_H);
    params.KI_pinv = pinv(params.KI_theo);

    %% Output Based on Mode
    if use_simulink
        % Add Simulink-specific parameters
        params.pos_m = reshape(p.Results.pos_m, [3, 1]);
        params.sample_rate_mode = p.Results.SampleRateMode;
        params.pos_m_source = p.Results.PosMSource;

        % Pre-load LUT for Simulink (avoid dlmread in real-time)
        params.LUT = load_lut_array(params.lut_path);

        % Create Bus Object and Simulink.Parameter
        alloc_params = create_alloc_params_bus(params);
    else
        % Return plain struct (backward compatible for offline scripts)
        alloc_params = params;
    end

end


% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function LUT = load_lut_array(lut_path)
% LOAD_LUT_ARRAY Load LUT data as 3D array for Simulink
%
% Returns LUT as (6, 1891, 10) array instead of cell array
% This format is compatible with MATLAB code generation.

    LUT = zeros(6, 1891, 10);

    for k = 1:6
        filename = sprintf('Feb28_2013 Coeff2nd_0CenterErr_I%d_1891x20.txt', k);
        filepath = fullfile(lut_path, filename);

        if ~exist(filepath, 'file')
            error('LUT file not found: %s', filepath);
        end

        data = dlmread(filepath, '\t');

        [rows, cols] = size(data);
        if rows ~= 1891
            warning('LUT file %s has %d rows (expected 1891)', filename, rows);
        end

        % Extract first 10 columns
        if cols >= 10
            LUT(k, :, :) = reshape(data(:, 1:10), [1, 1891, 10]);
        else
            LUT(k, 1:rows, 1:cols) = reshape(data, [1, rows, cols]);
        end
    end

end


function alloc_params = create_alloc_params_bus(params)
% CREATE_ALLOC_PARAMS_BUS Create Simulink Bus Object and Parameter
%
% Creates ForceModelAllocationParamsBus and wraps params as Simulink.Parameter

    %% Define Bus Elements
    % Note: Simulink Bus requires fixed-size arrays. We must define
    % dimensions explicitly for matrices.

    bus = Simulink.Bus;
    bus.Description = 'Force Model Allocation Parameters';

    elems = [];

    % Scalar parameters
    elem = Simulink.BusElement;
    elem.Name = 'R_norm';
    elem.DataType = 'double';
    elem.Dimensions = 1;
    elems = [elems; elem];

    elem = Simulink.BusElement;
    elem.Name = 'force_scale';
    elem.DataType = 'double';
    elem.Dimensions = 1;
    elems = [elems; elem];

    elem = Simulink.BusElement;
    elem.Name = 'g_H';
    elem.DataType = 'double';
    elem.Dimensions = 1;
    elems = [elems; elem];

    % Matrix parameters (6x6)
    elem = Simulink.BusElement;
    elem.Name = 'D_H_hat';
    elem.DataType = 'double';
    elem.Dimensions = [6, 6];
    elems = [elems; elem];

    elem = Simulink.BusElement;
    elem.Name = 'D_H_hat_inv';
    elem.DataType = 'double';
    elem.Dimensions = [6, 6];
    elems = [elems; elem];

    elem = Simulink.BusElement;
    elem.Name = 'KI_theo';
    elem.DataType = 'double';
    elem.Dimensions = [6, 6];
    elems = [elems; elem];

    elem = Simulink.BusElement;
    elem.Name = 'DH_hat_inv_KI';
    elem.DataType = 'double';
    elem.Dimensions = [6, 6];
    elems = [elems; elem];

    % Coordinate transforms (3x3)
    elem = Simulink.BusElement;
    elem.Name = 'T_m2a';
    elem.DataType = 'double';
    elem.Dimensions = [3, 3];
    elems = [elems; elem];

    elem = Simulink.BusElement;
    elem.Name = 'T_a2m';
    elem.DataType = 'double';
    elem.Dimensions = [3, 3];
    elems = [elems; elem];

    % Position (3x1) - Simulink specific
    elem = Simulink.BusElement;
    elem.Name = 'pos_m';
    elem.DataType = 'double';
    elem.Dimensions = [3, 1];
    elems = [elems; elem];

    % Sample rate mode - Simulink specific
    elem = Simulink.BusElement;
    elem.Name = 'sample_rate_mode';
    elem.DataType = 'double';
    elem.Dimensions = 1;
    elems = [elems; elem];

    % pos_m source - Simulink specific (0=static, 1=dynamic)
    elem = Simulink.BusElement;
    elem.Name = 'pos_m_source';
    elem.DataType = 'double';
    elem.Dimensions = 1;
    elems = [elems; elem];

    % LUT (6 x 1891 x 10) - Simulink specific
    elem = Simulink.BusElement;
    elem.Name = 'LUT';
    elem.DataType = 'double';
    elem.Dimensions = [6, 1891, 10];
    elems = [elems; elem];

    bus.Elements = elems;

    % Assign Bus Object to base workspace
    assignin('base', 'ForceModelAllocationParamsBus', bus);

    %% Create Simulink.Parameter
    % Build struct with only the fields that are in the Bus
    param_struct = struct();
    param_struct.R_norm = params.R_norm;
    param_struct.force_scale = params.force_scale;
    param_struct.g_H = params.g_H;
    param_struct.D_H_hat = params.D_H_hat;
    param_struct.D_H_hat_inv = params.D_H_hat_inv;
    param_struct.KI_theo = params.KI_theo;
    param_struct.DH_hat_inv_KI = params.DH_hat_inv_KI;
    param_struct.T_m2a = params.T_m2a;
    param_struct.T_a2m = params.T_a2m;
    param_struct.pos_m = params.pos_m;
    param_struct.sample_rate_mode = params.sample_rate_mode;
    param_struct.pos_m_source = params.pos_m_source;
    param_struct.LUT = params.LUT;

    alloc_params = Simulink.Parameter(param_struct);
    alloc_params.DataType = 'Bus: ForceModelAllocationParamsBus';

end
