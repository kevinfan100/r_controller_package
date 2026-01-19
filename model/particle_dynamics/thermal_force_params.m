function thermal_force_params = thermal_force_params(varargin)
% THERMAL_FORCE_PARAMS Create parameters for thermal force generator
%
% Creates a Simulink.Parameter with ThermalForceParamsBus type for use
% with calc_thermal_force_function in Simulink MATLAB Function blocks.
%
% Usage:
%   thermal_force_params = thermal_force_params()
%       Returns Simulink.Parameter with default values
%
%   thermal_force_params = thermal_force_params('Enable', 1, ...)
%       Returns Simulink.Parameter with custom values
%
% Parameters (Name-Value pairs):
%   'Enable'   - Enable thermal force (default: 1)
%   'kB'       - Boltzmann constant [pN*um/K] (default: 1.3806503e-5)
%   'T'        - Temperature [K] (default: 310.15 = 37C)
%   'GammaN'   - Stokes drag coefficient [pN*s/um] (default: 0.0425)
%   'Ts'       - Sampling period [s] (default: 1/1600)
%   'Seed'     - Random seed for reproducibility (default: random)
%   'R'        - Particle radius [um] (default: 2.25)
%   'Theta'    - Wall azimuth angle [rad] (default: 0)
%   'Phi'      - Wall polar angle [rad] (default: 0)
%   'Pz'       - Wall displacement [um] (default: 0)
%
% Output:
%   thermal_force_params - Simulink.Parameter with ThermalForceParamsBus
%
% Note:
%   This function follows the project naming convention:
%   function name = variable name = Bus base name
%
% Physical Background (Brownian Motion):
%   Einstein relation: D = k_B*T / gamma
%   Variance: <F_th * F_th> = 4*k_B*T*gamma / Ts
%
%   With Wall Effect (direction-dependent variance):
%     sigma_para = sqrt(variance_coeff / c_para)
%     sigma_perp = sqrt(variance_coeff / c_perp)
%
% Physical Constants:
%   k_B = 1.3806503e-5 pN*um/K (Boltzmann constant)
%   T = 310.15 K (37C, body temperature)
%   gamma_N = 0.0425 pN*s/um (Stokes drag for R=2.25um in water)
%   Ts = 1/1600 s (motion control sampling period)
%   variance_coeff = 4*k_B*T*gamma_N/Ts ~ 4.65 pN^2
%
% Example:
%   params = thermal_force_params('Enable', 1, 'Seed', 42);
%   F_th = calc_thermal_force_function([0;0;5], params.Value);
%
% See also: calc_thermal_force_function, particle_dynamics_params, CLAUDE.md

    %% Parse Input Parameters
    p = inputParser;
    addParameter(p, 'Enable', 1, @(x) ismember(x, [0, 1]));
    addParameter(p, 'kB', 1.3806503e-5, @(x) x > 0);  % pN*um/K
    addParameter(p, 'T', 310.15, @(x) x > 0);          % K (37C)
    addParameter(p, 'GammaN', 0.0425, @(x) x > 0);     % pN*s/um
    addParameter(p, 'Ts', 1/1600, @(x) x > 0);         % s
    addParameter(p, 'Seed', round(rand*1e6), @(x) x >= 0);
    addParameter(p, 'R', 2.25, @(x) x > 0);
    addParameter(p, 'Theta', 0, @isnumeric);
    addParameter(p, 'Phi', 0, @isnumeric);
    addParameter(p, 'Pz', 0, @isnumeric);
    parse(p, varargin{:});

    %% Calculate Wall Unit Vectors
    theta = p.Results.Theta;
    phi = p.Results.Phi;

    % Wall normal vector (pointing into fluid)
    w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];

    % Wall parallel vectors
    u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
    v_hat = [sin(theta); -cos(theta); 0];

    %% Build Parameter Structure
    % Order must match Bus element order: scalars first, then vectors
    params.enable = double(p.Results.Enable);
    params.k_B = p.Results.kB;
    params.T = p.Results.T;
    params.gamma_N = p.Results.GammaN;
    params.Ts = p.Results.Ts;
    params.seed = p.Results.Seed;
    params.R = p.Results.R;
    params.pz = p.Results.Pz;
    % Pre-calculate variance coefficient: 4*k_B*T*gamma_N/Ts (before vectors)
    params.variance_coeff = 4 * params.k_B * params.T * params.gamma_N / params.Ts;
    % Vector parameters
    params.w_hat = w_hat;
    params.u_hat = u_hat;
    params.v_hat = v_hat;

    %% Create Bus Object (ThermalForceParamsBus)
    ThermalForceParamsBus = Simulink.Bus;
    ThermalForceParamsBus.Description = 'Thermal Force Parameters';

    elems = [];

    % Scalar parameters
    scalar_names = {'enable', 'k_B', 'T', 'gamma_N', 'Ts', 'seed', ...
                    'R', 'pz', 'variance_coeff'};
    for i = 1:length(scalar_names)
        elem = Simulink.BusElement;
        elem.Name = scalar_names{i};
        elem.DataType = 'double';
        elem.Dimensions = 1;
        elem.Complexity = 'real';
        elems = [elems; elem]; %#ok<AGROW>
    end

    % Vector parameters (3x1)
    vector_names = {'w_hat', 'u_hat', 'v_hat'};
    for i = 1:length(vector_names)
        elem = Simulink.BusElement;
        elem.Name = vector_names{i};
        elem.DataType = 'double';
        elem.Dimensions = [3, 1];
        elem.Complexity = 'real';
        elems = [elems; elem]; %#ok<AGROW>
    end

    ThermalForceParamsBus.Elements = elems;

    % Assign Bus Object to base workspace
    assignin('base', 'ThermalForceParamsBus', ThermalForceParamsBus);

    %% Wrap as Simulink.Parameter
    thermal_force_params = Simulink.Parameter(params);
    thermal_force_params.DataType = 'Bus: ThermalForceParamsBus';

end
