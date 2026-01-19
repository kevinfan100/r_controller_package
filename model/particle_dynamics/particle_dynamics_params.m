function particle_dynamics_params = particle_dynamics_params(varargin)
% PARTICLE_DYNAMICS_PARAMS Create parameters for particle dynamics
%
% Creates a Simulink.Parameter with ParticleDynamicsParamsBus type for use
% with calc_gamma_inv_function in Simulink MATLAB Function blocks.
%
% Usage:
%   particle_dynamics_params = particle_dynamics_params()
%       Returns Simulink.Parameter with default values
%
%   particle_dynamics_params = particle_dynamics_params('R', 2.5, ...)
%       Returns Simulink.Parameter with custom values
%
% Parameters (Name-Value pairs):
%   'R'               - Particle radius [um] (default: 2.25)
%   'GammaN'          - Stokes drag coefficient [pN*s/um] (default: 0.0425)
%   'WallEffectEnable'- Enable Wall Effect (default: 1)
%   'Theta'           - Wall azimuth angle [rad] (default: 0)
%   'Phi'             - Wall polar angle [rad] (default: 0)
%   'Pz'              - Wall displacement along w_hat [um] (default: 0)
%   'HMin'            - Minimum safe distance from wall [um] (default: 3.375)
%   'HInit'           - Initial distance from wall [um] (default: 5)
%
% Output:
%   particle_dynamics_params - Simulink.Parameter with ParticleDynamicsParamsBus
%
% Note:
%   This function follows the project naming convention:
%   function name = variable name = Bus base name
%
% Wall Coordinate System (theta=0, phi=0 default):
%   w_hat = [0; 0; 1]   - Wall normal (pointing into fluid, +Z)
%   u_hat = [-1; 0; 0]  - Wall parallel vector 1 (-X)
%   v_hat = [0; -1; 0]  - Wall parallel vector 2 (-Y)
%
%   This means:
%   - z_sine trajectory: perpendicular to wall (along w_hat)
%   - xy_circle trajectory: parallel to wall (in u_hat-v_hat plane)
%
% Physical Parameters:
%   R = 2.25 um (typical magnetic bead radius)
%   gamma_N = 6*pi*eta*R = 0.0425 pN*s/um for water at 37C
%   h_min = 1.5*R = 3.375 um (safety distance)
%
% Example:
%   params = particle_dynamics_params('WallEffectEnable', 1, 'HInit', 5);
%   p0 = calc_initial_position_function(params.Value);
%
% See also: calc_gamma_inv_function, thermal_force_params, CLAUDE.md

    %% Parse Input Parameters
    p = inputParser;
    addParameter(p, 'R', 2.25, @(x) x > 0);
    addParameter(p, 'GammaN', 0.0425, @(x) x > 0);
    addParameter(p, 'WallEffectEnable', 1, @(x) ismember(x, [0, 1]));
    addParameter(p, 'Theta', 0, @isnumeric);
    addParameter(p, 'Phi', 0, @isnumeric);
    addParameter(p, 'Pz', 0, @isnumeric);
    addParameter(p, 'HMin', 3.375, @(x) x > 0);  % 1.5 * R default
    addParameter(p, 'HInit', 5, @(x) x > 0);
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
    params.R = p.Results.R;
    params.gamma_N = p.Results.GammaN;
    params.wall_effect_enable = double(p.Results.WallEffectEnable);
    params.pz = p.Results.Pz;
    params.h_min = p.Results.HMin;
    params.h_init = p.Results.HInit;
    params.w_hat = w_hat;
    params.u_hat = u_hat;
    params.v_hat = v_hat;

    %% Create Bus Object (ParticleDynamicsParamsBus)
    ParticleDynamicsParamsBus = Simulink.Bus;
    ParticleDynamicsParamsBus.Description = 'Particle Dynamics Parameters';

    elems = [];

    % Scalar parameters
    scalar_names = {'R', 'gamma_N', 'wall_effect_enable', 'pz', 'h_min', 'h_init'};
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

    ParticleDynamicsParamsBus.Elements = elems;

    % Assign Bus Object to base workspace
    assignin('base', 'ParticleDynamicsParamsBus', ParticleDynamicsParamsBus);

    %% Wrap as Simulink.Parameter
    particle_dynamics_params = Simulink.Parameter(params);
    particle_dynamics_params.DataType = 'Bus: ParticleDynamicsParamsBus';

end
