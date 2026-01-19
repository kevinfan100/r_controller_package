function trajectory_generator_params = trajectory_generator_params(varargin)
% TRAJECTORY_GENERATOR_PARAMS Create parameters for trajectory generator
%
% Creates a Simulink.Parameter with TrajectoryGeneratorParamsBus type for use
% with the trajectory_generator_function in Simulink MATLAB Function blocks.
%
% Usage:
%   trajectory_generator_params = trajectory_generator_params()
%       Returns Simulink.Parameter with default values (z_sine)
%
%   trajectory_generator_params = trajectory_generator_params('Type', 1, ...)
%       Returns Simulink.Parameter with custom values
%
% Parameters (Name-Value pairs):
%   'Type'       - 0=z_sine, 1=xy_circle (default: 0)
%   'Amplitude'  - z_sine amplitude [um] (default: 5)
%   'Frequency'  - z_sine frequency [Hz] (default: 1)
%   'Radius'     - xy_circle radius [um] (default: 5)
%   'Period'     - xy_circle period [s] (default: 1)
%   'NCycles'    - Number of cycles (default: 3)
%
% Output:
%   trajectory_generator_params - Simulink.Parameter with TrajectoryGeneratorParamsBus
%
% Note:
%   This function follows the project naming convention:
%   function name = variable name = Bus base name
%
% Trajectory Definitions (Measuring Coordinate):
%   z_sine:
%     p_d = p0 + amplitude * sin(2*pi*frequency*t) * [0;0;1]
%     Duration: n_cycles / frequency
%
%   xy_circle:
%     p_d = p0 + radius*(cos(omega*t)-1)*[1;0;0] + radius*sin(omega*t)*[0;1;0]
%     Duration: period * n_cycles
%
% Example:
%   % Z-axis sine wave
%   params = trajectory_generator_params('Type', 0, 'Amplitude', 5, 'Frequency', 1);
%
%   % XY circle
%   params = trajectory_generator_params('Type', 1, 'Radius', 3, 'Period', 2);
%
% See also: trajectory_generator_function, motion_control_law_params, CLAUDE.md

    %% Parse Input Parameters
    p = inputParser;
    addParameter(p, 'Type', 0, @(x) ismember(x, [0, 1]));
    addParameter(p, 'Amplitude', 5, @(x) x > 0);
    addParameter(p, 'Frequency', 1, @(x) x > 0);
    addParameter(p, 'Radius', 5, @(x) x > 0);
    addParameter(p, 'Period', 1, @(x) x > 0);
    addParameter(p, 'NCycles', 3, @(x) x > 0);
    parse(p, varargin{:});

    %% Build Parameter Structure
    params.type = p.Results.Type;
    params.amplitude = p.Results.Amplitude;
    params.frequency = p.Results.Frequency;
    params.radius = p.Results.Radius;
    params.period = p.Results.Period;
    params.n_cycles = p.Results.NCycles;

    %% Create Bus Object (TrajectoryGeneratorParamsBus)
    TrajectoryGeneratorParamsBus = Simulink.Bus;
    TrajectoryGeneratorParamsBus.Description = 'Trajectory Generator Parameters';

    elem_names = {'type', 'amplitude', 'frequency', 'radius', 'period', 'n_cycles'};

    elems = Simulink.BusElement.empty(0, length(elem_names));
    for i = 1:length(elem_names)
        elem = Simulink.BusElement;
        elem.Name = elem_names{i};
        elem.DataType = 'double';
        elem.Dimensions = 1;
        elem.Complexity = 'real';
        elems(i) = elem;
    end
    TrajectoryGeneratorParamsBus.Elements = elems;

    % Assign Bus Object to base workspace
    assignin('base', 'TrajectoryGeneratorParamsBus', TrajectoryGeneratorParamsBus);

    %% Wrap as Simulink.Parameter
    trajectory_generator_params = Simulink.Parameter(params);
    trajectory_generator_params.DataType = 'Bus: TrajectoryGeneratorParamsBus';

end
