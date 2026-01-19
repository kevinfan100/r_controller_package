function motion_control_law_params = motion_control_law_params(varargin)
% MOTION_CONTROL_LAW_PARAMS Create parameters for motion control law
%
% Creates a Simulink.Parameter with MotionControlLawParamsBus type for use
% with the motion_control_law_function in Simulink MATLAB Function blocks.
%
% Usage:
%   motion_control_law_params = motion_control_law_params()
%       Returns Simulink.Parameter with default values
%
%   motion_control_law_params = motion_control_law_params('LambdaC', 0.7, ...)
%       Returns Simulink.Parameter with custom values
%
% Parameters (Name-Value pairs):
%   'Enable'            - 1=closed-loop, 0=open-loop (default: 1)
%   'LambdaC'           - Closed-loop pole, 0 < lambda_c < 1 (default: 0.7)
%   'Gamma'             - Stokes drag coefficient [pN*s/um] (default: 0.0425)
%   'Ts'                - Sampling period [s] (default: 1/1600)
%   'NoiseFilterEnable' - Enable low-pass filter on feedback (default: 0)
%   'NoiseFilterCutoff' - Filter cutoff frequency [Hz] (default: 5)
%   'DelayCompEnable'   - Enable d=2 delay compensation (default: 0)
%   'DelaySteps'        - Delay steps d for compensation (default: 2)
%
% Output:
%   motion_control_law_params - Simulink.Parameter with MotionControlLawParamsBus
%
% Note:
%   This function follows the project naming convention:
%   function name = variable name = Bus base name
%
% Control Law (d=0, no delay compensation):
%   f_d[k] = (gamma/Ts) * {p_d[k] - lambda_c*p_d[k-1] - (1-lambda_c)*p_feedback[k]}
%
% Control Law (d=2, with delay compensation - Paper Eq.17):
%   f_d[k] = (gamma/Ts) * {p_d[k] - lambda_c*p_d[k-1] - (1-lambda_c)*p_feedback[k]}
%            - (1-lambda_c) * {f_d[k-1] + f_d[k-2]}
%
% Physical Parameters:
%   gamma = 0.0425 pN*s/um for R=2.25um sphere in water at 37C
%   Ts = 1/1600 s (motion control sampling period)
%   lambda_c controls closed-loop bandwidth (smaller = faster response)
%
% IIR Low-Pass Filter:
%   y[k] = alpha*x[k] + (1-alpha)*y[k-1]
%   alpha = Ts / (Ts + 1/(2*pi*fc))
%
% Example:
%   % Default closed-loop control
%   params = motion_control_law_params();
%
%   % Custom pole location with noise filter
%   params = motion_control_law_params('LambdaC', 0.5, 'NoiseFilterEnable', 1);
%
% See also: motion_control_law_function, trajectory_generator_params, CLAUDE.md

    %% Parse Input Parameters
    p = inputParser;
    addParameter(p, 'Enable', 1, @(x) ismember(x, [0, 1]));
    addParameter(p, 'LambdaC', 0.7, @(x) x > 0 && x < 1);
    addParameter(p, 'Gamma', 0.0425, @(x) x > 0);
    addParameter(p, 'Ts', 1/1600, @(x) x > 0);
    addParameter(p, 'NoiseFilterEnable', 0, @(x) ismember(x, [0, 1]));
    addParameter(p, 'NoiseFilterCutoff', 5, @(x) x > 0);
    addParameter(p, 'DelayCompEnable', 0, @(x) ismember(x, [0, 1]));
    addParameter(p, 'DelaySteps', 2, @(x) x >= 0 && x == floor(x));
    parse(p, varargin{:});

    %% Build Parameter Structure
    % Note: Field order must match Bus Object element order exactly
    params.enable = double(p.Results.Enable);
    params.lambda_c = p.Results.LambdaC;
    params.gamma = p.Results.Gamma;
    params.Ts = p.Results.Ts;
    params.noise_filter_enable = double(p.Results.NoiseFilterEnable);
    params.noise_filter_cutoff = p.Results.NoiseFilterCutoff;

    % Pre-calculate IIR filter coefficient: alpha = Ts / (Ts + 1/(2*pi*fc))
    fc = p.Results.NoiseFilterCutoff;
    Ts = p.Results.Ts;
    params.filter_alpha = Ts / (Ts + 1/(2*pi*fc));

    % Delay compensation parameters (must come after filter_alpha for Bus order)
    params.delay_comp_enable = double(p.Results.DelayCompEnable);
    params.delay_steps = double(p.Results.DelaySteps);

    %% Create Bus Object (MotionControlLawParamsBus)
    MotionControlLawParamsBus = Simulink.Bus;
    MotionControlLawParamsBus.Description = 'Motion Control Law Parameters';

    elem_names = {'enable', 'lambda_c', 'gamma', 'Ts', ...
                  'noise_filter_enable', 'noise_filter_cutoff', 'filter_alpha', ...
                  'delay_comp_enable', 'delay_steps'};

    elems = Simulink.BusElement.empty(0, length(elem_names));
    for i = 1:length(elem_names)
        elem = Simulink.BusElement;
        elem.Name = elem_names{i};
        elem.DataType = 'double';
        elem.Dimensions = 1;
        elem.Complexity = 'real';
        elems(i) = elem;
    end
    MotionControlLawParamsBus.Elements = elems;

    % Assign Bus Object to base workspace
    assignin('base', 'MotionControlLawParamsBus', MotionControlLawParamsBus);

    %% Wrap as Simulink.Parameter
    motion_control_law_params = Simulink.Parameter(params);
    motion_control_law_params.DataType = 'Bus: MotionControlLawParamsBus';

end
