function vd_signal_params = vd_signal_params(varargin)
% VD_SIGNAL_PARAMS Create signal parameters for Vd_Signal_Generator block
%
% Creates a Simulink.Parameter with VdSignalParamsBus type for use
% with the vd_signal_function in Simulink MATLAB Function blocks.
%
% Usage:
%   vd_signal_params = vd_signal_params()
%       Returns Simulink.Parameter with default values
%
%   vd_signal_params = vd_signal_params('Mode', 1, 'Channel', 1, ...)
%       Returns Simulink.Parameter with custom values
%
% Parameters (Name-Value pairs):
%   'Mode'       - 1=Sine, 2=Step (default: 1)
%   'Channel'    - Excitation channel 1-6 (default: 1)
%   'Amplitude'  - Signal amplitude [V] (default: 1)
%   'Frequency'  - Sine frequency [Hz] (default: 100)
%   'Phase'      - Sine phase [deg] (default: 0)
%   'StepTime'   - Step transition time [s] (default: 0)
%   'Ts'         - Sampling time [s] (default: 1e-5)
%   'ff_preview' - Feedforward preview steps: 0 (no preview) or 2 (ZPETC) (default: 0)
%
% Output:
%   vd_signal_params - Simulink.Parameter with VdSignalParamsBus type
%
% Note:
%   This function follows the project naming convention:
%   function name = variable name = Bus base name
%
% Example:
%   % Sine wave test at 100 Hz on channel 1
%   params = vd_signal_params('Mode', 1, 'Channel', 1, 'Frequency', 100);
%
%   % Step response test on channel 3
%   params = vd_signal_params('Mode', 2, 'Channel', 3, 'StepTime', 0.01);
%
% See also: vd_signal_function, CLAUDE.md

    %% Parse Input Parameters
    p = inputParser;
    addParameter(p, 'Mode', 1, @(x) ismember(x, [1, 2]));
    addParameter(p, 'Channel', 1, @(x) x >= 1 && x <= 6);
    addParameter(p, 'Amplitude', 1, @isnumeric);
    addParameter(p, 'Frequency', 100, @(x) x > 0);
    addParameter(p, 'Phase', 0, @isnumeric);
    addParameter(p, 'StepTime', 0, @(x) x >= 0);
    addParameter(p, 'Ts', 1e-5, @(x) x > 0);
    addParameter(p, 'ff_preview', 0, @(x) ismember(x, [0, 2]));
    parse(p, varargin{:});

    %% Build Parameter Structure
    params.mode = p.Results.Mode;
    params.channel = p.Results.Channel;
    params.amplitude = p.Results.Amplitude;
    params.frequency = p.Results.Frequency;
    params.phase = p.Results.Phase;
    params.step_time = p.Results.StepTime;
    params.Ts = p.Results.Ts;
    params.ff_preview = p.Results.ff_preview;

    %% Create Bus Object (VdSignalParamsBus)
    VdSignalParamsBus = Simulink.Bus;
    VdSignalParamsBus.Description = 'Vd Signal Generator Parameters';

    % Define bus elements in the order they appear in the struct
    elem_names = {'mode', 'channel', 'amplitude', 'frequency', 'phase', ...
                  'step_time', 'Ts', 'ff_preview'};

    elems = Simulink.BusElement.empty(0, length(elem_names));
    for i = 1:length(elem_names)
        elem = Simulink.BusElement;
        elem.Name = elem_names{i};
        elem.DataType = 'double';
        elem.Dimensions = 1;
        elem.Complexity = 'real';
        elems(i) = elem;
    end
    VdSignalParamsBus.Elements = elems;

    % Assign Bus Object to base workspace
    assignin('base', 'VdSignalParamsBus', VdSignalParamsBus);

    %% Wrap as Simulink.Parameter
    vd_signal_params = Simulink.Parameter(params);
    vd_signal_params.DataType = 'Bus: VdSignalParamsBus';

end
