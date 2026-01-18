function pi_ctrl_params = pi_ctrl_params(Kp, Ki, varargin)
% PI_CTRL_PARAMS PI Controller parameter calculator with Bus Object creation
%
% Calculate all controller coefficients for discrete-time PI control
% using Tustin (bilinear) discretization method.
%
% Inputs:
%   Kp - Proportional gain
%   Ki - Integral gain [1/s] (or compute from Kp * zc)
%
% Optional Parameters (Name-Value pairs):
%   'Ts' - Sampling time [s] (default: 1e-5 = 100 kHz)
%
% Output:
%   pi_ctrl_params - Simulink.Parameter object with Bus type
%
% Naming Convention (see CLAUDE.md):
%   Function name = Variable name = Bus Object base = Simulink Workspace
%   pi_ctrl_params() -> pi_ctrl_params -> PICtrlParamsBus -> pi_ctrl_params
%
% Example:
%   % Direct Kp, Ki specification
%   params = pi_ctrl_params(2, 4412);
%
%   % From bandwidth specification (zc = 2206 rad/s)
%   Kp = 2; zc = 2206;
%   params = pi_ctrl_params(Kp, Kp * zc);
%
% Note: This function creates 'PICtrlParamsBus' in base workspace
%
% See also: pi_ctrl_function, model_base_ctrl_params, CLAUDE.md

    %% Parameter Parsing
    p = inputParser;
    addParameter(p, 'Ts', 1e-5);
    parse(p, varargin{:});

    Ts = p.Results.Ts;

    %% System Constants
    params.Kp = Kp;
    params.Ki = Ki;
    params.Ts = Ts;

    % Pre-computed coefficient for Tustin integration
    % i[k] = i[k-1] + (Ki * Ts / 2) * (e[k] + e[k-1])
    params.Ki_Ts_half = Ki * Ts / 2;

    % Coupling matrix B and its inverse (same as R-Controller)
    B = [0.2365  -0.0064  -0.0327  -0.0344  -0.0408  -0.0343;
        -0.0037   0.2818  -0.0427  -0.0675  -0.0779  -0.0368;
        -0.0375  -0.0328   0.2108  -0.0060  -0.0265  -0.0341;
        -0.0245  -0.0777  -0.0056   0.2361  -0.0770  -0.0241;
        -0.0413  -0.0760  -0.0234  -0.0720   0.2572  -0.0045;
        -0.0244  -0.0330  -0.0257  -0.0245  -0.0030   0.1845];
    params.B_inv = inv(B);

    %% Create Simulink Bus Object
    % Simulink requires explicit Bus Object definition for structures
    PICtrlParamsBus = Simulink.Bus;
    PICtrlParamsBus.Description = 'PI Controller Parameters Structure';

    % Define all bus elements
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'Kp';
    elems(1).DataType = 'double';

    elems(2) = Simulink.BusElement;
    elems(2).Name = 'Ki';
    elems(2).DataType = 'double';

    elems(3) = Simulink.BusElement;
    elems(3).Name = 'Ts';
    elems(3).DataType = 'double';

    elems(4) = Simulink.BusElement;
    elems(4).Name = 'Ki_Ts_half';
    elems(4).DataType = 'double';

    elems(5) = Simulink.BusElement;
    elems(5).Name = 'B_inv';
    elems(5).Dimensions = [6 6];
    elems(5).DataType = 'double';

    % Assign elements to bus
    PICtrlParamsBus.Elements = elems;

    % Save PICtrlParamsBus to base workspace
    assignin('base', 'PICtrlParamsBus', PICtrlParamsBus);

    %% Wrap as Simulink.Parameter with Bus Type
    params_data = params;  % Keep the raw data
    pi_ctrl_params = Simulink.Parameter(params_data);
    pi_ctrl_params.DataType = 'Bus: PICtrlParamsBus';
    pi_ctrl_params.Description = 'PI Controller Parameters';

end
