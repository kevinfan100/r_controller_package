function v_d = vd_signal_function(signal_type, t, params)
% VD_SIGNAL_FUNCTION Generate desired voltage for Signal mode (Sine/Step)
%
% Generates v_d (desired Hall sensor voltage) for inner loop testing.
% This function is designed to be called from a Simulink MATLAB Function block.
%
% Inputs:
%   signal_type - 1 for this function to be active (Signal mode)
%   t           - Current time [s]
%   params      - Signal parameters (Bus: VdSignalParamsBus)
%       .mode       - 1=Sine, 2=Step
%       .channel    - Excitation channel (1-6)
%       .amplitude  - Signal amplitude [V]
%       .frequency  - Sine frequency [Hz]
%       .phase      - Sine phase [deg]
%       .step_time  - Step transition time [s]
%       .Ts         - Sampling time [s]
%       .d          - Preview steps
%
% Output:
%   v_d         - Desired Hall voltage (6x1) [V]
%
% Example:
%   params = vd_signal_params('Mode', 1, 'Channel', 1, 'Frequency', 100);
%   v_d = vd_signal_function(1, 0.01, params.Value);
%
% See also: vd_signal_params, CLAUDE.md

%#codegen

    %% Compute future time (preview)
    t_future = t + params.Ts * params.d;

    %% Initialize output
    v_d = zeros(6, 1);

    %% Clamp channel to valid range
    channel = max(1, min(6, round(params.channel)));

    %% Generate signal based on mode
    if params.mode == 1
        % Sine Wave mode
        phase_rad = params.phase * (pi / 180);  % deg2rad for codegen
        v_d(channel) = params.amplitude * sin(2*pi*params.frequency*t_future + phase_rad);
    else
        % Step mode
        if t_future >= params.step_time
            v_d(channel) = params.amplitude;
        else
            v_d(channel) = 0;
        end
    end

end
