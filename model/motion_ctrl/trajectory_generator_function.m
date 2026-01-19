function p_d = trajectory_generator_function(t, p0, params)
% TRAJECTORY_GENERATOR_FUNCTION Generate desired trajectory position
%
% Calculates the desired trajectory position based on current time,
% starting position, and trajectory parameters.
%
% Inputs:
%   t      - Current time [s]
%   p0     - Initial position (3x1) [um], Measuring coordinate
%   params - TrajectoryGeneratorParamsBus
%
% Output:
%   p_d    - Desired position (3x1) [um], Measuring coordinate
%
% Trajectory types:
%   type=0 (z_sine):
%     p_d = p0 + amplitude * sin(2*pi*frequency*t) * [0; 0; 1]
%     Duration: n_cycles / frequency
%     Returns to p0 after completion
%
%   type=1 (xy_circle):
%     p_d = p0 + radius*(cos(omega*t)-1)*[1;0;0] + radius*sin(omega*t)*[0;1;0]
%     Duration: period * n_cycles
%     Returns to p0 after completion
%
% Note:
%   - Measuring coordinate system used throughout
%   - p0 is pre-calculated initial position passed as input
%   - Trajectory returns to p0 after completion for smooth ending
%
% Example:
%   params = trajectory_generator_params('Type', 0, 'Amplitude', 5);
%   p0 = [0; 0; 5];
%   p_d = trajectory_generator_function(0.5, p0, params.Value);
%
% See also: trajectory_generator_params, motion_control_law_function, CLAUDE.md

%#codegen

    if params.type < 0.5  % z_sine
        p_d = trajectory_z_sine(t, p0, params);
    else  % xy_circle
        p_d = trajectory_xy_circle(t, p0, params);
    end

end


% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function p_d = trajectory_z_sine(t, p0, params)
% TRAJECTORY_Z_SINE Sinusoidal motion along Z axis (perpendicular to default wall)

    % Z axis direction in Measuring coordinate
    z_hat = [0; 0; 1];

    amplitude = params.amplitude;
    frequency = params.frequency;
    n_cycles = params.n_cycles;

    % Total trajectory time
    T_total = n_cycles / frequency;

    if t <= T_total
        % Angular frequency
        omega = 2 * pi * frequency;

        % Sinusoidal displacement along Z axis
        displacement = amplitude * sin(omega * t);
        p_d = p0 + displacement * z_hat;
    else
        % After completing all cycles, return to starting position
        p_d = p0;
    end

end


function p_d = trajectory_xy_circle(t, p0, params)
% TRAJECTORY_XY_CIRCLE Circular motion in XY plane (parallel to default wall)

    % XY plane directions in Measuring coordinate
    x_hat = [1; 0; 0];
    y_hat = [0; 1; 0];

    radius = params.radius;
    period = params.period;
    n_cycles = params.n_cycles;

    % Total trajectory time
    T_total = period * n_cycles;

    if t <= T_total
        % Angular frequency
        omega = 2 * pi / period;

        % Circular motion in X-Y plane
        % (cos(omega*t) - 1) ensures p_d(0) = p0
        p_d = p0 + radius * (cos(omega * t) - 1) * x_hat ...
                 + radius * sin(omega * t) * y_hat;
    else
        % After completing all cycles, return to starting position
        p_d = p0;
    end

end
