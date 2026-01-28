function [p_d, p_d_next] = trajectory_generator_function(t, p0, traj_params, particle_params)
% TRAJECTORY_GENERATOR_FUNCTION Generate desired trajectory position
%
% Calculates the desired trajectory position based on current time,
% starting position, and trajectory parameters. Also computes p_d_next
% for trajectory preview control.
%
% Inputs:
%   t               - Current time [s]
%   p0              - Initial position (3x1) [um], Measuring coordinate
%   traj_params     - TrajectoryGeneratorParamsBus
%   particle_params - ParticleDynamicsParamsBus (provides w_hat for z_sine)
%
% Outputs:
%   p_d      - Desired position at t (3x1) [um], Measuring coordinate
%   p_d_next - Desired position at t+Ts (3x1) [um], for preview control
%
% Trajectory types:
%   type=0 (z_sine):
%     p_d = p0 + amplitude * sin(2*pi*frequency*t) * w_hat
%     Duration: n_cycles / frequency
%     w_hat is wall normal from particle_params
%     Returns to p0 after completion
%
%   type=1 (xy_circle):
%     p_d = p0 + radius*(cos(omega*t)-1)*[1;0;0] + radius*sin(omega*t)*[0;1;0]
%     Duration: period * n_cycles
%     Returns to p0 after completion
%
%   type=2 (positioning):
%     p_d = p0 (constant)
%     Duration: hold_time
%     Use case: Test disturbance rejection (thermal force)
%
% Note:
%   - Measuring coordinate system used throughout
%   - p0 is pre-calculated initial position passed as input
%   - Trajectory returns to p0 after completion for smooth ending
%   - z_sine uses w_hat from particle_params to align with wall normal
%
% Example:
%   traj_params = trajectory_generator_params('Type', 0, 'Amplitude', 5);
%   particle_params = particle_dynamics_params('HInit', 5);
%   p0 = [0; 0; 5];
%   p_d = trajectory_generator_function(0.5, p0, traj_params.Value, particle_params.Value);
%
% See also: trajectory_generator_params, particle_dynamics_params, CLAUDE.md

%#codegen

    % Motion control sampling period
    Ts_motion = 1/1600;

    % Calculate p_d at current time t
    if traj_params.type < 0.5  % z_sine
        p_d = trajectory_z_sine(t, p0, traj_params, particle_params);
    elseif traj_params.type < 1.5  % xy_circle
        p_d = trajectory_xy_circle(t, p0, traj_params);
    else  % positioning
        p_d = trajectory_positioning(t, p0, traj_params);
    end

    % Calculate p_d_next at time t + Ts_motion (for trajectory preview)
    t_next = t + Ts_motion;
    if traj_params.type < 0.5  % z_sine
        p_d_next = trajectory_z_sine(t_next, p0, traj_params, particle_params);
    elseif traj_params.type < 1.5  % xy_circle
        p_d_next = trajectory_xy_circle(t_next, p0, traj_params);
    else  % positioning
        p_d_next = trajectory_positioning(t_next, p0, traj_params);
    end

end


% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function p_d = trajectory_z_sine(t, p0, traj_params, particle_params)
% TRAJECTORY_Z_SINE Sinusoidal motion along wall normal (perpendicular to wall)

    % Wall normal direction from particle parameters
    w_hat = particle_params.w_hat;

    amplitude = traj_params.amplitude;
    frequency = traj_params.frequency;
    n_cycles = traj_params.n_cycles;

    % Total trajectory time
    T_total = n_cycles / frequency;

    if t <= T_total
        % Angular frequency
        omega = 2 * pi * frequency;

        % Sinusoidal displacement along wall normal
        displacement = amplitude * sin(omega * t);
        p_d = p0 + displacement * w_hat;
    else
        % After completing all cycles, return to starting position
        p_d = p0;
    end

end


function p_d = trajectory_xy_circle(t, p0, traj_params)
% TRAJECTORY_XY_CIRCLE Circular motion in XY plane (parallel to default wall)

    % XY plane directions in Measuring coordinate
    x_hat = [1; 0; 0];
    y_hat = [0; 1; 0];

    radius = traj_params.radius;
    period = traj_params.period;
    n_cycles = traj_params.n_cycles;

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


function p_d = trajectory_positioning(t, p0, traj_params) %#ok<INUSD>
% TRAJECTORY_POSITIONING Hold position at p0 (constant trajectory)
%
% Use case: Test disturbance rejection capability under thermal force

    % Always return initial position
    p_d = p0;

end
