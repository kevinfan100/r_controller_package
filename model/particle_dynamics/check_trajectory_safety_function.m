function [is_safe, h_min_actual, t_critical] = check_trajectory_safety_function(p0, traj_params, particle_params, T_sim, Ts)
% CHECK_TRAJECTORY_SAFETY_FUNCTION Verify trajectory maintains safe distance from wall
%
% Checks if a trajectory will bring the particle too close to the wall.
%
% Safety criterion:
%   h(t) >= h_min for all t in [0, T_sim]
%
% Inputs:
%   p0              - Initial position (3x1) [um]
%   traj_params     - TrajectoryGeneratorParamsBus or value struct
%   particle_params - ParticleDynamicsParamsBus or value struct
%   T_sim           - Total simulation time [s]
%   Ts              - Time step for checking [s] (default: 1e-3)
%
% Outputs:
%   is_safe      - true if trajectory is safe
%   h_min_actual - Minimum h along trajectory [um]
%   t_critical   - Time at minimum h [s]
%
% Safety Parameters:
%   h_min = 1.5 * R = 3.375 um (default, corresponds to h_bar_min = 1.5)
%   At h_bar = 1.5: c_para = 2.53, c_perp = 6.26
%
% Usage:
%   traj_params = trajectory_generator_params('Amplitude', 5);
%   particle_params = particle_dynamics_params('HMin', 3.375);
%   p0 = calc_initial_position_function(particle_params.Value);
%   [is_safe, h_min, t_crit] = check_trajectory_safety_function(...
%       p0, traj_params.Value, particle_params.Value, 5, 1e-3);
%
% Example (unsafe trajectory):
%   % h_init = 5, amplitude = 10 => may go below h_min
%   traj_params = trajectory_generator_params('Amplitude', 10);
%   particle_params = particle_dynamics_params('HInit', 5, 'HMin', 3.375);
%   p0 = calc_initial_position_function(particle_params.Value);
%   [is_safe, h_min, ~] = check_trajectory_safety_function(...
%       p0, traj_params.Value, particle_params.Value, 5);
%   % is_safe = false, h_min < 0 (would penetrate wall)
%
% See also: trajectory_generator_function, particle_dynamics_params, CLAUDE.md

    if nargin < 5
        Ts = 1e-3;  % Default check interval: 1 ms
    end

    % Extract wall parameters
    w_hat = particle_params.w_hat;
    pz = particle_params.pz;
    h_min_threshold = particle_params.h_min;

    % Time vector for checking
    t_check = 0:Ts:T_sim;
    N = length(t_check);

    % Calculate h along trajectory
    h = zeros(1, N);
    for i = 1:N
        p_d = trajectory_generator_function(t_check(i), p0, traj_params);
        h(i) = dot(p_d, w_hat) - pz;
    end

    % Find minimum h
    [h_min_actual, idx] = min(h);
    t_critical = t_check(idx);

    % Safety check
    is_safe = h_min_actual >= h_min_threshold;

    % Display warning if unsafe
    if ~is_safe
        warning('check_trajectory_safety:unsafeTrajectory', ...
            'Trajectory unsafe! Min h = %.2f um at t = %.3f s (threshold: %.2f um)', ...
            h_min_actual, t_critical, h_min_threshold);
    end

end
