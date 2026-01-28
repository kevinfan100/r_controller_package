function f_d = motion_control_law_function(p_d, p_d_next, p_m, params)
% MOTION_CONTROL_LAW_FUNCTION Position-dependent discrete-time motion control
%
% Implements closed-loop position control for overdamped particle dynamics.
%
% Control Law (d=0, no delay compensation, no preview):
%   f_d[k] = (gamma/Ts) * {p_d[k] - lambda_c*p_d[k-1] - (1-lambda_c)*p_feedback[k]}
%
% Control Law (d=0, with trajectory preview - traj_preview_enable=1):
%   f_d[k] = (gamma/Ts) * {p_d[k+1] - lambda_c*p_d[k] - (1-lambda_c)*p_feedback[k]}
%   Note: p_d[k+1] is known since trajectory is given analytically
%
% Control Law (d=2, with delay compensation - Paper Eq.17):
%   f_d[k] = (gamma/Ts) * {p_d[k] - lambda_c*p_d[k-1] - (1-lambda_c)*p_feedback[k]}
%            - (1-lambda_c) * {f_d[k-1] + f_d[k-2]}
%
% When noise_filter_enable = 1:
%   p_feedback = p_m_filtered (IIR low-pass filtered position)
% When noise_filter_enable = 0:
%   p_feedback = p_m (raw measured position)
%
% Inputs:
%   p_d      - Desired position at t (3x1) [um], Measuring coordinate
%   p_d_next - Desired position at t+Ts (3x1) [um], for preview control
%   p_m      - Measured position (3x1) [um], Measuring coordinate
%   params   - MotionControlLawParamsBus
%
% Output:
%   f_d    - Desired force (3x1) [pN], Measuring coordinate
%
% Control Law Derivation:
%   Discrete-time overdamped Langevin equation:
%     p[k+1] = p[k] + (Ts/gamma) * f[k]
%
%   Closed-loop design with pole lambda_c (0 < lambda_c < 1):
%     p[k+1] - p_d[k+1] = lambda_c * (p[k] - p_d[k])
%
%   Solving for f_d[k]:
%     f_d[k] = (gamma/Ts) * {p_d[k+1] - lambda_c*p_d[k] - (1-lambda_c)*p[k]}
%
%   When trajectory is known analytically, we can use actual p_d[k+1]:
%     f_d[k] = (gamma/Ts) * {p_d[k+1] - lambda_c*p_d[k] - (1-lambda_c)*p_feedback[k]}
%
%   Otherwise, using p_d[k] to approximate p_d[k+1] (zero-order hold):
%     f_d[k] = (gamma/Ts) * {p_d[k] - lambda_c*p_d[k-1] - (1-lambda_c)*p_feedback[k]}
%
% Note:
%   - First call initializes p_d_prev = p_d (current input)
%   - Filter state updates even when disabled for smooth switching
%
% Example:
%   params = motion_control_law_params('LambdaC', 0.7, 'TrajPreviewEnable', 1);
%   f_d = motion_control_law_function([0;0;5], [0;0;5.01], [0;0;4.9], params.Value);
%
% See also: motion_control_law_params, trajectory_generator_function, CLAUDE.md

%#codegen

    persistent p_d_prev p_d_prev2 f_d_prev1 f_d_prev2 p_m_filtered_prev initialized

    % Initialize on first call
    if isempty(initialized)
        p_d_prev = p_d;
        p_d_prev2 = p_d;
        f_d_prev1 = zeros(3, 1);
        f_d_prev2 = zeros(3, 1);
        p_m_filtered_prev = p_m;
        initialized = true;
    end

    % Open-loop check
    if params.enable < 0.5
        f_d = zeros(3, 1);
        % Still update states for potential switch to closed-loop
        p_d_prev2 = p_d_prev;
        p_d_prev = p_d;
        f_d_prev2 = f_d_prev1;
        f_d_prev1 = f_d;
        p_m_filtered_prev = params.filter_alpha * p_m + (1 - params.filter_alpha) * p_m_filtered_prev;
        return;
    end

    % Extract parameters
    gamma = params.gamma;
    lambda_c = params.lambda_c;
    Ts = params.Ts;
    alpha = params.filter_alpha;

    % Single-stage IIR low-pass filter for p_m
    % y[k] = alpha*x[k] + (1-alpha)*y[k-1]
    p_m_filtered = alpha * p_m + (1 - alpha) * p_m_filtered_prev;

    % Select feedback source
    if params.noise_filter_enable > 0.5
        p_feedback = p_m_filtered;
    else
        p_feedback = p_m;
    end

    % Pre-compute common terms
    one_minus_lambda = 1 - lambda_c;

    % =========================================================================
    % Control Law Selection
    % =========================================================================

    % ---------------------------------------------------------------------
    % Trajectory Term: Select between preview and ZOH approximation
    % ---------------------------------------------------------------------
    if params.traj_preview_enable > 0.5
        % Trajectory Preview: Use p_d[k+1] (known analytically)
        % delta_p_d = p_d[k+1] - lambda_c * p_d[k]
        delta_p_d = p_d_next - lambda_c * p_d;
    else
        % ZOH Approximation: Approximate p_d[k+1] with p_d[k]
        % delta_p_d = p_d[k] - lambda_c * p_d[k-1]
        delta_p_d = p_d - lambda_c * p_d_prev;
    end

    % ---------------------------------------------------------------------
    % Control Law with optional delay compensation
    % ---------------------------------------------------------------------
    if params.delay_comp_enable > 0.5
        % Delay Compensation Control Law (d=2) - Paper Eq.17
        % f_d[k] = (gamma/Ts) * {delta_p_d - (1-lambda_c)*p_feedback[k]}
        %          - (1-lambda_c) * {f_d[k-1] + f_d[k-2]}
        f_d = (gamma / Ts) * (delta_p_d - one_minus_lambda * p_feedback) ...
              - one_minus_lambda * (f_d_prev1 + f_d_prev2);
    else
        % Standard Control Law (d=0) - No delay compensation
        f_d = (gamma / Ts) * (delta_p_d - one_minus_lambda * p_feedback);
    end

    % =========================================================================
    % Update States for Next Iteration
    % =========================================================================
    p_d_prev2 = p_d_prev;           % p_d[k-2] <- p_d[k-1]
    p_d_prev = p_d;                 % p_d[k-1] <- p_d[k]
    f_d_prev2 = f_d_prev1;          % f_d[k-2] <- f_d[k-1]
    f_d_prev1 = f_d;                % f_d[k-1] <- f_d[k]
    p_m_filtered_prev = p_m_filtered;

end
