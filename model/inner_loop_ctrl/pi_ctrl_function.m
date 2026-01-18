function [u, e] = pi_ctrl_function(vd, vm, params)
% PI_CTRL_FUNCTION Discrete-time PI Controller (Tustin discretization)
%
% Implements PI Controller using Tustin (bilinear/trapezoidal) method
% with pre-calculated parameters from pi_ctrl_params().
%
% Continuous-time PI:
%   C(s) = Kp + Ki/s
%
% Tustin discretization (s -> 2(z-1)/(Ts(z+1))):
%   p[k] = Kp * e[k]                                    (proportional)
%   i[k] = i[k-1] + (Ki*Ts/2) * (e[k] + e[k-1])        (integral, trapezoidal)
%   uc[k] = p[k] + i[k]                                 (control law)
%   u[k] = B^-1 * uc[k]                                 (decoupling)
%
% Inputs:
%   vd     - Desired voltage (6x1) [V]
%   vm     - Measured voltage (6x1) [V]
%   params - Controller parameters from pi_ctrl_params()
%
% Outputs:
%   u - Control output (6x1) [A]
%   e - Error signal (6x1) [V]
%
% See also: pi_ctrl_params, model_base_ctrl_function, CLAUDE.md

    %% Persistent State Variables
    persistent e_k1         % Previous error (6x1)
    persistent i_k          % Integral state (6x1)
    persistent initialized

    %% Initialization
    if isempty(initialized)
        initialized = true;
        e_k1 = zeros(6, 1);
        i_k = zeros(6, 1);
    end

    %% Error Calculation
    e = vd - vm;

    %% Proportional Term
    p_k = params.Kp * e;

    %% Integral Term (Tustin trapezoidal rule)
    % i[k] = i[k-1] + (Ki * Ts / 2) * (e[k] + e[k-1])
    i_k = i_k + params.Ki_Ts_half * (e + e_k1);

    %% Control Law
    uc = p_k + i_k;

    %% Decoupling (Coupling matrix inverse)
    u = params.B_inv * uc;

    %% State Update
    e_k1 = e;

end
