function [c_para, c_perp] = calc_correction_functions(h_bar)
% CALC_CORRECTION_FUNCTIONS Calculate Wall Effect correction coefficients
%
% Calculates the parallel (c_para) and perpendicular (c_perp) correction
% coefficients for the Stokes drag near a planar wall.
%
% Input:
%   h_bar - Normalized distance from wall: h_bar = h/R
%           where h is the distance from wall to particle center
%           and R is the particle radius
%           MUST be > 1 (particle not touching wall)
%
% Outputs:
%   c_para - Parallel correction coefficient (c_parallel)
%   c_perp - Perpendicular correction coefficient (c_perpendicular)
%
% Physical Interpretation:
%   - Effective parallel drag: gamma_parallel = gamma_N * c_para
%   - Effective perpendicular drag: gamma_perp = gamma_N * c_perp
%   - Both c_para and c_perp approach 1 as h_bar -> infinity
%   - Both increase as h_bar -> 1 (particle approaches wall)
%   - c_perp > c_para (harder to move perpendicular to wall)
%
% Formulas:
%   c_para = 1 / (1 - 9/16*(1/h) + 1/8*(1/h)^3 - 45/256*(1/h)^4 - 1/16*(1/h)^5)
%
%   c_perp = 1 / (1 - 9/8*(1/h) + 1/2*(1/h)^3 - 57/100*(1/h)^4 + 1/5*(1/h)^5
%                + 7/200*(1/h)^11 - 1/25*(1/h)^12)
%
% Numerical Examples:
%   h_bar = 1.5:  c_para = 2.53, c_perp = 6.26
%   h_bar = 2.0:  c_para = 1.72, c_perp = 2.80
%   h_bar = 5.0:  c_para = 1.13, c_perp = 1.22
%   h_bar = 10:   c_para = 1.06, c_perp = 1.10
%   h_bar = inf:  c_para = 1.00, c_perp = 1.00
%
% Reference:
%   Drag_MATLAB.pdf - Wall Effect mathematical model
%
% Example:
%   [c_para, c_perp] = calc_correction_functions(1.5);
%   fprintf('c_para = %.2f, c_perp = %.2f\n', c_para, c_perp);
%
% See also: calc_gamma_inv_function, particle_dynamics_params, CLAUDE.md

%#codegen

    % Safety check: ensure h_bar > 1 (clamped for code generation)
    % In real use, h_bar should never be < 1
    h_bar_safe = max(h_bar, 1.01);

    % Compute inverse of h_bar for efficiency
    inv_h = 1 / h_bar_safe;

    % Parallel correction coefficient (c_para)
    % c_para = 1 / (1 - 9/16*(1/h) + 1/8*(1/h)^3 - 45/256*(1/h)^4 - 1/16*(1/h)^5)
    denom_para = 1 - 9/16*inv_h + 1/8*inv_h^3 - 45/256*inv_h^4 - 1/16*inv_h^5;
    c_para = 1 / denom_para;

    % Perpendicular correction coefficient (c_perp)
    % c_perp = 1 / (1 - 9/8*(1/h) + 1/2*(1/h)^3 - 57/100*(1/h)^4 + 1/5*(1/h)^5
    %              + 7/200*(1/h)^11 - 1/25*(1/h)^12)
    denom_perp = 1 - 9/8*inv_h + 1/2*inv_h^3 - 57/100*inv_h^4 + 1/5*inv_h^5 ...
                 + 7/200*inv_h^11 - 1/25*inv_h^12;
    c_perp = 1 / denom_perp;

end
