function p0 = calc_initial_position_function(params)
% CALC_INITIAL_POSITION_FUNCTION Calculate initial position from wall parameters
%
% Calculates the initial particle position based on wall configuration
% and desired initial distance from wall.
%
% Formula:
%   p0 = (pz + h_init) * w_hat
%
% Input:
%   params - ParticleDynamicsParamsBus or compatible struct with fields:
%            w_hat  - Wall normal vector (3x1)
%            pz     - Wall displacement [um]
%            h_init - Initial distance from wall [um]
%
% Output:
%   p0 - Initial position (3x1) [um], Measuring coordinate
%
% Physical Interpretation:
%   The particle starts at distance h_init from the wall,
%   positioned along the wall normal direction.
%
%   Default (theta=0, phi=0):
%     w_hat = [0; 0; 1]
%     pz = 0
%     h_init = 5 um
%     => p0 = [0; 0; 5] um
%
% Example:
%   params = particle_dynamics_params('HInit', 5);
%   p0 = calc_initial_position_function(params.Value);
%   % p0 = [0; 0; 5]
%
% See also: particle_dynamics_params, trajectory_generator_function, CLAUDE.md

    w_hat = params.w_hat;
    pz = params.pz;
    h_init = params.h_init;

    p0 = (pz + h_init) * w_hat;

    % Ensure column vector output
    p0 = p0(:);

end
