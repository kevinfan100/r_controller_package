function lpf_params = lpf_system_params(f_low, varargin)
% LPF_SYSTEM_PARAMS Calculate 3rd-order system parameters with LPF
%
% Computes Z-domain parameters for the combined system H_model(s) × H_lpf(s)
% using zero-order hold (ZOH) discretization.
%
% Inputs:
%   f_low - LPF cutoff frequency [Hz]
%
% Optional Parameters (Name-Value pairs):
%   'Ts' - Sampling time [s] (default: 1e-5)
%
% Output:
%   lpf_params - Structure containing:
%       k_o_lpf  - 3rd-order system gain
%       bu       - Non-minimum phase zero coefficient (≈3.16 for f_low=10kHz)
%       ba       - Minimum phase zero coefficient (≈0.22 for f_low=10kHz)
%       p3       - LPF pole coefficient (≈0.53 for f_low=10kHz)
%       a1       - Pole coefficient 1 (same as 2nd-order)
%       a2       - Pole coefficient 2 (same as 2nd-order)
%       a3       - Pole coefficient 3 (from LPF)
%
% Reference:
%   docs/lpf_transfer_function.pdf - Z-domain coefficients for f_low=10kHz
%
% Example:
%   lpf_params = lpf_system_params(10000);
%   % Returns bu≈3.1629, ba≈0.2235, p3≈0.5335, k_o_lpf≈1.0273e-4
%
% See also: model_base_ctrl_params, CLAUDE.md

    %% Parse Optional Parameters
    p = inputParser;
    addParameter(p, 'Ts', 1e-5, @(x) isnumeric(x) && x > 0);
    parse(p, varargin{:});

    Ts = p.Results.Ts;

    %% Validate Input
    if f_low <= 0
        error('lpf_system_params:InvalidInput', 'f_low must be positive');
    end

    % Warning for unusual f_low values
    if f_low < 1000 || f_low > 50000
        warning('lpf_system_params:UnusualFrequency', ...
            'f_low=%.0f Hz is outside recommended range [1000, 50000] Hz', f_low);
    end

    %% S-Domain Model Parameters (from existing system)
    % H_model(s) = 1.1592e7 / (s^2 + 6.6172e3*s + 1.1592e7)
    wn_model = sqrt(1.1592e7);           % Natural frequency [rad/s]
    zeta_model = 6.6172e3 / (2*wn_model); % Damping ratio
    k_model = 1.1592e7;                   % DC gain numerator

    %% S-Domain LPF Parameters
    w_low = 2*pi*f_low;  % LPF cutoff angular frequency [rad/s]

    %% Build S-Domain Transfer Functions
    % H_model(s): 2nd-order plant model
    num_model = k_model;
    den_model = [1, 2*zeta_model*wn_model, wn_model^2];
    H_model_s = tf(num_model, den_model);

    % H_lpf(s): 1st-order low-pass filter
    num_lpf = w_low;
    den_lpf = [1, w_low];
    H_lpf_s = tf(num_lpf, den_lpf);

    % Combined system: H(s) = H_model(s) × H_lpf(s)
    H_combined_s = H_model_s * H_lpf_s;

    %% Convert to Z-Domain using ZOH
    H_combined_z = c2d(H_combined_s, Ts, 'zoh');

    %% Extract Z-Domain Coefficients
    [num_z, den_z] = tfdata(H_combined_z, 'v');

    % Normalize denominator (should already be normalized, but ensure)
    den_z = den_z / den_z(1);

    % Extract zero polynomial coefficients
    % Z-domain: H(z^-1) = z^-1 * k_o_lpf * (1 + bu*z^-1)(1 + ba*z^-1) / ...
    %                     [(1 - p1*z^-1 + p2*z^-2)(1 - p3*z^-1)]

    % Find zeros and poles
    zeros_z = roots(num_z);
    poles_z = roots(den_z);

    % Sort zeros by magnitude (non-minimum phase zero has |z| > 1)
    zeros_sorted = sort(zeros_z, 'descend', 'ComparisonMethod', 'abs');

    % Convert zeros from z-domain to z^-1 coefficients
    % Zero at z=z0 means factor (z - z0) = z(1 - z0*z^-1)
    % In z^-1 form: (1 + b*z^-1) where b = -z0
    bu = -zeros_sorted(1);  % Non-minimum phase zero (|z0| > 1, so |bu| > 1)
    ba = -zeros_sorted(2);  % Minimum phase zero (|z0| < 1, so |ba| < 1)

    % Ensure bu is the non-minimum phase zero (larger magnitude)
    if abs(bu) < abs(ba)
        temp = bu;
        bu = ba;
        ba = temp;
    end

    % Extract gain
    % The gain k_o_lpf is num_z(2) / ((1+bu)(1+ba)) considering unit delay
    % Or directly: k_o_lpf = num_z(2) / prod(1 - zeros_z) but need careful handling
    k_o_lpf = num_z(2);  % Coefficient of z^-1 term in numerator

    % Verify by computing expected DC gain
    % At DC (z=1): H(1) should equal H_s(0) = 1 (unity DC gain for normalized system)
    dc_gain_z = sum(num_z) / sum(den_z);

    %% Extract Pole Coefficients
    % Denominator: (1 - 1.9348*z^-1 + 0.9360*z^-2)(1 - p3*z^-1)
    % = 1 - (1.9348 + p3)*z^-1 + (0.9360 + 1.9348*p3)*z^-2 - 0.9360*p3*z^-3

    % From den_z = [1, -a1_total, a2_total, -a3_total] (in z^-1 form)
    % Actually den_z is in z form: [1, d1, d2, d3] where d1 = coefficient of z^2, etc.

    % Sort poles: complex conjugate pair (from H_model) and real pole (from H_lpf)
    poles_real = poles_z(imag(poles_z) == 0);
    poles_complex = poles_z(imag(poles_z) ~= 0);

    % If all poles appear real (numerical), identify by expected values
    if isempty(poles_complex)
        % All poles are real - sort by magnitude
        poles_sorted = sort(poles_z, 'descend');
        p3 = poles_sorted(3);  % LPF pole (smallest, around 0.53)

        % The remaining two form the 2nd-order system
        % a1, a2 are in standard form: (1 - p1*z^-1 + p2*z^-2)
        % For conjugate pair z1, z2: a1 = z1 + z2, a2 = -z1*z2
        a1 = poles_sorted(1) + poles_sorted(2);
        a2 = -poles_sorted(1) * poles_sorted(2);
    else
        % Complex conjugate pair from H_model
        p3 = poles_real(1);  % LPF pole
        a1 = 2 * real(poles_complex(1));
        a2 = -abs(poles_complex(1))^2;
    end

    % Convert to standard form coefficients used in control law
    % Den(z^-1) = 1 - a1*z^-1 - a2*z^-2 - a3*z^-3
    % where a3 = contribution from p3
    lpf_params.a1 = a1;
    lpf_params.a2 = a2;
    lpf_params.a3 = -a2 * p3;  % Cross term: -(-p1*p2)*p3 = p1*p2*p3

    %% Store Results
    lpf_params.k_o_lpf = k_o_lpf;
    lpf_params.bu = real(bu);  % Should be real
    lpf_params.ba = real(ba);  % Should be real
    lpf_params.p3 = real(p3);  % LPF pole

    % Store raw coefficients for verification
    lpf_params.num_z = num_z;
    lpf_params.den_z = den_z;
    lpf_params.dc_gain = dc_gain_z;

    %% Stability Check
    if any(abs(poles_z) >= 1)
        warning('lpf_system_params:Unstable', ...
            'System has poles outside unit circle - may be unstable');
    end

end
