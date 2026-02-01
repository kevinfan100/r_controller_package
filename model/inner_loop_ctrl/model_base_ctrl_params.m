function ctrl_params = model_base_ctrl_params(fB_c, fB_e, fB_f, varargin)
% MODEL_BASE_CTRL_PARAMS Model Based Controller parameter calculator with Bus Object creation
%
% Calculate all controller coefficients from bandwidth parameters and create
% the corresponding Simulink Bus Object definition.
%
% Inputs:
%   fB_c - Control bandwidth [Hz]
%   fB_e - Estimator bandwidth [Hz]
%   fB_f - Feedforward bandwidth [Hz]
%
% Optional Parameters (Name-Value pairs):
%   'ff_enable'  - Enable feedforward filter (default: true)
%                  true  = Use feedforward filter (ZPETC)
%                  false = Bypass feedforward (vf = vd)
%   'lpf_enable' - Enable LPF mode for 3rd-order controller (default: false)
%                  false = Use original 2nd-order controller
%                  true  = Use 3rd-order controller with LPF
%   'f_low'      - LPF cutoff frequency [Hz] (required when lpf_enable=true)
%                  Typical values: 5000, 10000, 20000 Hz
%
% Output:
%   ctrl_params - Simulink.Parameter object with Bus type
%
% Example:
%   % Default: feedforward enabled, LPF disabled (backward compatible)
%   ctrl_params = model_base_ctrl_params(300, 500, 1000);
%
%   % Disable feedforward for pure feedback control
%   ctrl_params = model_base_ctrl_params(300, 500, 1000, 'ff_enable', false);
%
%   % Enable LPF mode with 10 kHz cutoff
%   ctrl_params = model_base_ctrl_params(500, 2500, 3000, ...
%       'lpf_enable', true, 'f_low', 10000);
%
% Note: This function creates 'ModelBaseCtrlParamsBus' in base workspace
%
% See also: model_base_ctrl_function, lpf_system_params, CLAUDE.md

    %% Parse Optional Parameters
    p = inputParser;
    addParameter(p, 'ff_enable', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'lpf_enable', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'f_low', 10000, @(x) isnumeric(x) && x > 0);
    parse(p, varargin{:});

    ff_enable = logical(p.Results.ff_enable);
    lpf_enable = logical(p.Results.lpf_enable);
    f_low = p.Results.f_low;

    %% Validate LPF Parameters
    if lpf_enable
        if f_low < 1000 || f_low > 50000
            warning('model_base_ctrl_params:UnusualFrequency', ...
                'f_low=%.0f Hz is outside recommended range [1000, 50000] Hz', f_low);
        end
    end

    %% System Constants
    params.k_o = 5.6695e-4;              % Plant gain from H(z^-1)
    params.b = 0.9782;                   % Zero coefficient from H(z^-1)
    params.a1 = 1.934848;                % Pole coefficient 1
    params.a2 = -0.935970;               % Pole coefficient 2
    params.T = 1e-5;                     % Sampling time [s]

    % Coupling matrix B and its inverse
    B = [0.2365  -0.0064  -0.0327  -0.0344  -0.0408  -0.0343;
        -0.0037   0.2818  -0.0427  -0.0675  -0.0779  -0.0368;
        -0.0375  -0.0328   0.2108  -0.0060  -0.0265  -0.0341;
        -0.0245  -0.0777  -0.0056   0.2361  -0.0770  -0.0241;
        -0.0413  -0.0760  -0.0234  -0.0720   0.2572  -0.0045;
        -0.0244  -0.0330  -0.0257  -0.0245  -0.0030   0.1845] .*1;
    params.B_inv = inv(B);

    %% Intermediate Parameters
    lambda_f = 0;  % Zero Phase Feedforward: λf = 0
    lambda_c = exp(-fB_c * 2 * pi * params.T);
    lambda_e = exp(-fB_e * 2 * pi * params.T);

    beta = sqrt(lambda_e * lambda_c);

    kc = (1 - lambda_c) / (1 + params.b);
    bc = params.b * kc;
    ku = kc / params.k_o;

    params.lambda_f = lambda_f;
    params.lambda_c = lambda_c;
    params.lambda_e = lambda_e;
    params.beta = beta;
    params.kc = kc;
    params.bc = bc;
    params.ku = ku;

    %% Feedforward Filter Coefficients
    % vf[k] = λf*vf[k-1] + kff{b*vd[k] + (1-b*λc)*vd[k-1] - λc*vd[k-2]}
    % Reference: Inner_ctrl_low.pdf Page 3 (yellow highlight)
    params.kff = (1 - lambda_f) / ((1 + params.b) * (1 - lambda_c));
    params.one_S_b_M_lambda_c = 1 - params.b * lambda_c;  % (1 - b·λc) for feedforward

    %% Common Coefficients
    % (λc + kc) used in δvf and uc equations
    % Reference: Inner_ctrl_low.pdf Page 4
    params.lambda_c_A_kc = lambda_c + kc;  % (λc + kc)

    %% Estimator Gains
    % delta_v_hat[k] = lambda_c * delta_v_hat[k-1] + delta_vf[k] + L1 * error
    % w1_hat[k] = (1+beta) * w1_hat[k-1] - beta * w2_hat[k-1] + L2 * error
    % w2_hat[k] = w1_hat[k-1] + L3 * error
    params.L1 = lambda_c + (1 + beta) - 3*lambda_e;

    params.L2 = (params.b*(lambda_e - 1)^3 - ...
                 beta*(params.b + 1)*(beta^2 - 3*beta*lambda_e + beta + ...
                 3*lambda_e^2 - 3*lambda_e + 1)) / ...
                (kc * (params.b + 1) * (params.b + beta));

    params.L3 = -(beta + params.b + beta*params.b - 3*beta*lambda_e - ...
                  3*params.b*lambda_e + params.b*beta^2 + ...
                  3*params.b*lambda_e^2 + beta^2 + lambda_e^3 - ...
                  3*beta*params.b*lambda_e) / ...
                 (kc * (params.b + 1) * (params.b + beta));

    params.one_A_beta = 1 + beta;        % 1 + beta
    params.neg_beta = -beta;             % -beta

    %% Feedforward Enable Flag
    % ff_enable: 1.0 = feedforward enabled (ZPETC), 0.0 = feedforward bypassed
    params.ff_enable = double(ff_enable);

    %% LPF Mode Parameters
    % lpf_enable: 1.0 = 3rd-order controller with LPF, 0.0 = 2nd-order (original)
    params.lpf_enable = double(lpf_enable);
    params.f_low = f_low;

    if lpf_enable
        % Calculate 3rd-order system parameters using lpf_system_params
        lpf_sys = lpf_system_params(f_low, 'Ts', params.T);

        % Store LPF-specific parameters
        params.k_o_lpf = lpf_sys.k_o_lpf;    % 3rd-order system gain
        params.bu = lpf_sys.bu;              % Non-minimum phase zero (≈3.16)
        params.ba = lpf_sys.ba;              % Minimum phase zero (≈0.22)
        params.p3 = lpf_sys.p3;              % LPF pole (≈0.53)

        % IMPORTANT: Override a1, a2, a3 with 3rd-order system coefficients
        % The control law uses: Den(z^-1) = 1 - a1*z^-1 - a2*z^-2 - a3*z^-3
        % These must be the full 3rd-order coefficients, not the original 2nd-order ones
        params.a1 = lpf_sys.a1;              % 3rd-order: ≈2.47
        params.a2 = lpf_sys.a2;              % 3rd-order: ≈-1.97
        params.a3 = lpf_sys.a3;              % 3rd-order: ≈0.50

        % ============================================================
        % LPF Mode: Recalculate kc, bc using bu (NOT b!)
        % Reference: FluxControl_LowPassFiltering.pdf Page 1
        % ============================================================
        kc_lpf = (1 - lambda_c) / (1 + params.bu);
        bc_lpf = params.bu * kc_lpf;
        params.kc_lpf = kc_lpf;
        params.bc_lpf = bc_lpf;

        % Recalculate control gain for 3rd-order system
        % ku = kc / k_o, where kc must use bu
        params.ku_lpf = kc_lpf / params.k_o_lpf;

        % LPF mode feedforward coefficients
        % Reference: FluxControl_LowPassFiltering.pdf Page 2
        % vf[k] = kff{bu*vd[k] + (1-bu*λc)*vd[k-1] - λc*vd[k-2]}
        % NOTE: Use bu (NOT ba!) for feedforward
        params.kff_lpf = (1 - lambda_f) / ((1 + params.bu) * (1 - lambda_c));
        params.one_S_bu_M_lambda_c = 1 - params.bu * lambda_c;  % (1 - bu·λc)

        % δvf coefficients for LPF mode
        % δvf[k] = vf[k] - (λc + kc_lpf)*vf[k-1] - bc_lpf*vf[k-2]
        params.lambda_c_A_kc_lpf = lambda_c + kc_lpf;

        % ============================================================
        % LPF Mode: Recalculate Estimator Gains L2, L3 using bu
        % Reference: FluxControl_LowPassFiltering.pdf Page 3
        % ============================================================
        % L1 is the same (doesn't contain b or bu)
        params.L1_lpf = lambda_c + (1 + beta) - 3*lambda_e;

        % L2 = [bu(λe-1)³ - β(bu+1)(β²-3βλe+β+3λe²-3λe+1)] / [kc(bu+1)(bu+β)]
        params.L2_lpf = (params.bu*(lambda_e - 1)^3 - ...
                     beta*(params.bu + 1)*(beta^2 - 3*beta*lambda_e + beta + ...
                     3*lambda_e^2 - 3*lambda_e + 1)) / ...
                    (kc_lpf * (params.bu + 1) * (params.bu + beta));

        % L3 = -[β + bu + β·bu - 3βλe - 3bu·λe + β²·bu + 3bu·λe² + β² + λe³ - 3β·bu·λe]
        %      / [kc(bu+1)(bu+β)]
        params.L3_lpf = -(beta + params.bu + beta*params.bu - 3*beta*lambda_e - ...
                      3*params.bu*lambda_e + params.bu*beta^2 + ...
                      3*params.bu*lambda_e^2 + beta^2 + lambda_e^3 - ...
                      3*beta*params.bu*lambda_e) / ...
                     (kc_lpf * (params.bu + 1) * (params.bu + beta));

        % ============================================================
        % LPF Mode: Control Law Denominator Coefficients (d=0)
        % Reference: FluxControl_LowPassFiltering.pdf Page 1, 4
        % ============================================================
        % b1 = λc - ba
        % b_{1} = kc_lpf  (when d=0)
        % b2 = ba * λc
        % b_{2} = bc_lpf + ba * kc_lpf
        % b_{3} = ba * bc_lpf
        %
        % uc[k] = (b1+b_{1})*uc[k-1] + (b2+b_{2})*uc[k-2] + b_{3}*uc[k-3] + ...
        params.uc_coef_k1 = (lambda_c - params.ba) + kc_lpf;  % λc - ba + kc_lpf
        params.uc_coef_k2 = params.ba * lambda_c + bc_lpf + params.ba * kc_lpf;
        params.uc_coef_k3 = params.ba * bc_lpf;
    else
        % Default values when LPF disabled (not used, but Bus requires them)
        params.k_o_lpf = params.k_o;
        params.bu = 0;
        params.ba = params.b;  % Use original zero for consistency
        params.p3 = 0;
        params.a3 = 0;
        params.kc_lpf = kc;
        params.bc_lpf = bc;
        params.ku_lpf = params.ku;
        params.kff_lpf = params.kff;
        params.one_S_bu_M_lambda_c = params.one_S_b_M_lambda_c;
        params.lambda_c_A_kc_lpf = params.lambda_c_A_kc;
        params.L1_lpf = params.L1;
        params.L2_lpf = params.L2;
        params.L3_lpf = params.L3;
        params.uc_coef_k1 = lambda_c + kc;  % Same as 2nd-order
        params.uc_coef_k2 = bc;
        params.uc_coef_k3 = 0;
    end

    %% Control Law Coefficients
    % ku, a1, a2, one_S_bc, bc already defined above

    %% Create Simulink Bus Object
    % Simulink requires explicit Bus Object definition for structures
    ModelBaseCtrlParamsBus = Simulink.Bus;
    ModelBaseCtrlParamsBus.Description = 'Model Based Controller Parameters Structure';

    % Define all bus elements 
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'k_o';
    elems(1).DataType = 'double';

    elems(2) = Simulink.BusElement;
    elems(2).Name = 'b';
    elems(2).DataType = 'double';

    elems(3) = Simulink.BusElement;
    elems(3).Name = 'a1';
    elems(3).DataType = 'double';

    elems(4) = Simulink.BusElement;
    elems(4).Name = 'a2';
    elems(4).DataType = 'double';

    elems(5) = Simulink.BusElement;
    elems(5).Name = 'T';
    elems(5).DataType = 'double';

    elems(6) = Simulink.BusElement;
    elems(6).Name = 'B_inv';
    elems(6).Dimensions = [6 6];
    elems(6).DataType = 'double';

    elems(7) = Simulink.BusElement;
    elems(7).Name = 'lambda_f';
    elems(7).DataType = 'double';

    elems(8) = Simulink.BusElement;
    elems(8).Name = 'lambda_c';
    elems(8).DataType = 'double';

    elems(9) = Simulink.BusElement;
    elems(9).Name = 'lambda_e';
    elems(9).DataType = 'double';

    elems(10) = Simulink.BusElement;
    elems(10).Name = 'beta';
    elems(10).DataType = 'double';

    elems(11) = Simulink.BusElement;
    elems(11).Name = 'kc';
    elems(11).DataType = 'double';

    elems(12) = Simulink.BusElement;
    elems(12).Name = 'bc';
    elems(12).DataType = 'double';

    elems(13) = Simulink.BusElement;
    elems(13).Name = 'ku';
    elems(13).DataType = 'double';

    elems(14) = Simulink.BusElement;
    elems(14).Name = 'kff';
    elems(14).DataType = 'double';

    elems(15) = Simulink.BusElement;
    elems(15).Name = 'one_S_b_M_lambda_c';
    elems(15).DataType = 'double';

    elems(16) = Simulink.BusElement;
    elems(16).Name = 'lambda_c_A_kc';
    elems(16).DataType = 'double';

    elems(17) = Simulink.BusElement;
    elems(17).Name = 'L1';
    elems(17).DataType = 'double';

    elems(18) = Simulink.BusElement;
    elems(18).Name = 'L2';
    elems(18).DataType = 'double';

    elems(19) = Simulink.BusElement;
    elems(19).Name = 'L3';
    elems(19).DataType = 'double';

    elems(20) = Simulink.BusElement;
    elems(20).Name = 'one_A_beta';
    elems(20).DataType = 'double';

    elems(21) = Simulink.BusElement;
    elems(21).Name = 'neg_beta';
    elems(21).DataType = 'double';

    elems(22) = Simulink.BusElement;
    elems(22).Name = 'ff_enable';
    elems(22).DataType = 'double';

    % LPF mode parameters (elements 23-32)
    elems(23) = Simulink.BusElement;
    elems(23).Name = 'lpf_enable';
    elems(23).DataType = 'double';

    elems(24) = Simulink.BusElement;
    elems(24).Name = 'f_low';
    elems(24).DataType = 'double';

    elems(25) = Simulink.BusElement;
    elems(25).Name = 'k_o_lpf';
    elems(25).DataType = 'double';

    elems(26) = Simulink.BusElement;
    elems(26).Name = 'bu';
    elems(26).DataType = 'double';

    elems(27) = Simulink.BusElement;
    elems(27).Name = 'ba';
    elems(27).DataType = 'double';

    elems(28) = Simulink.BusElement;
    elems(28).Name = 'p3';
    elems(28).DataType = 'double';

    elems(29) = Simulink.BusElement;
    elems(29).Name = 'a3';
    elems(29).DataType = 'double';

    % LPF mode parameters (elements 30-41)
    % Order must match struct field assignment order exactly!
    elems(30) = Simulink.BusElement;
    elems(30).Name = 'kc_lpf';
    elems(30).DataType = 'double';

    elems(31) = Simulink.BusElement;
    elems(31).Name = 'bc_lpf';
    elems(31).DataType = 'double';

    elems(32) = Simulink.BusElement;
    elems(32).Name = 'ku_lpf';
    elems(32).DataType = 'double';

    elems(33) = Simulink.BusElement;
    elems(33).Name = 'kff_lpf';
    elems(33).DataType = 'double';

    elems(34) = Simulink.BusElement;
    elems(34).Name = 'one_S_bu_M_lambda_c';
    elems(34).DataType = 'double';

    elems(35) = Simulink.BusElement;
    elems(35).Name = 'lambda_c_A_kc_lpf';
    elems(35).DataType = 'double';

    elems(36) = Simulink.BusElement;
    elems(36).Name = 'L1_lpf';
    elems(36).DataType = 'double';

    elems(37) = Simulink.BusElement;
    elems(37).Name = 'L2_lpf';
    elems(37).DataType = 'double';

    elems(38) = Simulink.BusElement;
    elems(38).Name = 'L3_lpf';
    elems(38).DataType = 'double';

    elems(39) = Simulink.BusElement;
    elems(39).Name = 'uc_coef_k1';
    elems(39).DataType = 'double';

    elems(40) = Simulink.BusElement;
    elems(40).Name = 'uc_coef_k2';
    elems(40).DataType = 'double';

    elems(41) = Simulink.BusElement;
    elems(41).Name = 'uc_coef_k3';
    elems(41).DataType = 'double';

    % Assign elements to bus
    ModelBaseCtrlParamsBus.Elements = elems;

    % Save ModelBaseCtrlParamsBus to base workspace
    assignin('base', 'ModelBaseCtrlParamsBus', ModelBaseCtrlParamsBus);

    %% Wrap as Simulink.Parameter with Bus Type
    params_data = params;  % Keep the raw data
    ctrl_params = Simulink.Parameter(params_data);
    ctrl_params.DataType = 'Bus: ModelBaseCtrlParamsBus';
    ctrl_params.Description = 'Model Based Controller Parameters';

end
