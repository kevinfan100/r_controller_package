function params = r_controller_calc_params(fB_c, fB_e, fB_f)
    % R Controller Parameter Calculator (with Bus Object creation)
    %
    % Calculate all controller coefficients from bandwidth parameters
    % and create the corresponding Simulink Bus Object definition.
    %
    % Inputs:
    %   fB_c - Control bandwidth [Hz]
    %   fB_e - Estimator bandwidth [Hz]
    %   fB_f - Feedforward bandwidth [Hz]
    %
    % Output:
    %   params - Simulink.Parameter object with Bus type
    %
    % Variable naming convention:
    %   - Addition: A (e.g., one_A_beta = 1 + beta)
    %   - Subtraction: S (e.g., one_S_bc = 1 - bc)
    %   - Multiplication: M (e.g., b_M_lambda_c = b * lambda_c)
    %   - Division: D (e.g., a_D_b = a / b)
    %   - Negative: neg_ (e.g., neg_beta = -beta)
    %
    % Note: This function also creates 'ParamsBus' in base workspace

    % ========================================
    % SYSTEM CONSTANTS
    % ========================================
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

    % ========================================
    % INTERMEDIATE PARAMETERS
    % ========================================
    lambda_f = exp(-fB_f * 2 * pi * params.T);
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

    % ========================================
    % FEEDFORWARD FILTER COEFFICIENTS
    % vf[k] = λf*vf[k-1] + kff{ vd[k] - λcvd[k-1]}
    % ========================================

    params.kff = (1 - lambda_f) / (1-lambda_c); % kff = 1 - λf/1 - λc
    params.one_S_bc = 1 - bc; % 1 - bc

    % ========================================
    % ESTIMATOR GAINS
    % delta_v_hat[k] = lambda_c * delta_v_hat[k-1] + delta_vf[k] + L1 * error
    % w1_hat[k] = (1+beta) * w1_hat[k-1] - beta * w2_hat[k-1] + L2 * error
    % w2_hat[k] = w1_hat[k-1] + L3 * error
    % ========================================
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

    % ========================================
    % CONTROL LAW COEFFICIENTS
    % (reuse existing parameters)
    % ========================================
    % ku, a1, a2, one_S_bc, bc already defined above

    % ========================================
    % CREATE SIMULINK BUS OBJECT
    % ========================================
    % Simulink requires explicit Bus Object definition for structures
    ParamsBus = Simulink.Bus;
    ParamsBus.Description = 'R Controller Parameters Structure';

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
    elems(15).Name = 'one_S_bc';
    elems(15).DataType = 'double';

    elems(16) = Simulink.BusElement;
    elems(16).Name = 'L1';
    elems(16).DataType = 'double';

    elems(17) = Simulink.BusElement;
    elems(17).Name = 'L2';
    elems(17).DataType = 'double';

    elems(18) = Simulink.BusElement;
    elems(18).Name = 'L3';
    elems(18).DataType = 'double';

    elems(19) = Simulink.BusElement;
    elems(19).Name = 'one_A_beta';
    elems(19).DataType = 'double';

    elems(20) = Simulink.BusElement;
    elems(20).Name = 'neg_beta';
    elems(20).DataType = 'double';

    % Assign elements to bus
    ParamsBus.Elements = elems;

    % Save ParamsBus to base workspace
    assignin('base', 'ParamsBus', ParamsBus);

    % ========================================
    % WRAP AS SIMULINK.PARAMETER WITH BUS TYPE
    % ========================================
    params_data = params;  % Keep the raw data
    params = Simulink.Parameter(params_data);
    params.DataType = 'Bus: ParamsBus';
    params.Description = 'R Controller Parameters';
end
