function [u, u_w1] = model_base_ctrl_function(vd, vm, params)
% MODEL_BASE_CTRL_FUNCTION Model Based Controller with pre-calculated parameters
%
% Implements Model Based Controller using pre-calculated parameters from
% model_base_ctrl_params(). This function contains ONLY the
% difference equations for real-time execution.
%
% Controller modes:
%   lpf_enable = 0: 2nd-order controller (original)
%     ff_enable = 1: Feedforward + DOB + PI (ZPETC mode)
%     ff_enable = 0: DOB + PI only (No feedforward, bypass mode)
%   lpf_enable = 1: 3rd-order controller with LPF
%     Feedforward uses bu (non-minimum phase zero)
%     Estimator gains L1, L2, L3 recalculated with bu
%     Control law uses 3rd-order denominator with ba terms
%     Reference: FluxControl_LowPassFiltering.pdf
%
% Inputs:
%   vd     - Desired voltage (6x1) [V]
%   vm     - Measured voltage (6x1) [V]
%   params - Controller parameters from model_base_ctrl_params()
%            Must include params.ff_enable and params.lpf_enable fields
%
% Outputs:
%   u     - Control output (6x1) [A]
%   u_w1  - Disturbance compensation output (6x1) [A]
%
% See also: model_base_ctrl_params, lpf_system_params, CLAUDE.md

    %% Persistent State Variables
    % 2nd-order states
    persistent vd_k1 vd_k2
    persistent vf_k1 vf_k2
    persistent delta_v_k1
    persistent delta_v_hat_k1
    persistent w1_hat_k1 w1_hat_k2 w2_hat_k1
    persistent delta_vc_k1 delta_vc_k2
    persistent uc_k1 uc_k2
    persistent uc_w1_k1 uc_w1_k2
    % 3rd-order states (additional for LPF mode)
    persistent vd_k3
    persistent vf_k3
    persistent delta_vc_k3
    persistent uc_k3
    persistent uc_w1_k3
    persistent w1_hat_k3
    persistent initialized

    %% Initialization
    if isempty(initialized)
        initialized = true;
        vd_k1 = zeros(6, 1);
        vd_k2 = zeros(6, 1);
        vd_k3 = zeros(6, 1);
        vf_k1 = zeros(6, 1);
        vf_k2 = zeros(6, 1);
        vf_k3 = zeros(6, 1);
        delta_v_k1 = zeros(6, 1);
        delta_v_hat_k1 = zeros(6, 1);
        w1_hat_k1 = zeros(6, 1);
        w1_hat_k2 = zeros(6, 1);
        w1_hat_k3 = zeros(6, 1);
        w2_hat_k1 = zeros(6, 1);
        delta_vc_k1 = zeros(6, 1);
        delta_vc_k2 = zeros(6, 1);
        delta_vc_k3 = zeros(6, 1);
        uc_k1 = zeros(6, 1);
        uc_k2 = zeros(6, 1);
        uc_k3 = zeros(6, 1);
        uc_w1_k1 = zeros(6, 1);
        uc_w1_k2 = zeros(6, 1);
        uc_w1_k3 = zeros(6, 1);
    end

    %% Mode Selection: LPF (3rd-order) vs Original (2nd-order)
    if params.lpf_enable > 0.5
        % ================================================================
        % LPF MODE: 3rd-order controller
        % Reference: FluxControl_LowPassFiltering.pdf
        % ================================================================

        %% Feedforward Filter (LPF Mode)
        % Reference: FluxControl_LowPassFiltering.pdf Page 2
        % vf[k] = kff_lpf{bu*vd[k] + (1-bu*λc)*vd[k-1] - λc*vd[k-2]}
        % NOTE: Use bu (non-minimum phase zero), NOT ba!
        if params.ff_enable > 0.5
            vf_k = params.lambda_f * vf_k1 + params.kff_lpf * ...
                   (params.bu * vd + params.one_S_bu_M_lambda_c * vd_k1 - params.lambda_c * vd_k2);
        else
            % Bypass feedforward: vf = vd
            vf_k = vd;
        end

        % δvf[k] = vf[k] - (λc + kc_lpf)*vf[k-1] - bc_lpf*vf[k-2]
        % Reference: FluxControl_LowPassFiltering.pdf Page 2
        % NOTE: Use kc_lpf, bc_lpf (calculated with bu)
        delta_vf = vf_k - params.lambda_c_A_kc_lpf * vf_k1 - params.bc_lpf * vf_k2;

        % δv[k] = vf[k] - vm[k]
        delta_v = vf_k - vm;

        %% Estimator (Disturbance Observer) - LPF Mode
        % Reference: FluxControl_LowPassFiltering.pdf Page 3
        % NOTE: Use L1_lpf, L2_lpf, L3_lpf (calculated with bu)
        error_term = delta_v_k1 - delta_v_hat_k1;

        % δv̂[k] = λc·δv̂[k-1] + δvf[k] + L1·{δv[k-1] - δv̂[k-1]}
        delta_v_hat = params.lambda_c * delta_v_hat_k1 + delta_vf + params.L1_lpf * error_term;

        % ŵ1[k] = (1+β)·ŵ1[k-1] - β·ŵ2[k-1] + L2·{δv[k-1] - δv̂[k-1]}
        w1_hat = params.one_A_beta * w1_hat_k1 + params.neg_beta * w2_hat_k1 + params.L2_lpf * error_term;

        % ŵ2[k] = ŵ1[k-1] + L3·{δv[k-1] - δv̂[k-1]}
        w2_hat = w1_hat_k1 + params.L3_lpf * error_term;

        %% Control Law (3rd-order)
        % δvc[k] = δv̂[k] - ŵ1[k]
        % Reference: FluxControl_LowPassFiltering.pdf Page 4, Step 7
        % NOTE: Use delta_v_hat (estimated), NOT delta_v (actual)!
        delta_vc = delta_v - w1_hat;

        % 3rd-order control law with correct denominator:
        % Reference: FluxControl_LowPassFiltering.pdf Page 1, 4
        % uc[k] = uc_coef_k1·uc[k-1] + uc_coef_k2·uc[k-2] + uc_coef_k3·uc[k-3]
        %       + ku_lpf·{δvc[k] - a1·δvc[k-1] - a2·δvc[k-2] - a3·δvc[k-3]}
        % where:
        %   uc_coef_k1 = λc - ba + kc_lpf
        %   uc_coef_k2 = ba*λc + bc_lpf + ba*kc_lpf
        %   uc_coef_k3 = ba*bc_lpf
        uc = params.uc_coef_k1 * uc_k1 + params.uc_coef_k2 * uc_k2 + params.uc_coef_k3 * uc_k3 + ...
             params.ku_lpf * (delta_vc - params.a1 * delta_vc_k1 ...
                              - params.a2 * delta_vc_k2 - params.a3 * delta_vc_k3);

        % uc_w1 for disturbance compensation (3rd-order)
        uc_w1 = params.uc_coef_k1 * uc_w1_k1 + params.uc_coef_k2 * uc_w1_k2 + params.uc_coef_k3 * uc_w1_k3 + ...
             params.ku_lpf * (w1_hat - params.a1 * w1_hat_k1 ...
                              - params.a2 * w1_hat_k2 - params.a3 * w1_hat_k3);

    else
        % ================================================================
        % ORIGINAL MODE: 2nd-order controller (unchanged)
        % ================================================================

        %% Feedforward Filter (Original Mode)
        if params.ff_enable > 0.5
            % ZPETC Feedforward Filter (Zero Phase Error Tracking Control)
            % vf[k] = λf*vf[k-1] + kff{b*vd[k] + (1-b*λc)*vd[k-1] - λc*vd[k-2]}
            % Reference: Inner_ctrl_low.pdf Page 3 (yellow highlight)
            vf_k = params.lambda_f * vf_k1 + params.kff * ...
                   (params.b * vd + params.one_S_b_M_lambda_c * vd_k1 - params.lambda_c * vd_k2);

            % δvf[k] = vf[k] - (λc+kc)*vf[k-1] - bc*vf[k-2]
            delta_vf = vf_k - params.lambda_c_A_kc * vf_k1 - params.bc * vf_k2;
        else
            % Bypass feedforward: vf = vd (direct passthrough)
            vf_k = vd;

            % δvf[k] = vd[k] - (λc+kc)*vd[k-1] - bc*vd[k-2]
            % Using vd directly when feedforward is bypassed
            delta_vf = vd - params.lambda_c_A_kc * vd_k1 - params.bc * vd_k2;
        end

        % δv[k] = vf[k] - vm[k]
        delta_v = vf_k - vm;

        %% Estimator (Disturbance Observer)
        error_term = delta_v_k1 - delta_v_hat_k1;

        % δv̂[k] = λc·δv̂[k-1] + δvf[k] + L1·{δv[k-1] - δv̂[k-1]}
        delta_v_hat = params.lambda_c * delta_v_hat_k1 + delta_vf + params.L1 * error_term;

        % ŵ1[k] = (1+β)·ŵ1[k-1] - β·ŵ2[k-1] + L2·{δv[k-1] - δv̂[k-1]}
        w1_hat = params.one_A_beta * w1_hat_k1 + params.neg_beta * w2_hat_k1 + params.L2 * error_term;

        % ŵ2[k] = ŵ1[k-1] + L3·{δv[k-1] - δv̂[k-1]}
        w2_hat = w1_hat_k1 + params.L3 * error_term;

        %% Control Law (2nd-order)
        % δvc[k] = δv̂[k] - ŵ1[k]
        % Reference: Inner_ctrl_low.pdf Page 4 (d=0), Equation 7
        % NOTE: Use delta_v_hat (estimated), NOT delta_v (actual)!
        delta_vc = delta_v - w1_hat;

        % uc[k] = (λc + kc)·uc[k-1] + bc·uc[k-2] + ku·{δvc[k] - a1·δvc[k-1] - a2·δvc[k-2]}
        % Reference: Inner_ctrl_low.pdf Page 4 (d=0), Equation 8
        uc = params.lambda_c_A_kc * uc_k1 + params.bc * uc_k2 + ...
             params.ku * (delta_vc - params.a1 * delta_vc_k1 - params.a2 * delta_vc_k2);

        % uc_w1[k] = (λc + kc)·uc_w1[k-1] + bc·uc_w1[k-2] + ku·{ŵ1[k] - a1·ŵ1[k-1] - a2·ŵ1[k-2]}
        uc_w1 = params.lambda_c_A_kc * uc_w1_k1 + params.bc * uc_w1_k2 + ...
             params.ku * (w1_hat - params.a1 * w1_hat_k1 - params.a2 * w1_hat_k2);
    end

    %% Output Calculation
    % u[k] = B^-1 · uc[k]
    u = params.B_inv * uc;

    % u[k] = B^-1 · uc_w1[k]
    u_w1 = params.B_inv * uc_w1;

    %% State Updates
    % 3rd-order states (shift before 2nd-order to preserve chain)
    vd_k3 = vd_k2;
    vf_k3 = vf_k2;
    delta_vc_k3 = delta_vc_k2;
    uc_k3 = uc_k2;
    uc_w1_k3 = uc_w1_k2;
    w1_hat_k3 = w1_hat_k2;

    % 2nd-order states
    vd_k2 = vd_k1;
    vd_k1 = vd;
    vf_k2 = vf_k1;
    vf_k1 = vf_k;
    delta_v_k1 = delta_v;
    delta_v_hat_k1 = delta_v_hat;
    w1_hat_k2 = w1_hat_k1;
    w1_hat_k1 = w1_hat;
    w2_hat_k1 = w2_hat;
    delta_vc_k2 = delta_vc_k1;
    delta_vc_k1 = delta_vc;
    uc_k2 = uc_k1;
    uc_k1 = uc;
    uc_w1_k2 = uc_w1_k1;
    uc_w1_k1 = uc_w1;
end
