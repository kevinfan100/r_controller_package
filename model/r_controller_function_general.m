function [u, u_w1] = r_controller_function_general(vd, vm, params)
    % R Controller with Pre-calculated Parameters
    %
    % Implements R Controller using pre-calculated parameters from
    % r_controller_calc_params(). This function contains ONLY the
    % difference equations for real-time execution.
    %
    %
    % Variable naming convention:
    %   - Addition: A (e.g., one_A_beta = 1 + beta)
    %   - Subtraction: S (e.g., one_S_bc = 1 - bc)
    %   - Multiplication: M (e.g., b_M_lambda_c = b * lambda_c)
    %   - Negative: neg_ (e.g., neg_beta = -beta)

    % ====================================================================
    % PERSISTENT STATE VARIABLES
    % ====================================================================
    persistent vd_k1           
    persistent vf_k1 vf_k2           
    persistent delta_v_k1            
    persistent delta_v_hat_k1        
    persistent w1_hat_k1 w1_hat_k2 w2_hat_k1   
    persistent delta_vc_k1 delta_vc_k2
    persistent uc_k1 uc_k2 
    persistent uc_w1_k1 uc_w1_k2
    persistent initialized

    % ====================================================================
    % INITIALIZATION
    % ====================================================================
    if isempty(initialized)
        initialized = true;
        vd_k1 = zeros(6, 1);
        vf_k1 = zeros(6, 1);
        vf_k2 = zeros(6, 1);
        delta_v_k1 = zeros(6, 1);
        delta_v_hat_k1 = zeros(6, 1);
        w1_hat_k1 = zeros(6, 1);
        w1_hat_k2 = zeros(6, 1);
        w2_hat_k1 = zeros(6, 1);
        delta_vc_k1 = zeros(6, 1);
        delta_vc_k2 = zeros(6, 1);
        uc_k1 = zeros(6, 1);
        uc_k2 = zeros(6, 1);
        uc_w1_k1 = zeros(6, 1);
        uc_w1_k2 = zeros(6, 1);
    end

    % ====================================================================
    % FEEDFORWARD FILTER (vf calculation)
    % vf[k] = λf*vf[k-1] + kff{ vd[k] - λc*vd[k-1]}
    % ====================================================================
    vf_k = params.lambda_f * vf_k1 + params.kff * (vd - params.lambda_c * vd_k1);

    % δvf[k] = vf[k] - (1-bc)·vf[k-1] - bc·vf[k-2]
    delta_vf = vf_k - params.one_S_bc * vf_k1 - params.bc * vf_k2;

    % δv[k] = vf[k] - vm[k]
    delta_v = vf_k - vm;

    % ====================================================================
    % ESTIMATOR (disturbance observer)
    % ====================================================================
    error_term = delta_v_k1 - delta_v_hat_k1;

    % δv̂[k] = λc·δv̂[k-1] + δvf[k] + L1·{δv[k-1] - δv̂[k-1]}
    delta_v_hat = params.lambda_c * delta_v_hat_k1 + delta_vf + params.L1 * error_term;

    % ŵ1[k] = (1+β)·ŵ1[k-1] - β·ŵ2[k-1] + L2·{δv[k-1] - δv̂[k-1]}
    w1_hat = params.one_A_beta * w1_hat_k1 + params.neg_beta * w2_hat_k1 + params.L2 * error_term;

    % ŵ2[k] = ŵ1[k-1] + L3·{δv[k-1] - δv̂[k-1]}
    w2_hat = w1_hat_k1 + params.L3 * error_term;

    % ====================================================================
    % CONTROL LAW
    % ====================================================================
    % δvc[k] = δv[k] - ŵ1[k]
    delta_vc = delta_v - w1_hat;

    % uc[k] = (1-bc)·uc[k-1] + bc·uc[k-2] + ku·{δvc[k] - a1·δvc[k-1] - a2·δvc[k-2]}
    uc = params.one_S_bc * uc_k1 + params.bc * uc_k2 + ...
         params.ku * (delta_vc - params.a1 * delta_vc_k1 - params.a2 * delta_vc_k2);

    % uc_w1[k] = (1-bc)·uc_w1[k-1] + bc·uc_w1[k-2] + ku·{ŵ1[k] - a1·ŵ1[k-1] - a2·ŵ1[k-2]}
    uc_w1 = params.one_S_bc * uc_w1_k1 + params.bc * uc_w1_k2 + ...
         params.ku * (w1_hat - params.a1 * w1_hat_k1 - params.a2 * w1_hat_k2);

    % u[k] = B^-1 · uc[k]
    u = params.B_inv * uc;

    % u[k] = B^-1 · uc_w1[k]
    u_w1 = params.B_inv * uc_w1;

    % ====================================================================
    % STATE UPDATES
    % ====================================================================
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
