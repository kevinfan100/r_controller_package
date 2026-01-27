function [u, u_w1] = model_base_ctrl_function(vd, vm, params)
% MODEL_BASE_CTRL_FUNCTION Zero Phase Error Tracking Controller (ZPETC)
%
% Implements ZPETC using pre-calculated parameters from
% model_base_ctrl_params(). This function contains ONLY the
% difference equations for real-time execution.
%
% Inputs:
%   vd     - Desired voltage (6x1) [V]
%   vm     - Measured voltage (6x1) [V]
%   params - Controller parameters from model_base_ctrl_params()
%
% Outputs:
%   u     - Control output (6x1) [A]
%   u_w1  - Disturbance compensation output (6x1) [A]
%
% See also: model_base_ctrl_params, CLAUDE.md

    %% Persistent State Variables
    persistent vd_k1 vd_k2            
    persistent vf_k1 vf_k2           
    persistent delta_v_k1            
    persistent delta_v_hat_k1        
    persistent w1_hat_k1 w1_hat_k2 w2_hat_k1   
    persistent delta_vc_k1 delta_vc_k2
    persistent uc_k1 uc_k2 
    persistent uc_w1_k1 uc_w1_k2
    persistent initialized

    %% Initialization
    if isempty(initialized)
        initialized = true;
        vd_k1 = zeros(6, 1);
        vd_k2 = zeros(6, 1);
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

    %% Feedforward Filter (temp zero phase)
    % vf[k] = λf*vf[k-1] + kff{b*vd[k] + (1-λc*b)*vd[k-1] - λc*vd[k-2]}
    vf_k = params.lambda_f * vf_k1 + params.kff * (params.b * vd + (1 - params.lambda_c * params.b) * vd_k1 - params.lambda_c * vd_k2);

    % δvf[k] = vf[k] - (λc+kc)·vf[k-1] - bc·vf[k-2] (temp zero phase)
    delta_vf = vf_k - (params.lambda_c + params.kc) * vf_k1 - params.bc * vf_k2;

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

    %% Control Law
    % δvc[k] = δv[k] - ŵ1[k]  (使用直接測量值，非估測值)
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

    %% State Updates
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
