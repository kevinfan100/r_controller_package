function config = test_config(varargin)
% TEST_CONFIG Returns unified test configuration parameters
%
% Provides centralized configuration for all test scripts, ensuring
% consistency across different tests.
%
% Usage:
%   config = test_config();                    % Default configuration
%   config = test_config('Type', 'force');     % Force control specific
%   config = test_config('Type', 'inner_loop'); % Inner loop specific
%
% Optional Parameters (Name-Value pairs):
%   'Type' - Configuration type: 'default', 'inner_loop', 'force' (default: 'default')
%
% Output:
%   config - Structure containing:
%     .Ts                - Sampling time [s] (default: 1e-5 = 100 kHz)
%     .fs                - Sampling frequency [Hz] (default: 100000)
%     .solver            - Simulink solver (default: 'ode5')
%     .model_name        - Simulink model name
%     .fB_f              - Default feedforward bandwidth [Hz]
%     .fB_c              - Default controller bandwidth [Hz]
%     .fB_e              - Default estimator bandwidth [Hz]
%     .total_cycles      - Total simulation cycles for periodic signals
%     .skip_cycles       - Cycles to skip for transient
%     .fft_cycles        - Cycles to use for FFT analysis
%     .display_cycles    - Cycles to display in plots
%     .min_sim_time      - Minimum simulation time [s]
%     .max_sim_time      - Maximum simulation time [s]
%     .steady_threshold  - Steady-state detection threshold
%     .thd_threshold     - THD threshold [%]
%     .dc_tolerance      - DC tolerance
%     .freq_error_threshold - Frequency error warning threshold [%]
%     .export_resolution - PNG export resolution [DPI]
%
% Example:
%   config = test_config('Type', 'inner_loop');
%   ctrl_params = model_base_ctrl_params(config.fB_c, config.fB_e, config.fB_f);
%
% Reference implementation of project standards.
% See also: CLAUDE.md - "Test Pass/Fail Criteria" section

    % Parse optional parameters
    p = inputParser;
    addParameter(p, 'Type', 'default');
    parse(p, varargin{:});

    config_type = lower(p.Results.Type);

    % ========================================================================
    % System Constants (Common to all)
    % ========================================================================
    config.Ts = 1e-5;                       % Sampling time [s] (100 kHz)
    config.fs = 1 / config.Ts;              % Sampling frequency [Hz]
    config.solver = 'ode45';                % Variable-step solver (for compatibility with 1600 Hz Motion Control)
    config.model_name = 'main_system';

    % ========================================================================
    % Controller Parameters
    % ========================================================================
    switch config_type
        case 'inner_loop'
            % Inner loop test defaults (conservative bandwidths)
            config.fB_f = 1000;             % Feedforward bandwidth [Hz]
            config.fB_c = 300;              % Controller bandwidth [Hz]
            config.fB_e = 500;              % Estimator bandwidth [Hz]

        case 'force'
            % Force control test defaults (higher bandwidths)
            config.fB_f = 3000;             % Feedforward bandwidth [Hz]
            config.fB_c = 3200;             % Controller bandwidth [Hz]
            config.fB_e = 16000;            % Estimator bandwidth [Hz]

        otherwise
            % Default (same as inner_loop)
            config.fB_f = 1000;
            config.fB_c = 300;
            config.fB_e = 500;
    end

    % ========================================================================
    % Simulation Cycle Parameters
    % ========================================================================
    config.total_cycles = 100;              % Total cycles to simulate
    config.skip_cycles = 60;                % Skip transient cycles
    config.fft_cycles = 40;                 % Use for FFT analysis
    config.display_cycles = 5;              % Display in plots

    % ========================================================================
    % Simulation Time Limits
    % ========================================================================
    config.min_sim_time = 0.1;              % Minimum simulation time [s]
    config.max_sim_time = 5.0;              % Maximum simulation time [s]

    % ========================================================================
    % Quality Check Thresholds
    % ========================================================================
    config.steady_threshold = 0.02;         % 2% of amplitude
    config.thd_threshold = 1.0;             % 1% THD
    config.dc_tolerance = 0.01;             % 1% of amplitude
    config.freq_error_threshold = 0.1;      % 0.1% frequency error warning

    % ========================================================================
    % Output Settings
    % ========================================================================
    config.export_resolution = 300;         % PNG export DPI

    % ========================================================================
    % Force Control Specific (only for 'force' type)
    % ========================================================================
    if strcmp(config_type, 'force')
        config.R_norm = 550.0;              % Normalization radius [um]
        config.FGain = 8.0;                 % Force gain [pN]
        config.pos_update_rate = 1600;      % Hardware position update rate [Hz]
        config.interp_method = 'linear';    % Interpolation method
    end

    % ========================================================================
    % Controller Type Selection
    % ========================================================================
    % 'model_base_ctrl' = R-Controller (Model Based Control)
    % 'pi_ctrl' = PI Controller
    config.default_controller_type = 'model_base_ctrl';
    config.controller_type_enum.MODEL_BASE_CTRL = 1;
    config.controller_type_enum.PI_CTRL = 2;

    % ========================================================================
    % PI Controller Parameters (alternative to R-Controller)
    % ========================================================================
    config.Kp_default = 2;                  % Default proportional gain
    config.zc_default = 2206;               % Default zero location [rad/s]
    config.Ki_default = config.Kp_default * config.zc_default;

    % ========================================================================
    % Frequency Sweep Points
    % ========================================================================
    % Inner loop frequencies (all produce integer samples_per_cycle)
    config.inner_loop_frequencies = [10, 50, 100, 125, 200, 250, ...
                                     400, 500, 625, 800, 1000, 1250, 2000];

    % Force control frequencies (limited by Nyquist of pos_update_rate)
    config.force_frequencies = [1, 10, 20, 50, 100, 125, 200, 250];

end
