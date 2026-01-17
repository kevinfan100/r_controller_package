function styles = plot_styles()
% PLOT_STYLES Returns unified plot style settings for all test scripts
%
% Output:
%   styles - Structure containing all style settings:
%     .channel_colors - 6x3 matrix, P1-P6 colors
%     .force_colors - 3x3 matrix, X/Y/Z colors
%     .force_markers - Cell array of markers for force axes
%     .channel_markers - Cell array of markers for channels
%     .measurement_linewidth - Measurement line width
%     .reference_linewidth - Reference line width
%     .axis_linewidth - Axis line width
%     .theory_linewidth - Theory curve line width
%     .xlabel_fontsize - X-axis label font size
%     .ylabel_fontsize - Y-axis label font size
%     .title_fontsize - Title font size
%     .tick_fontsize - Tick font size
%     .legend_fontsize - Legend font size
%     .marker_size - Marker size
%     .theory_color - Theory curve color (gray)
%
% Example:
%   styles = plot_styles();
%   plot(x, y, 'Color', styles.channel_colors(1,:), ...
%        'LineWidth', styles.measurement_linewidth);
%
% Author: R-Controller Package Team
% Date: 2025-01-17

    % ========================================================================
    % Channel Colors (P1-P6)
    % ========================================================================
    styles.channel_colors = [
        0.0000, 0.0000, 0.5000;  % P1: Dark blue
        0.0000, 0.0000, 1.0000;  % P2: Blue
        0.0000, 0.5000, 0.0000;  % P3: Green
        1.0000, 0.0000, 0.0000;  % P4: Red
        0.8000, 0.0000, 0.8000;  % P5: Purple
        0.0000, 0.7500, 0.7500;  % P6: Cyan
    ];

    % ========================================================================
    % Force Direction Colors (X/Y/Z)
    % ========================================================================
    styles.force_colors = [
        0.0000, 0.0000, 1.0000;  % Fx: Blue
        0.0000, 0.5000, 0.0000;  % Fy: Green
        1.0000, 0.0000, 0.0000;  % Fz: Red
    ];

    % ========================================================================
    % Markers
    % ========================================================================
    % Channel markers (P1-P6)
    styles.channel_markers = {'o', 's', '^', 'd', 'v', 'p'};

    % Force axis markers (X/Y/Z)
    styles.force_markers = {'o', 's', '^'};

    % ========================================================================
    % Line Widths
    % ========================================================================
    styles.measurement_linewidth = 3.0;     % Measurement data
    styles.reference_linewidth = 2.5;       % Reference/desired signals
    styles.axis_linewidth = 1.5;            % Axis frame
    styles.theory_linewidth = 3.5;          % Theory curves
    styles.bode_linewidth = 3.5;            % Bode plot curves

    % ========================================================================
    % Font Sizes
    % ========================================================================
    styles.xlabel_fontsize = 14;            % X-axis label
    styles.ylabel_fontsize = 14;            % Y-axis label
    styles.title_fontsize = 15;             % Plot title
    styles.tick_fontsize = 12;              % Tick marks
    styles.legend_fontsize = 11;            % Legend text

    % Bode plot specific (larger fonts)
    styles.bode_label_fontsize = 22;        % Bode plot axis labels
    styles.bode_tick_fontsize = 18;         % Bode plot tick labels
    styles.bode_legend_fontsize = 13;       % Bode plot legend

    % ========================================================================
    % Marker Settings
    % ========================================================================
    styles.marker_size = 8;                 % Default marker size
    styles.bode_marker_size = 9;            % Bode plot marker size

    % ========================================================================
    % Theory/Reference Colors
    % ========================================================================
    styles.theory_color = [0.5, 0.5, 0.5];  % Gray for theory curves
    styles.reference_color = [0, 0, 0];      % Black for reference lines
    styles.grid_color = [0.8, 0.8, 0.8];    % Light gray for grids

    % ========================================================================
    % Channel Labels
    % ========================================================================
    styles.channel_labels = {'P1', 'P2', 'P3', 'P4', 'P5', 'P6'};
    styles.force_labels = {'Fx', 'Fy', 'Fz'};
    styles.force_labels_short = {'X', 'Y', 'Z'};

end
