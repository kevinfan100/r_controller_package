function styles = plot_styles()
% PLOT_STYLES Returns unified plot style settings for all test scripts
%
% Output:
%   styles - Structure containing all style settings:
%     Colors:
%       .channel_colors - 6x3 matrix, P1-P6 colors
%       .force_colors - 3x3 matrix, X/Y/Z colors
%       .theory_color, .reference_color, .grid_color
%     Markers:
%       .channel_markers, .force_markers - Cell arrays
%       .marker_size, .bode_marker_size, .marker_edgewidth
%     Line Widths:
%       .measurement_linewidth, .reference_linewidth
%       .axis_linewidth, .theory_linewidth, .bode_linewidth
%     Font Sizes:
%       .xlabel_fontsize, .ylabel_fontsize, .title_fontsize
%       .tick_fontsize, .legend_fontsize
%       .bode_label_fontsize, .bode_tick_fontsize, .bode_title_fontsize
%     Font Weights:
%       .label_fontweight, .title_fontweight ('bold')
%       .tick_fontweight, .legend_fontweight ('normal')
%     Axis Settings:
%       .fontname, .box, .tickdir, .ticklength
%     Export:
%       .export_resolution, .export_format
%
% Example:
%   styles = plot_styles();
%   plot(x, y, 'Color', styles.channel_colors(1,:), ...
%        'LineWidth', styles.measurement_linewidth);
%
% Reference implementation of project standards.
% See also: CLAUDE.md - "Plot Style Standards" section
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
    styles.bode_title_fontsize = 22;        % Bode plot title

    % ========================================================================
    % Font Weights (for publication quality)
    % ========================================================================
    styles.label_fontweight = 'bold';       % Axis labels (xlabel/ylabel)
    styles.title_fontweight = 'bold';       % Plot titles
    styles.tick_fontweight = 'normal';      % Tick labels
    styles.legend_fontweight = 'normal';    % Legend text

    % ========================================================================
    % Font Name
    % ========================================================================
    styles.fontname = 'Arial';              % IEEE standard font

    % ========================================================================
    % Marker Settings
    % ========================================================================
    styles.marker_size = 8;                 % Default marker size
    styles.bode_marker_size = 9;            % Bode plot marker size
    styles.marker_edgewidth = 1.5;          % Marker edge line width

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

    % ========================================================================
    % Axis Settings
    % ========================================================================
    styles.box = 'on';                      % Show complete box
    styles.tickdir = 'out';                 % Tick direction outward
    styles.ticklength = [0.02, 0.02];       % Tick length [2D, 3D]

    % ========================================================================
    % Export Settings
    % ========================================================================
    styles.export_resolution = 300;         % PNG export DPI
    styles.export_format = 'png';           % Export format

end
