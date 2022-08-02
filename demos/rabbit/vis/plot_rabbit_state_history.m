function fig = plot_rabbit_state_history(results, type)
%% fig = plot_rabbit_state_history(results, type)
% results:
%   the structure that contains the result of a controller
%   rollout, or the cell array that contains multiple rollout information.
%   the structure (or each cell element) should contain the following info:
%       .stamp: time history (1, T)
%       .trajectory: state history (14, T)
%       .forces: history of contact forces (2, T)
%       .legend: title of the rollout (for instance, the name of the
%       controller) that will be displayed in the plot legend.
%       .Color: line color to use (optional)
%       .LineWidth: line width to use (optional)
% type: type of the plot, options are
%   'simple': Display only the hip positions, swing foot positions, and the
%      contact forces.
%   'extened': Display the configuration variables together.
%   'full': Display the whole state variables.

if nargin < 2
    type = 'simple'; 
end
    

    palette = get_palette_colors();
    magenta = palette.magenta;
    blue = palette.blue;
    grey = palette.grey;
    green = palette.green;
    navy = palette.navy;
    orange = palette.orange;
    yellow = palette.yellow;
    
    if isstruct(results)
        fig = plot_rabbit_state_history_single(results);
        return
    end
    
    n_result = numel(results);
    colors_result = get_color_map([blue; yellow; magenta;], n_result-1);
    
    fig = open_figure('size', [1200, 900]);
    
    t = tiledlayout(3, 1);
    title(t, "State Info");    
    nexttile;
    for i_result = 1:n_result
        result = results{i_result};
        p = plot(result.trajectory(1, :), result.trajectory(2, :), 'DisplayName', result.legend);
        p.Color = colors_result(i_result, :);
        if isfield(result, 'Color')
            p.Color = result.Color;
        end
        if isfield(result, 'LineWidth')
            p.LineWidth = result.LineWidth;
        end
        hold on;       
    end
    ylabel("$y_{cm}$", "Interpreter", "latex");
    xlabel("$x_{cm}$", "Interpreter", "latex");
    title('Hip Position');
    
    % Swing Foot position
    nexttile;
    for i_result = 1:n_result
        result = results{i_result};
        traj_size = size(result.trajectory);
        traj_length = traj_size(2);
        p_lts = [];
        for i=1:traj_length
            p_lt = p_LeftToe(result.trajectory(1:7, i));
            p_lts = [p_lts, p_lt([1, 3])];
        end

        p = plot(p_lts(1, :), p_lts(2, :), '.', 'DisplayName', result.legend);
        p.Color = colors_result(i_result, :);
        if isfield(result, 'Color')
            p.Color = result.Color;
        end
        if isfield(result, 'LineWidth')
            p.LineWidth = result.LineWidth;
        end
        ylabel("$y_{sw}$", "Interpreter", "latex");
        xlabel("$x_{sw}$", "Interpreter", "latex");
        hold on;
        legend;
    end
    title('Swing Foot Position')
    
    % Contact Force Ratio
    nexttile;
    for i_result = 1:n_result
        result = results{i_result};
        p = plot(result.stamps, abs(result.forces(1, :)./result.forces(2,:)),...
            'DisplayName', result.legend);
        p.Color = colors_result(i_result, :);
        if isfield(result, 'Color')
            p.Color = result.Color;
        end
        if isfield(result, 'LineWidth')
            p.LineWidth = result.LineWidth;
        end
        hold on;
    end
    grid on;
    title('Contact Force Ratio ($|F_{T}$/$F_{N}|$)', 'Interpreter', 'latex');
    xlabel('$t$(sec)', 'Interpreter', 'latex');
    ylabel('$|F_{T}$/$F_{N}|$', 'Interpreter', 'latex');
    ylim([0, 1]);
end

function fig = plot_rabbit_state_history_single(result)
    palette = get_palette_colors();
    color_result = palette.blue;
    fig = open_figure('size', [1200, 900]);
    
    t = tiledlayout(3, 1);
    title(t, "State Info");    
    nexttile;
    p = plot(result.trajectory(1, :), result.trajectory(2, :));
    p.Color = color_result;
    if isfield(result, 'Color')
        p.Color = result.Color;
    end
    if isfield(result, 'LineWidth')
        p.LineWidth = result.LineWidth;
    end
    hold on;       
    ylabel("$y_{cm}$", "Interpreter", "latex");
    xlabel("$x_{cm}$", "Interpreter", "latex");
    title('Hip Position');
    
    % Swing Foot position
    nexttile;
    traj_size = size(result.trajectory);
    traj_length = traj_size(2);
    p_lts = [];
    for i=1:traj_length
        p_lt = p_LeftToe(result.trajectory(1:7, i));
        p_lts = [p_lts, p_lt([1, 3])];
    end

    p = plot(p_lts(1, :), p_lts(2, :), '.');
    p.Color = color_result;
    if isfield(result, 'Color')
        p.Color = result.Color;
    end
    if isfield(result, 'LineWidth')
        p.LineWidth = result.LineWidth;
    end
    ylabel("$y_{sw}$", "Interpreter", "latex");
    xlabel("$x_{sw}$", "Interpreter", "latex");
    hold on;
    title('Swing Foot Position')
    
    % Contact Force Ratio
    nexttile;
    p = plot(result.stamps, abs(result.forces(1, :)./result.forces(2,:)));
    p.Color = color_result;
    if isfield(result, 'Color')
        p.Color = result.Color;
    end
    if isfield(result, 'LineWidth')
        p.LineWidth = result.LineWidth;
    end
    hold on;

    grid on;
    title('Contact Force Ratio ($|F_{T}$/$F_{N}|$)', 'Interpreter', 'latex');
    xlabel('$t$(sec)', 'Interpreter', 'latex'); 
    ylabel('$|F_{T}$/$F_{N}|$', 'Interpreter', 'latex');
    ylim([0, 1]);
end