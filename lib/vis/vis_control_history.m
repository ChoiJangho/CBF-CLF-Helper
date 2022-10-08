function vis_control_history(results, varargin)
% Plot helper for control history information
% It compares every control input dimension for every results
%   TODO: support, udim selection (not sure if we need this a lot, though)
% Argument
%   results: cell array or single struct of result that contains controls
%       as its fieldname
%   varargin:
%       ylabel_description: ylabel name for each state plot
%       title_description: title name for each state plot
%       window_name: window name
    
    % Result Sanity Check
    n_result = numel(results);
    if ~iscell(results)
        results = {results};
    end
    result_peek = results{1};
    [udim, ~] = size(result_peek.controls);
    if ~isfield(result_peek, 'controls')
        error("Result doesn't have <controls> as a fieldname.");
    end
    
    % Varargin Sanity Check
    kwargs = parse_function_args(varargin{:});
    if ~isfield(kwargs, "ylabel_descriptions")
        ylabel_descriptions = arrayfun(@(dim) strcat("$u_", num2str(dim),"$"), ...
            linspace(1, udim, udim));
    else
        ylabel_descriptions = kwargs.ylabel_descriptions;
        if numel(ylabel_descriptions) ~= udim
            error("You should provide same number of ylabel description with state dimension");
        end
    end
    if ~isfield(kwargs, "title_descriptions")
        title_descriptions = arrayfun(@(dim) strcat("$u_", num2str(dim), "$"), ...
            linspace(1, udim, udim));
    else
        title_descriptions = kwargs.title_descriptions;
        if numel(title_descriptions) ~= udim
            error("You should provide same number of title description with state dimension");
        end
    end
    if ~isfield(kwargs, "window_name")
        window_name = "Control Hisotry";
    else
        window_name = kwargs.window_name;
    end
    
    % Load Color Map
    palette = get_palette_colors();
    magenta = palette.magenta;
    blue = palette.blue;
    grey = palette.grey;
    green = palette.green;
    navy = palette.navy;
    orange = palette.orange;
    yellow = palette.yellow;

    if n_result == 1
        colors_result = blue;
    elseif n_result == 2
        colors_result = [blue; magenta];
    elseif n_result == 3
        colors_result = [blue; green; magenta];
    else
        colors_result = get_color_map([blue; yellow; magenta;], n_result-1);
    end
    
    % Open figure
    fig = open_figure('size', [900, 600], 'name', window_name);
    
    t = tiledlayout(udim, 1);
    title(t, "Control Input Information");    

    for idx = 1:udim
        nexttile;
        for i_result = 1:n_result
            result = results{i_result};
            p = plot(result.stamps, result.controls(idx, :), 'DisplayName', result.legend);
            p.Color = colors_result(i_result, :);
            if isfield(result, 'Color')
                p.Color = result.Color;
            end
            if isfield(result, 'LineWidth')
                p.LineWidth = result.LineWidth;
            end
            if isfield(result, 'LineStyle')
                p.LineStyle = result.LineStyle;
            end
            hold on;       
        end
    ylabel(ylabel_descriptions(idx), "Interpreter", "latex");
    xlabel("$t_{sec}$", "Interpreter", "latex");
    grid on;
    title(title_descriptions(idx), 'Interpreter', 'latex');
    end
    legend;
end