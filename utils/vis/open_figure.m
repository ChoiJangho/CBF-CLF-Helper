function fig = open_figure(varargin)
%% fig = open_figure(fig_num, 'property1',value1,'property2',value2,...)
%% fig = open_figure('property1',value1,'property2',value2,...)
% 'name': name of the figure (no need to be specified).
% 'size': [width height] (Default: [1152 648], Unit: point)
%       'full': 
% 'background_color': Default: 'white';
% 'font_size': Default: 12 (this is used to compute the margin)
% 'font_name': Default: 'Times New Roman'
% 'margin': [margin_width, margin_height] (by ratio) 
%       This can be specified by string options:
%       'tight', 'small' (Small margin, good for paper figures)
%       'normal', 'middle' (Default)
%       'original', 'large', 'big' (Similar to Matlab's default margin)
% 'interpreter': 'latex', 'tex', 'none', (Default: 'latex')
if mod(length(varargin),2) == 1
    fig_num = varargin{1};
    varargin = varargin(2:end);
end
settings = parse_function_args(varargin{:});
if ~isfield(settings, 'background_color')
    settings.background_color = 'white';
end
if ~isfield(settings, 'size')
    settings.size = [1152, 648];
end
if ~isfield(settings, 'font_size')
    settings.font_size = 12;
end
if ~isfield(settings, 'font_name')
    settings.font_name = 'Times New Roman';
end
if ~isfield(settings, 'interpreter')
    settings.interpreter = 'latex';
end
if ~strcmp(settings.interpreter, 'latex') && ...
        ~strcmp(settings.interpreter, 'tex') && ...
        ~strcmp(settings.interpreter, 'none')
    error("Undefined interpreter");
end
if ~isfield(settings, 'margin')
    settings.margin = 'normal';
end

if ischar(settings.size)
    if strcmp(settings.size, 'full')
        set(0, 'units', 'points');
        screen_size = get(0, 'ScreenSize');
        settings.size = screen_size(3:4);
        set(0, 'units', 'pixels');
    elseif strcmp(settings.size, 'half-full-width')
        set(0, 'units', 'points');
        screen_size = get(0, 'ScreenSize');
        settings.size = [floor(0.5 * screen_size(3)), screen_size(4)];
        set(0, 'units', 'pixels');        
    end
end

if ischar(settings.margin)
    if strcmp(settings.margin, 'tight') || strcmp(settings.margin, 'small')
        margin = [0.01 + 3 * settings.font_size / settings.size(1), ...
        0.01 + 2.5 * settings.font_size / settings.size(2)];
    elseif strcmp(settings.margin, 'normal') || strcmp(settings.margin, 'middle')
        margin = [0.05 + 3 * settings.font_size / settings.size(1), ...
        0.05 + 2.5 * settings.font_size / settings.size(2)];
    elseif strcmp(settings.margin, 'large') || strcmp(settings.margin, 'big') ...
            || strcmp(settings.margin, 'original')
        margin = [0.13, 0.11];
    else
        error("Undefined margin");
    end
else
    margin = settings.margin;
end

% Create figure
if exist('fig_num')
    fig = figure(fig_num);
else
    fig = figure;
end
% Assign name.
if isfield(settings, 'name')
    set(fig, 'Name', settings.name);
end
% Set up background color.
set(fig, 'Color', settings.background_color);
% Always use point as the unit for the size.
set(fig, 'Units', 'points');
% Set up size.
fig.Position(3:4) = settings.size;
% Set up margin.
set(fig, 'DefaultAxesPosition', [margin(1), margin(2), ...
    1 - 2 * margin(1) + settings.font_size / settings.size(1), ...
    1 - 2 * margin(2) + settings.font_size / settings.size(2)]);
% Set up font size.
set(fig, 'DefaultAxesFontSize', settings.font_size);
% Set up font name
set(fig, 'DefaultTextFontName', settings.font_name);
% Set up interpreter
set(fig, 'DefaultTextInterpreter', settings.interpreter);

movegui(fig, 'center');
movegui(fig, 'onscreen');

