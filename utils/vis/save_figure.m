function save_figure(varargin)
%% save_figure(fig, 'file_name', 'cool_figure', 'property1', value1, ...)
%% save_figure('file_name', 'cool_figure', 'property1', value1, ...)
%% save current figure to a image file.
%% keywords:
%   file_name: name of the file in string.
%   file_format: 'fig' 'png' 'jpg' 'pdf' (Default: 'fig')
%   figure_size: [width height] (Default: [16 9], Unit: inch)
if mod(length(varargin),2) == 1
    fig = varargin{1};
    varargin = varargin(2:end);
else
    fig = gcf;
end

settings = parse_function_args(varargin{:});
if ~isfield(settings, 'file_name')
    error("file_name must be provided.");
end
if ~isfield(settings, 'file_format')
    settings.file_format = 'fig';
end
if ~isfield(settings, 'figure_size')
    settings.figure_size = [16 9];
end
plot_position = [0, 0, settings.figure_size];

set(fig, 'PaperPositionMode', 'manual');
set(fig, 'PaperUnits', 'inches');
set(fig, 'PaperSize', settings.figure_size);
set(fig, 'PaperPosition', plot_position);
saveas(fig, settings.file_name, settings.file_format);
