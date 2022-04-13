function ax = set_axis(varargin)
%% ax = set_axis(ax, 'property1',value1,'property2',value2,...)
%% ax = set_axis('property1',value1,'property2',value2,...)
%% keywords:
% font_size: Unit: points
% font_name: example: 'Times New Roman';
% interpreter: 'latex', 'tex', 'none'
if mod(length(varargin),2) == 1
    ax = varargin{1};
    varargin = varargin(2:end);
else
    ax = gca;
end

settings = parse_function_args(varargin{:});
if isfield(settings, 'font_size')
    set(ax, 'FontSize', settings.font_size);
end
if isfield(settings, 'font_name')
    set(ax, 'FontName', settings.font_name);
end
if isfield(settings, 'interpreter')
    if ~strcmp(settings.interpreter, 'latex') && ...
            ~strcmp(settings.interpreter, 'tex') && ...
            ~strcmp(settings.interpreter, 'none')
        error("Undefined interpreter");
    end
    set(ax, 'DefaultTextInterpreter', settings.interpreter);
end
    
