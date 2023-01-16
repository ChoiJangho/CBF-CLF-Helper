function h = draw_circle(center,r, varargin)
%% Draw Circle 2D
% Input
%   r: radius of the circle (scalar)
%   center: center of the circle (2-dim vector)
%   varargin
%       face_color : color of the circle
%       face_alpha : transparency of the circle
kwargs = parse_function_args(varargin{:});

face_alpha = 1;
if isfield(kwargs, "face_alpha")
    face_alpha = kwargs.face_alpha;
end

face_color = [0.7, 0, 0];
if isfield(kwargs, 'face_color')
    face_color = kwargs.face_color;
end

line_style = "-";
if isfield(kwargs, 'line_style')
    line_style = kwargs.line_style;
end

line_width = 1;
if isfield(kwargs, 'line_width')
    line_width = kwargs.line_width;
end

edge_color = [0, 0, 0];
if isfield(kwargs, 'edge_color')
    edge_color = kwargs.edge_color;
end

Visible = "on";
if isfield(kwargs, "Visible")
    Visible = kwargs.Visible;
end

th = 0:pi/300:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);

h = fill(xunit, yunit, face_color, 'FaceAlpha',face_alpha, "Visible", Visible);
h.LineStyle = line_style;
h.LineWidth = line_width;
h.EdgeColor = edge_color;


end