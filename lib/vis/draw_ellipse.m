function h = draw_ellipse(major_start_coord, major_end_coord, eccentricity, varargin)
%% Draw the ellipsoid
% Argument
%   major start coord: (x1, y1)
%   major end coord: (x2, y2)
%   eccentricity: 0<e<1

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

x1 = major_start_coord(1); y1 = major_start_coord(2);
x2 = major_end_coord(1); y2 = major_end_coord(2);

numPoints = 3600; % Less for a coarser ellipse, more for a finer resolution.
% Make equations:
a = (1/2) * sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2);
b = a * sqrt(1-eccentricity^2);
t = linspace(0, 2 * pi, numPoints); % Absolute angle parameter
X = a * cos(t);
Y = b * sin(t);
% Compute angles relative to (x1, y1).
angles = atan2(y2 - y1, x2 - x1);
x = (x1 + x2) / 2 + X * cos(angles) - Y * sin(angles);
y = (y1 + y2) / 2 + X * sin(angles) + Y * cos(angles);
% Plot the ellipse as a blue curve.
h = fill(x, y, face_color, 'FaceAlpha',face_alpha, "Visible", Visible);
% plot(x,y,'b-', 'LineWidth', 2);	% Plot ellipse
h.LineStyle = line_style;
h.LineWidth = line_width;
h.EdgeColor = edge_color;

% grid on;
% axis equal
% % Plot the two vertices with a red spot:
% hold on;
% plot(x1, y1, 'r.', 'MarkerSize', 25);
% plot(x2, y2, 'r.', 'MarkerSize', 25);
end

