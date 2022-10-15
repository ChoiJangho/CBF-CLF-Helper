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

th = 0:pi/300:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = fill(xunit, yunit, face_color, 'FaceAlpha',face_alpha);
hold off
end