function [u, extraout] = ctrl_pursue_target(obj, t, x, varargin)

kwargs = parse_function_args(varargin{:});

if ~isfield(kwargs, 'target')
    target = zeros(2, 1);
else
    target = kwargs.target;
end

if size(target, 2) > 1
    num_target = size(target, 2);
    if ~isfield(kwargs, 'time_per_target')
        error("'time_per_target' must be given if there are multiple target points.");
    end
    time_per_target = kwargs.time_per_target;
    if isfield(kwargs, 'periodic')
        periodic = kwargs.periodic;
    else
        periodic = false;
    end
    
    index_t = floor(t / time_per_target);
    if periodic
        index_target = mod(index_t, num_target) + 1;
    else
        index_target = min(index_t + 1, num_target);
    end
    target_i = target(:, index_target);
else
    target_i = target;
end

if ~isfield(kwargs, 'v_ref')
    error("'v_ref' must be given as additional arugment to evaluate the acceleration.");
end
v_ref = kwargs.v_ref;

p_x = x(1);
p_y = x(2);
theta = x(3);
v = x(4);

% Determine acceleration.
if v < v_ref
    % apply maximal accelaration.
    a = obj.u_max(2);
elseif v < v_ref && v > 0
    a = obj.u_min(2);
else
    a = 0;
end

dx = target_i(1) - p_x;
dy = target_i(2) - p_y;
d = sqrt(dx^2 + dy^2);
if d < 0.001
    if v > 0
        a = obj.u_min(2);
    end
    omega = 0;
    u = [omega; a];
    extraout.distance_to_target = d;
    return
end
% smallest turning radius
R = v / obj.u_max(1);
if cos(theta) * dy - sin(theta) * dx > 0
    % target is on the left side of the vehicle.
    max_turn_center = [p_x - R * sin(theta);
        p_y + R * cos(theta)];
    if (target_i(1) - max_turn_center(1))^2 + (target_i(2) - max_turn_center(2))^2 < R^2
        % if target is closer to the center of rotation than the smallest
        % turning radius, reduce the speed so that the turning radius
        % decrease.        
        a = obj.u_min(2);
        % turn steep left.
        omega = obj.u_max(1);
%         omega = 0;
    else
        sin_dtheta = (cos(theta) * dy - sin(theta) * dx) / sqrt(dx^2 + dy^2);
        cos_dtheta = (cos(theta) * dx + sin(theta) * dy) / sqrt(dx^2 + dy^2);
        % turn_radius = d / sqrt(1 - cos_dtheta^2);
        if cos_dtheta < 0
            omega = obj.u_max(1);
        else
            turn_radius = d / (2 * sin_dtheta);
            omega = v / turn_radius;
        end
    end
% elseif cos(theta) * dy - sin(theta) * dx < -0.01
else
    % target is on the right side of the vehicle.
    max_turn_center = [p_x + R * sin(theta);
        p_y - R * cos(theta)];
    if (target_i(1) - max_turn_center(1))^2 + (target_i(2) - max_turn_center(2))^2 < R^2
        % if target is closer to the center of rotation than the smallest
        % turning radius, reduce the speed so that the turning radius
        % decrease.        
        a = obj.u_min(2);
        % turn steep right.
        omega = obj.u_min(1);
%         omega = 0;
    else
        sin_dtheta = (sin(theta) * dx - cos(theta) * dy) / sqrt(dx^2 + dy^2);
        cos_dtheta = (cos(theta) * dx + sin(theta) * dy) / sqrt(dx^2 + dy^2);
        % turn_radius = d / sqrt(1 - cos_dtheta^2);
        if cos_dtheta < 0
            omega = obj.u_min(1);
        else
            turn_radius = d / (2 * sin_dtheta);
            omega = -v / turn_radius;
        end
    end
end
 
u = [omega; a];
extraout.distance_to_target = d;