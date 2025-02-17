function animate_car4d_trajectory(time_stamps, trajectory, varargin)
%% animate_car4d_trajectory(time_stamps, trajectory, varargin)
% Animate the car 4d trajectory in video fashion.
% Note. Car is not circular, but is denoted as circular for convenience.
% Input
%   time_stamps
%   trajectory : trajectory
%   varargin
%       save_path
%       dt_frame
%       x_range
%       obstacle
kwargs = parse_function_args(varargin{:});
save_video = true;
if isfield(kwargs, 'save_dir')
    save_video = true;
    save_dir = kwargs.save_dir;
end

dt_frame = 0.2;
if isfield(kwargs, 'dt_frame')
    dt_frame = kwargs.dt_frame;
end

if isfield(kwargs, 'targets')
    targets = kwargs.targets;
else
    targets = [];
end
num_target = size(targets, 2);

if isfield(kwargs, 'time_per_target')
    time_per_target = kwargs.time_per_target;
else
    time_per_target = [];
end

if isfield(kwargs, 'cbfs')
    cbfs = kwargs.cbfs;
else
    cbfs = [];
end

if save_video
    extraArgs.videoFilename = ...
        [datestr(now,'YYYYMMDD_hhmmss') '.mp4'];
    vout = VideoWriter(extraArgs.videoFilename,'MPEG-4');
    vout.Quality = 100;
    vout.FrameRate = 1/dt_frame;
    vout.open;
end

%% Set Camera View Radius
x_min = min(trajectory(1,:)); x_max = max(trajectory(1, :));
y_min = min(trajectory(2,:)); y_max = max(trajectory(2, :));
margin = 1;

side = max(x_max - x_min, y_max - y_min) + margin;
x_center = (x_min+x_max)/2;
y_center = (y_min+y_max)/2;
xrange = [x_center-side/2, x_center+side/2, y_center-side/2, y_center+side/2];
if isfield(kwargs, 'x_range')
    xrange = kwargs.x_range;
end

%% Draw Obstacle
draw_obstacle = false;
if isfield(kwargs, 'obstacle')
    draw_obstacle = true;
    obstacle = kwargs.obstacle;
end

%% Draw Car
palette = get_palette_colors();

car_radius  = 0.5;
head_radius = 0.1;

% Load Color Map


%% Open figure
fig = open_figure('size', [900, 1200], 'name', "Trajectory");
t = tiledlayout(1, 1);
title(t, "Car 4d trajectory");    
nexttile;
xlabel("x"); ylabel("y"); grid on;

% (1) Draw Whole Trajectory (Fixed)
q = quiver(trajectory(1, 1:end-1), trajectory(2, 1:end-1), ...
                diff(trajectory(1, :)), diff(trajectory(2, :)), 0);
q.Color = palette.magenta;
q.DisplayName = "Whole Trajectory";
axis(xrange); axis square; grid on;
hold on;

% (2) Draw Obstacle (Fixed)
if draw_obstacle
    draw_circle(obstacle.center, obstacle.radius, ...
          'face_alpha', 0.2, 'face_color', palette.orange);
end
hold on;

traj_length = size(trajectory, 2);
dt = time_stamps(2) - time_stamps(1);

% (3) Draw Car 4D
% Dummy car, car_head plot
car = plot(0, 0); car_head = plot(0, 0); car_center = plot(0, 0);
for i = 1: traj_length
    if rem(time_stamps(i), dt_frame) >= dt
        continue
    end
    title_with_stamp = strcat("Car 4D Trajectory (T:", num2str(time_stamps(i)), "(s))");
    title(t, title_with_stamp);

    center = trajectory(1:2, i);
    theta  = trajectory(3, i);
    v      = trajectory(4, i);
    unit_vector = [cos(theta); sin(theta)];
    
    %% Draw CBF level set
    syms x y
    xo = obstacle.center(1);
    yo = obstacle.center(2);
    Ro = obstacle.radius;
    omega = 2;
    max_acc = 1;
    v_min = 1;
    v_max = 5;
    r_steer = v_max / omega;
    steering_distance = Ro * (sqrt(1 + 2*r_steer / (Ro * omega)) -1 );
    d = 0.5 * steering_distance;
    tau = 0.5 * v_max / max_acc;
    distance_margin = d + tau * (v - v_min);
            
    avoid_center = [-distance_margin * cos(theta) + xo;
        -distance_margin * sin(theta) + yo];
    avoid_radius = Ro + distance_margin;
    cbf = sqrt((x - avoid_center(1))^2 + (y - avoid_center(2))^2) - avoid_radius;    
    cbf_curve = fcontour(cbf, [-10, 10, -10, 10], 'r');
    cbf_curve.LevelList = 0.0;
    cbf_curve.LineWidth = 1.5;
    hold on;

    delete(car); delete(car_head); delete(car_center);
    if isempty(cbfs)
        car = draw_circle(center, car_radius, ...
            'face_alpha', 0.1, 'face_color', palette.blue);
    elseif cbfs(i) > 0
        car = draw_circle(center, car_radius, ...
            'face_alpha', 0.1, 'face_color', palette.blue);
    else
        car = draw_circle(center, car_radius, ...
            'face_alpha', 0.5, 'face_color', palette.magenta);
    end
        
    
    car_center = scatter(center(1), center(2), 40, 'x');
    car_center.MarkerEdgeColor = palette.blue;
    car_head = draw_circle(center + car_radius * unit_vector, head_radius, ...
        'face_alpha', 0.5, 'face_color', palette.green);
    
    %% Draw target
    if ~isempty(targets) && ~isempty(time_per_target)
        index_t = floor(time_stamps(i) / time_per_target);
            index_target = min(index_t + 1, num_target);
        current_target = targets(:, index_target);
        target = draw_circle(current_target, head_radius, ...
        'face_alpha', 0.5, 'face_color', palette.blue);
    end
    
    pause(dt_frame);

    if save_video
        current_frame = getframe(gcf);
        writeVideo(vout, current_frame);
    end
    delete(cbf_curve);
    if ~isempty(targets) && ~isempty(time_per_target)
        delete(target);
    end
end