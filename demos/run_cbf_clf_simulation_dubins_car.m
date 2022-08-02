%% Example for multiple CBF constraints.
clear all;
close all;
dt = 0.0005a;
sim_t = 20;
x0 = [0;5;0];

params.v = 1; % velocity
params.u_max = 4; % max yaw rate (left)
params.u_min = -4; % min yaw rate (right)

% Obstacle position
params.xo = [5, 11];
params.yo = [4, 4];
% Obstacle radius
params.d = [2, 2];

params.cbf_gamma0 = 1;
% Desired target point
params.xd = 12;
params.yd = 0;

params.clf.rate = 0.5;
params.weight_slack = 1;

params.cbf.rate = 50;

dubins = DubinsCar(params);

weight_slack_for_cbfs = 100 * ones(size(params.d));

controller = @(x, varargin) dubins.ctrlCbfClfQp(x, ...
    'weight_slack', [params.weight_slack, weight_slack_for_cbfs], varargin{:});

[xs, us, ts, extraout] = rollout_controller( ...
    x0, dubins, dubins, controller, sim_t, 'dt', dt);
Bs = extraout.Bs;
plot_results(ts, xs, us, Bs, [params.xo;params.yo], params.d)

function plot_results(t, xs, us, Bs, p_o, r_o)

figure
subplot(3,1,1)
plot(t, xs(1, :))
xlabel('t')
ylabel('x [m]')

subplot(3,1,2)
plot(t, xs(2, :))
xlabel('t')
ylabel('y [m]')

subplot(3,1,3)
plot(t, xs(3, :))
xlabel('t')
ylabel('theta [rad]')


figure
plot(t, us)
xlabel('t')
ylabel('u [rad/s]')


lim_min = min(min(xs(1, :)), min(xs(2, :)));
lim_max = max(max(xs(1, :)), max(xs(2, :)));
lim_min = min([lim_min, p_o(1)-r_o, p_o(2)-r_o]);
lim_max = max([lim_max, p_o(1)+r_o, p_o(2)+r_o]);

figure
plot(xs(1, :), xs(2, :)); hold on;
for i = 1:size(p_o, 2)
    draw_circle(p_o(:, i), r_o(i));
end
xlim([lim_min, lim_max]);
ylim([lim_min, lim_max]);
xlabel('x [m]')
ylabel('y [m]')
axis equal

figure
for i = 1:size(Bs, 1)
    subplot(size(Bs, 1), 1, i)
    plot(t, Bs(i, :))
    xlabel('t')
    ylabel('cbf B(s)');
end
end

function h = draw_circle(center,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = plot(xunit, yunit);
hold off

end