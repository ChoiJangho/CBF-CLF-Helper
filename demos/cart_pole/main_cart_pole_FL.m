init_settings_cart_pole;
verbose = 1;

with_slack = 1;
% initial state
% x0 = [0; 0; pi-0.1; 0;];
x0 = [0; 0; 0; 0;];
% simulation time
sim_t = 1.65;

[xs, us, ts, Vs, ~, ys] = rollout_feedback_linearization( ...
    x0, plant_sys, control_sys, @control_sys.ctrlClfQpFL, dt, sim_t, with_slack, [], verbose);
results.vanilla.trajectory = xs;
results.vanilla.controls = us;
results.vanilla.stamps = ts;

phase = xs(:, 1) + 0.5*xs(:, 3);
c_normal = 2.37079632679490;
phase = phase / c_normal; 
y_d = control_sys.bezier(control_sys.params.bezier, phase);


figure
subplot(4, 2, 1);
plot(ts, xs(:, 1));
ylabel('x');

subplot(4, 2, 2);
plot(ts, xs(:, 3)); hold on;
plot(ts, y_d);
ylabel('theta');
legend('theta', 'y_d');

subplot(4, 2, 3);
plot(ts, xs(:, 2));
ylabel('dx');

subplot(4, 2, 4);
plot(ts, xs(:, 4));
ylabel('dtheta');

subplot(4, 2, 5);
plot(ts(1:end-1), us);
ylabel('u');

subplot(4, 2, 6);
plot(ts, Vs);
ylabel('V');

subplot(4, 2, 7);
plot(ts, phase);
ylabel('phase');

subplot(4, 2, 8);
plot(ts, ys);
ylabel('y_error');

% draw_rollout(results.vanilla, 0, dt, plant_sys, control_sys, "Vanilla")

function mu = dummy_controller(x, ~, ~)
    mu = 0;
end