init_settings_cart_pole;
verbose = 0;

with_slack = 1;
% initial state
% x0 = [0; 0; pi - 0.1; 0;];
x0 = [0; 0; 0.1; 0;];
% simulation time
sim_t = 3;


[xs, us, ts, Vs, ~, ~, ~, ~] = rollout_lyapunov_controller( ...
    x0, plant_sys, control_sys, @control_sys.ctrlClfQp, dt, sim_t, with_slack, [], verbose);
results.vanilla.trajectory = xs;
results.vanilla.controls = us;
results.vanilla.stamps = ts;

draw_rollout(results.vanilla, 0, dt, plant_sys, control_sys, "Vanilla")

figure;
plot(ts, Vs);