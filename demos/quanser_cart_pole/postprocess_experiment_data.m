function [fig, results] = postprocess_experiment_data(file_name_str)
load(file_name_str);
ts = xs.time;
xs = xs.signals.values';

ts_us = us.time;
us_raw = squeeze(us.signals.values);

us = interp1(ts_us, us_raw, ts, 'linear')';

params = get_predefined_parameter_set('QUANSER');
% Set up CLF-related parameters
params.clf.rate = 0.5;
params.weight_slack = 1e10;

params.k_cbf = 10;
params.x_lim = 0.35;
params.cbf.rate = 10;
params.u_max = 6;
params.u_min = -6;
dynsys = QuanserCartPole(params);

[fig, Eps, Bs, Vs, cbf_constraints] = plot_cart_pole_core_results(xs, us, ts, dynsys);

results.xs = xs;
results.ts = ts;
results.us = us;
results.Eps = Eps;
results.Bs = Bs;
results.Vs = Vs;
results.cbf_constraints = cbf_constraints;

save(strcat(file_name_str, '_postprocessed'), 'xs', 'ts', 'us', 'Eps', 'Bs', 'Vs', 'cbf_constraints');
save_figure(fig, 'file_name',strcat(file_name_str, '_result'), 'file_format', 'pdf', 'figure_size', [12, 18]);
end