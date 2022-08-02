init_settings_cart_pole;
verbose = 0;

with_slack = 1;
% initial state
% x0 = [0; 0; pi-0.1; 0;];
x0 = [0; 0; 0; 0;];
% simulation time
sim_t = 2.0;

%% PHASE 1

tune_params_grid = cell(4, 1);
% tune_params_grid{1} = 1:1:100; % Kp
% tune_params_grid{2} = 1:1:100; % Kd
% tune_params_grid{3} = 0.005:0.005:0.2; % eps
tune_params_grid{1} = [1, 2, 4, 8, 10, 16, 20, 25, 30, 32, 35, 40, 50, 60, 64]; % Kp
tune_params_grid{2} = [0.5, 1, 1.5, 2, 4, 8, 10, 16, 18, 20, 25, 32, 40]; % Kd
tune_params_grid{3} = 0.01:0.01:0.2; % eps
tune_params_grid{4} = 1:1:10; % clf rate

% tune_params_grid{1} = 24:0.2:35; % Kp
% tune_params_grid{2} = 16:1:42; % Kd
% tune_params_grid{3} = 0.01:0.005:0.2; % eps
% tune_params_grid{4} = 1:1:10; % clf rate

tune_meshgrids = cell(4, 1);
[tune_meshgrids{:}] = ndgrid(tune_params_grid{:});
tune_params_grid = [];
for i = 1:4
    tune_params_grid = [tune_params_grid, tune_meshgrids{i}(:)];
end

hyp = struct('mean', [], 'cov', [log(10.0); log(1.2)], 'lik', log(1e-2));
likfunc = @likGauss; 
covfunc = @covSEiso; 
lik = log(0.1);
gp_handle = GPHandleKernelBaseline(4, covfunc, hyp, likfunc, lik);

eval_params = [28, 25, 0.15, 1];
eval_loss = [];
num_iter = 80;
beta = 1.0;
tune_params_grid_original = tune_params_grid;

for k = 1:num_iter
    disp(k);
    params.Kp_FL = eval_params(end, 1);
    params.Kd_FL = eval_params(end, 2);
    params.epsilon_FL = eval_params(end, 3);
    params.clf.rate = eval_params(end, 4);
    control_sys = CartPole(params);
    [xs, us, ts, Vs, ~, ys] = rollout_feedback_linearization( ...
        x0, plant_sys, control_sys, @control_sys.ctrlClfQpFL, dt, sim_t, with_slack, [], verbose);
    loss_out = bo_loss_func(ys(end), xs(end, :));
    eval_loss = [eval_loss; loss_out];
    
    gp_handle.update_training_data(eval_params, eval_loss);
    [zs_mu, zs_s2] = gp_handle.get_posterior(tune_params_grid);
    acquisition_vals = zs_mu + beta * sqrt(zs_s2);
    [~, idx] = max(acquisition_vals);
    new_eval_params = tune_params_grid(idx, :);
    tune_params_grid = tune_params_grid([1:idx-1,idx+1:end], :);
    eval_params = [eval_params; new_eval_params];
end

[zs_mu, zs_s2] = gp_handle.get_posterior(tune_params_grid_original);
[~, max_indices] = maxk(zs_mu, 10);
disp("Phase 1: Best tuning parameters:");
max_tune_params = tune_params_grid_original(max_indices, :);
disp(max_tune_params);

[hyp_trained, lik_trained] = gp_handle.get_optimized_hyperparameters(hyp, lik);
disp("New hyperparams suggeestions")
disp(hyp_trained.cov)
disp(lik_trained)

disp("PHASE 2 initiating.");

%% PHASE 2

new_grid_max = max(max_tune_params);
new_grid_min = min(max_tune_params);
for i = 1:4
    if (new_grid_min(i) == new_grid_max(i))
        new_grid_min(i) = new_grid_min(i) / 2;
        new_grid_max(i) = new_grid_min(i) * 3;
    end
end

tune_params_grid = cell(4, 1);
tune_params_grid{1} = new_grid_min(1):0.5:new_grid_max(1); % Kp
tune_params_grid{2} = new_grid_min(2):0.5:new_grid_max(2); % Kd
tune_params_grid{3} = new_grid_min(3):0.005:new_grid_max(3); % eps
tune_params_grid{4} = new_grid_min(4):0.1:new_grid_max(4); % eps

tune_meshgrids = cell(4, 1);
[tune_meshgrids{:}] = ndgrid(tune_params_grid{:});
tune_params_grid = [];
for i = 1:4
    tune_params_grid = [tune_params_grid, tune_meshgrids{i}(:)];
end

gp_handle = GPHandleKernelBaseline(4, covfunc, hyp_trained, likfunc, lik_trained);

eval_params = eval_params(end, :);
eval_loss = [];
num_iter = 20;
beta = 1.0;
tune_params_grid_original = tune_params_grid;

for k = 1:num_iter
    disp(k);
    params.Kp_FL = eval_params(end, 1);
    params.Kd_FL = eval_params(end, 2);
    params.epsilon_FL = eval_params(end, 3);
    params.clf.rate = eval_params(end, 4);
    control_sys = CartPole(params);
    [xs, us, ts, Vs, ~, ys] = rollout_feedback_linearization( ...
        x0, plant_sys, control_sys, @control_sys.ctrlClfQpFL, dt, sim_t, with_slack, [], verbose);
    loss_out = bo_loss_func(ys(end), xs(end, :));
    eval_loss = [eval_loss; loss_out];
    
    gp_handle.update_training_data(eval_params, eval_loss);
    [zs_mu, zs_s2] = gp_handle.get_posterior(tune_params_grid);
    acquisition_vals = zs_mu + beta * sqrt(zs_s2);
    [~, idx] = max(acquisition_vals);
    new_eval_params = tune_params_grid(idx, :);
    tune_params_grid = tune_params_grid([1:idx-1,idx+1:end], :);
    eval_params = [eval_params; new_eval_params];
end

[zs_mu, zs_s2] = gp_handle.get_posterior(tune_params_grid_original);
[~, max_indices] = maxk(zs_mu, 10);
disp("Phase 2: Best tuning parameters:");
max_tune_params = tune_params_grid_original(max_indices, :);
disp(max_tune_params);