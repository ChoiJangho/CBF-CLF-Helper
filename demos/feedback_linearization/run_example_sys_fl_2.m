%% Comparison to backstepping control.
clear all
close all
feedback_gain = [3, 2];
dynsys = ExampleSysFl2(feedback_gain);

x0 = [2; 0];

fl_controller = @(t, x, varargin) dynsys.ctrlFeedbackLinearize( ...
   t, x, @dynsys.ctrlSisoLinearFeedback, varargin{:});
backstepping_controller = @(t, x, varargin) dynsys.ctrlBackstepping(t, x, varargin{:});

t_max = 8;
[xs_fl, us_fl, ts_fl, extraout] = rollout_controller(x0, dynsys, fl_controller, t_max);
[xs_bs, us_bs, ts_bs, extraout] = rollout_controller(x0, dynsys, backstepping_controller, t_max);

x1_test = -2:0.1:2;
u_fl_test = zeros(size(x1_test));
u_bs_test = zeros(size(x1_test));
for i=1:length(x1_test)
    x1 = x1_test(i);
    x0 = [x1; 0];
    [u_fl_test(i), ~] = fl_controller(ts_fl(i), x0);
    [u_bs_test(i), ~] = backstepping_controller(ts_bs(i), x0);

end


open_figure('font_size', 16);
subplot(3, 1, 1);
plot(ts_fl, xs_fl(1, :));
ylabel('$x_1$');
title('Feedback Linearization');
subplot(3, 1, 2);
plot(ts_fl, xs_fl(2, :));
ylabel('$x_2$');
subplot(3, 1, 3);
plot(ts_fl, us_fl);
ylabel('$u$');
xlabel('$t$');

open_figure('font_size', 16);
subplot(3, 1, 1);
plot(ts_bs, xs_bs(1, :));
ylabel('$x_1$');
title('Backstepping');
subplot(3, 1, 2);
plot(ts_bs, xs_bs(2, :));
ylabel('$x_2$');
subplot(3, 1, 3);
plot(ts_bs, us_bs);
ylabel('$u$');
xlabel('$t$');


open_figure('font_size', 16);
subplot(2, 1, 1);
plot(x1_test, u_fl_test);
ylabel('$u_{fl}$');
xlabel('$x_1$');
subplot(2, 1, 2);
plot(x1_test, u_bs_test);
ylabel('$u_{bs}$');
xlabel('$x_1$');

% save_figure('file_name', 'exercise_2', 'figure_size', [9, 12], 'file_format', 'png');