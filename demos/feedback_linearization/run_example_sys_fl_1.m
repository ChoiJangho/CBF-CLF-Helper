clear all
close all
feedback_gain = [3, 2];
dynsys = ExampleSysFl1(feedback_gain);

x0 = [1; 1; 1];

simple_fl_controller = @(t, x, varargin) dynsys.ctrlFeedbackLinearize( ...
    t, x, @dynsys.ctrlSisoLinearFeedback, varargin{:});

[xs, us, ts, extraout] = rollout_controller(x0, dynsys, simple_fl_controller, 5);
zs = cell2mat(extraout.z);

open_figure('font_size', 16);
subplot(5, 1, 1);
plot(ts, xs(1, :));
ylabel('$x_1$');
subplot(5, 1, 2);
plot(ts, xs(2, :));
ylabel('$x_2$');
subplot(5, 1, 3);
plot(ts, xs(3, :));
ylabel('$x_3$');

subplot(5, 1, 4);
plot(ts, zs);
ylabel('$\eta$');

subplot(5, 1, 5);
plot(ts, us);
ylabel('$u$');
xlabel('$t$');

save_figure('file_name', 'exercise_2', 'figure_size', [9, 12], 'file_format', 'png');