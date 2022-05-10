function [u, extraout] = ctrlFeedbackLinearizeMimo(obj, t, x, mu, verbose, extraout)
if ~isempty(obj.z_sym)
    z = obj.z_sym(x);
else
    z = [];
end

Lfs_y = zeros(obj.ydim, 1);
y = zeros(obj.ydim, 1);
xi = cell(obj.ydim, 1);
for i = 1:obj.ydim
   xi_i = obj.xi_sym{i}(x);
   y(i) = xi_i(1);
   xi{i} = xi_i;
   Lfs_y(i) = obj.lfs_y_sym{i}{end}(x);
end

A = obj.decoupling_matrix_sym(x);

feedforward = -A\Lfs_y;
u_raw = feedforward + A\mu;
u = obj.clipInput(u_raw);

extraout.mu = mu;
extraout.y = y;
extraout.xi = xi;
extraout.feedforward = feedforward;
extraout.u_raw = u_raw;
extraout.z = z;