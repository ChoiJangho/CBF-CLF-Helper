function [u, extraout] = ctrlFeedbackLinearizeSiso(obj, t, x, mu, verbose, extraout)
xi = obj.xi_sym(x);
if ~isempty(obj.z_sym)
    z = obj.z_sym(x);
else
    z = [];
end
Lfs_y = obj.lfs_y_sym{end}(x);
Lglfr_y = obj.lglfr_y_sym(x);

feedforward = - Lfs_y / Lglfr_y;
u_raw = feedforward + mu / Lglfr_y;

u = obj.clipInput(u_raw);

extraout.mu = mu;
extraout.y = xi(1);
if obj.rel_deg_y > 1
extraout.dy = xi(2);
end
extraout.xi = xi;
extraout.z = z;
extraout.feedforward = feedforward;
extraout.u_raw = u_raw;