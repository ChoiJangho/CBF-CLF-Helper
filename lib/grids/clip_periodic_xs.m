function xs_clipped = clip_periodic_xs(grid, xs)
if size(xs, 2) ~= grid.dim
  if size(xs, 1) == grid.dim
    % Take transpose if number of input rows is same as grid dimension
    xs = xs';
  else
    error('Input points must have the same dimension as grid!')
  end
end
    
%% Dealing with periodicity
xs_clipped = xs;
for i = 1:grid.dim
    if isfield(grid, 'bdry') && isequal(grid.bdry{i}, @addGhostPeriodic)
        max_i = max(grid.vs{i}) + grid.vs{i}(2) - grid.vs{i}(1);
        min_i = min(grid.vs{i});
        period = max_i - min_i;
        xs_i_clipped = mod(xs(:, i), period);
        xs_i_clipped(xs_i_clipped > max_i) = xs_i_clipped(xs_i_clipped > max_i) - period;
        xs_clipped(:, i) = xs_i_clipped;
    end
end
end