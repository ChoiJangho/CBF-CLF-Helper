function vs = eval_value(grid, table, xs, interp_method)
% v = eval_u(g, datas, x)
%   Computes the interpolated value of the value functions datas at the
%   states xs
% 
% Inputs:
%   Option 1: Single grid, single value function, multiple states
%     gs    - a single grid structure
%     datas - a single matrix (look-up stable) representing the value
%             function
%     xs    - set of states; each row is a state
%
%   Option 2: Single grid, multiple value functions, single state
%     gs    - a single grid structure
%     datas - a cell structure of matrices representing the value function
%     xs    - a single state
%
%   Option 3: Multiple grids, value functions, and states. The number of
%             grids, value functions, and states must be equal under this
%             option
%     gs    - a cell structure of grid structures
%     datas - a cell structure of matrices representing value functions
%     xs    - a cell structure of states
%
% Mo Chen, 2016-05-18

if nargin < 4
  interp_method = 'linear';
end

if size(xs, 2) ~= grid.dim
  if size(xs, 1) == grid.dim
    % Take transpose if number of input rows is same as grid dimension
    xs = xs';
  else
    error('Input points must have the same dimension as grid!')
  end
end


if isstruct(grid) && isnumeric(table) && ismatrix(xs)
  % Option 1
  vs = eval_u_single(grid, table, xs, interp_method);  
elseif isstruct(grid) && iscell(table) && isvector(xs)
  % Option 2
  if length(size(table)) > 2
      error("does not support table cell array more than 2 dimensional.");      
  elseif length(size(table)) == 2
      vs = zeros([size(table), size(xs, 1)]);
      for i = 1:size(table, 1)
          for j = 1:size(table, 2)
              vs(i, j, :) = eval_u_single(grid, table{i, j}, xs, interp_method);
          end
      end              
  else
      vs = zeros(length(table), size(xs, 1));
      for i = 1:length(table)
        vs(i, :) = eval_u_single(grid, table{i}, xs, interp_method);
      end        
  end
else
  error('Unrecognized combination of input data types!')
end
end

function v = eval_u_single(grid, table, xs, interp_method)
xs = clip_periodic_xs(grid, xs);
for i = 1:grid.dim
    if isfield(grid, 'bdry') && isequal(grid.bdry{i}, @addGhostPeriodic)
        max_i = max(grid.vs{i});
        grid.vs{i} = [grid.vs{i}; max_i + grid.vs{i}(2) - grid.vs{i}(1)];
        indices = repmat({':'},1,grid.dim);
        indices{i} = 1;
        slice_to_add = table(indices{:});
        table = cat(i, table, slice_to_add);
    end
end

%% Interpolate
interpn_argin_x = cell(grid.dim, 1);
for i = 1:grid.dim
  interpn_argin_x{i} = xs(:,i);
end

v = interpn(grid.vs{:}, table, interpn_argin_x{:}, interp_method);
if isnan(v)
    error("xs out of grid boundary.");
end
end