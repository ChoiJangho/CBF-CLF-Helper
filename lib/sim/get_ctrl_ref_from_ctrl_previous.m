function ctrl_ref = get_ctrl_ref_from_ctrl_previous(~, ctrl_previous, ...
    ratio_ctrl_diff)
%% ctrl_ref = get_ctrl_ref_from_ctrl_previous(x, ctrl_previous, ratio_ctrl_diff)
%   ratio_ctrl_diff: ratio(0.0~1.0) in the min-norm objective for minimizing the
%   difference between u(mu) and u_prev(mu_prev) (default 0.0)
%       if 0.0 : minimizes the norm of plain u.
%       if 1.0: minimize the norm of plain (u_k-u_{k-1})
    if nargin < 3
        ratio_ctrl_diff = 1;
    end
    if ratio_ctrl_diff > 1 || ratio_ctrl_diff < 0
        error("ratio_ctrl_diff should be a value between 0 and 1.");
    end    
    ctrl_ref = ratio_ctrl_diff * ctrl_previous;
end