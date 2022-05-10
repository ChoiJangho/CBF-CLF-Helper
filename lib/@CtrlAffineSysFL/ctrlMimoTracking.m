function [mu, extraout] = ctrlMimoTracking(obj, t, x, reference_trajectory, varargin)
% reference_trajectory: a function handle with input t, output xi_desired.
% xi_desired should have a cell structure of size (ydim, 1)
    if isempty(obj.K_mimo)
        error("obj.K_mimo has to be specified.");
    end
    
    y_d = reference_trajectory(t);
    mu = zeros(obj.udim, 1);
    for i = 1:obj.ydim
        xi_i = obj.xi_sym{i}(x);
        if i == 1
            e_i = xi_i - y_d{i}(1:end-1);
            fprintf("t: %.3f, e_x: %.3f e_dx: %.3f \n", [t, e_i(1), e_i(2)]);
        end
        mu(i) = y_d{i}(end) - obj.K_mimo{i} * (xi_i - y_d{i}(1:end-1));
    end
    extraout.xi_d = y_d;
    extraout.y_d = y_d;
end