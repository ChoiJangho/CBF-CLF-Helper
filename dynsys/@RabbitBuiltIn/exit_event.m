function [exit_flag, last_valid] = exit_event(obj, xs)
% Exit function
% Argument
%   xs : trajectory of the roobt
% Output
%   exit_flag: judge if robot has fallen
%   last_valid: return the last valid index of the trajectory

height_threshold = obj.fall_threshold;
height_trajectory = xs(2, :);
last_valid = find(height_trajectory<height_threshold, 1, 'first')-1;
if isempty(last_valid)
    exit_flag = false;
else
    exit_flag = true;
end

end