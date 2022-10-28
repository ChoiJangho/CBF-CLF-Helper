function [value, is_terminal, direction] = event_failure(obj, t, x)
%% Event Failure
% We consider it a failure if the car hits the obstacle.
value = norm(x(1:2) - [obj.params.xo; obj.params.yo]) - obj.params.Ro;
is_terminal = 1;
direction   = -1;
end

