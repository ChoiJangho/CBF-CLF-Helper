function [value, isterminal, direction] = rabbit_event(obj,t, s)
% Define the event here to partition the simulation
% according to event
% RABBIT_EVENT: solve ode45 until the Rabbit's LeftToe
% touches ground
% Based on left toe position

x = s(1:7);
p_lt = p_LeftToe(x);
value = p_lt(3);

isterminal = 1;
direction = -1;

end

