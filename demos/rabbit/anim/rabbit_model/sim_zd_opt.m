function varargout = sim_zd_opt(t,x,flag,opt_param)
%SIM_ZD_OPT   Simulate the zero dynamics of the kneed biped walker
%   for closed-loop optimization purposes.

%Eric Westervelt
%3/13/01


switch flag
case ''                                 % Return dx/dt = dynamics(t,x).
   varargout{1} = f(t,x);
case 'events'                           % Return [value,isterminal,direction].
   [varargout{1:3}] = events(t,x);
otherwise
   error(['Unknown flag ''' flag '''.']);
end


% --------------------------------------------------------------------------
% This is the system dynamics function

function dz = f(t,z)

dz = zd(z);

% --------------------------------------------------------------------------
% This is the events function

function [value,isterminal,direction] = events(t,z)
%Locate the time when critial angle of stance leg minus stance leg
%angle passes through zero in a decreasing direction and stop
%integration.

global SIM_COUNTER
global T_STOP
global STEP_LEN

SIM_COUNTER = SIM_COUNTER + 1;

dz = zd(z);
x = zd_lift(z(1),dz(1)).';

if SIM_COUNTER > 200 & isempty(T_STOP)
  T_STOP = t;
  value(9) = 1;
elseif ~isempty(T_STOP) & abs(rem(t,T_STOP)) < 1e-3
  value(9) = 1;
else
  value(9) = 0;
end

[pHh,pHv]=hip_pos(x);

h = swing_foot_height(x');

value(1) = h+.005; % this will simulate for a longer time
		   % guaranteeing that the swing foot will contact
		   % the ground
value(2) = x(5) - pi/3;        % torso not too far backward
value(3) = x(5) + pi/3;        % torso not too far forward
value(4) = x(2) - 100/180*pi;  % swing femur not too far back
value(5) = x(2) - 260/180*pi;  % swing femur not too far forward
value(6) = pHv - .6;           % hips not too low
value(7) = x(1) - x(3) - 3*pi/4; % stance knee not too bent
value(8) = x(2) - x(4) - 3*pi/4; % swing knee not too bent
value(10) = h; % this is the actual touching of the foot with the
               % ground
value(11) = 1.2*STEP_LEN - step_length(x');

isterminal(1) = 1;  % stop when this event occurs
isterminal(2) = 1;  % stop  "
isterminal(3) = 1;  % stop  "
isterminal(4) = 1;  % stop  "
isterminal(5) = 1;  % stop  "
isterminal(6) = 1;  % stop  "
isterminal(7) = 1;  % stop  "
isterminal(8) = 1;  % stop  "
isterminal(9) = 1;  % stop  "
isterminal(10) = 0;
isterminal(11) = 1;

direction(1) = -1;  % decreasing direction
direction(2) = 0;  % either
direction(3) = 0;  % either
direction(4) = 0;  % either
direction(5) = 0;  % either
direction(6) = 0;  % either
direction(7) = 0;  % either
direction(8) = 0;  % either
direction(9) = 0;  % either
direction(10) = -1;
direction(11) = 0;