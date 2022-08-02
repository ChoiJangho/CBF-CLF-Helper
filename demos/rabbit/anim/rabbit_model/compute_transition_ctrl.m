%COMPUTE_TRANSITION_CTRL.M

%Eric Westervelt
%4/23/01

global WHICH_PARAM

from_param = 100;
to_param = 102;

WHICH_PARAM = from_param;
[a1,b1,m1,dtheta_fixed1] = controlParameters;

% get velocities for matching up
x0 = sigma(dtheta_fixed1);
x0 = impact_abs_red(x0); 
X_START = x0(1:10);

WHICH_PARAM = to_param;
[a2,b2,m2,dtheta_fixed2] = controlParameters;

% get velocities for matching up
x_end = sigma(dtheta_fixed2);
Q_END = x_end(1:5);


% calculate m and b scaling parameters -- they're uniquely determinted
% by imposing posture at beginning and end of step
theta_start = sum(X_START([1;3]))/2;
theta_end = sum(Q_END([1;3]))/2;
m = theta_end - theta_start;
b = theta_start;

[p1,p5] = compute_p1_p5(m,from_param,to_param);

params = [a1(5); p1(1);
	  (a1(7:9)+a2(7:9))/2;
	  p5(1); a2(11);
	  a1(12); p1(2);
	  (a1(14:16)+a2(14:16))/2;
	  p5(2); a2(18);
	  a1(19); p1(3);
	  (a1(21:23)+a2(21:23))/2;
	  p5(3); a2(25);
	  a1(26); p1(4);
	  (a1(28:30)+a2(28:30))/2;
	  p5(4); a2(32)];

disp('[copy this stuff to CONTROLPARAMETERS.M]');
disp(' ');
disp(['  m = ',num2str(m),';']);
disp(['  b = ',num2str(b),';']);
disp(' ');

a = char('a');
for counter = 1:4
  % print out in a format to be copied to CONTROLPARAMETERS.M
  %
  b = [char(a+counter-1) char(a+counter-1)];
  disp(['  ',b,' = [',num2str(params((counter-1)*7+1)),'; ', ...
	num2str(params((counter-1)*7+2)),'; ', ...
	num2str(params((counter-1)*7+3)),'; ', ...
	num2str(params((counter-1)*7+4)),'; ',]);
  disp(['        ',num2str(params((counter-1)*7+5)),'; ', ...
	num2str(params((counter-1)*7+6)),'; ', ...
 	num2str(params((counter-1)*7+7)),'];']);
end