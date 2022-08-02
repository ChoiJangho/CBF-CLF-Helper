%INITIALIZE.M
%
% Initialize the biped robot

%Eric Westervelt
%1/6/01

[a,b,m,dtheta_fixed] = controlParameters;

x0 = sigma(dtheta_fixed);

out = impact_abs_red(x0);

x0 = out(1:10);

if x0(1) == 0, error('whoops!! -- check INITIALIZE.M'), end