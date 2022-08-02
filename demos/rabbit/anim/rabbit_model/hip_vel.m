function [vHh,vHv] = hip_vel(x)
% HIP_VEL    The hip velocity.
%    [vHh,vHv] = HIP_VEL(X)

%Eric Westervelt
%24-Mar-2001 10:46:13

[g,L1,L3,L4] = modelParameters;

vHh = L3.*cos(x(:,1)).*x(:,6)+L4.*cos(x(:,3)).*x(:,8);

vHv = L3.*sin(x(:,1)).*x(:,6)+L4.*sin(x(:,3)).*x(:,8);
