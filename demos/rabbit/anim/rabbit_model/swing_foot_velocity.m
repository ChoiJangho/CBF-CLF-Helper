function [vh,vv] = swing_foot_velocity(x)
% SWING_FOOT_VELOCITY    The swing foot vertical velocity.
%    [VH,VV] = SWING_FOOT_VELOCITY(X)

%Eric Westervelt
%24-Mar-2001 10:46:13

[g,L1,L3,L4] = modelParameters;

[n,m] = size(x);

if n ~= 1 & m ~= 1
  vh = L3.*cos(x(:,1)).*x(:,6)-L3.*cos(x(:,2)).*x(:,7)+L4.*cos(x(:,3)).*x(:,8)-L4.*cos(x(:,4)).*x(:,9);
  vv = L3.*sin(x(:,1)).*x(:,6)-L3.*sin(x(:,2)).*x(:,7)+L4.*sin(x(:,3)).*x(:,8)-L4.*sin(x(:,4)).*x(:,9);
else
  vh = L3*cos(x(1))*x(6)-L3*cos(x(2))*x(7)+L4*cos(x(3))*x(8)-L4*cos(x(4))*x(9);
  vv = L3*sin(x(1))*x(6)-L3*sin(x(2))*x(7)+L4*sin(x(3))*x(8)-L4*sin(x(4))*x(9);
end