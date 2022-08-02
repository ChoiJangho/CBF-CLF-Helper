function [pHh,pHv] = hip_pos(x)
% HIP_POS    The hip position.
%    [pHv,pHh] = HIP_POS(X)

%Eric Westervelt
%24-Mar-2001 10:46:13

[g,L1,L3,L4] = modelParameters;

[n,m] = size(x);

if n ~= 1 & m ~= 1
  pHh = L3.*sin(x(:,1))+L4.*sin(x(:,3));
  pHv = -L3.*cos(x(:,1))-L4.*cos(x(:,3));
else
  pHh = L3*sin(x(1))+L4*sin(x(3));
  pHv = -L3*cos(x(1))-L4*cos(x(3));
end
