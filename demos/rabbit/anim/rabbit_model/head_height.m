function h = head_height(x)
% HEAD_HEIGHT    Head height.
%    H = HEAD_HEIGHT(X)

%Eric Westervelt
%24-Mar-2001 10:46:12

[g,L1,L3,L4] = modelParameters;

[n,m] = size(x);

if n ~= 1 & m ~= 1
  h = -L3.*cos(x(:,1))-L4.*cos(x(:,3))+L1.*cos(x(:,5));
else
  h = -L3*cos(x(1))-L4*cos(x(3))+L1*cos(x(5));
end
