function x = zd_lift_special(z)
%ZD_LIFT_SPECIAL   Lifts the zero dynamics to the full state of the
%system.
%
%    X = ZD_LIFT_SPECIAL(Z)

%Eric Westervelt
%3/9/01

[n,m] = size(z);

if n ~= 1 & m ~= 1
  dz = [];
  for k = 1:length(z)
    dz(k,:) = zd(z(k,:))';
    x(k,:) = zd_lift(z(k,1)',dz(k,1)')';
  end
else
  dz = zd(z)';
  x = zd_lift(z(1)',dz(1)')';
end
