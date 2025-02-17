function [cmh,cmv] = center_of_mass(x)
% CENTER_OF_MASS    Robot center of mass.
%    [CMH,CMV] = CENTER_OF_MASS(X)

%Eric Westervelt
%24-Mar-2001 10:46:13

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]= modelParameters;

[n,m] = size(x);

if n ~= 1 & m ~= 1
  cmh = (L3.*sin(x(:,1)).*M1+L4.*sin(x(:,3)).*M1-sin(x(:,5)).*MZ1+ ...
cos(x(:,5)).*MY1+2.*L3.*sin(x(:,1)).*M3+2.*L4.*sin(x(:,3)).*M3-MZ3.* ...
sin(x(:,1))-MZ3.*sin(x(:,2))+2.*L4.*sin(x(:,3)).*M4-MZ4.*sin(x(:,3))+ ...
L3.*sin(x(:,1)).*M4-L3.*sin(x(:,2)).*M4-MZ4.*sin(x(:,4)))/(M1+2.*M3+ ...
2.*M4);
  cmv = (-L3.*cos(x(:,1)).*M1-L4.*cos(x(:,3)).*M1+cos(x(:,5)).*MZ1+ ...
sin(x(:,5)).*MY1-2.*L3.*cos(x(:,1)).*M3-2.*L4.*cos(x(:,3)).*M3+MZ3.* ...
cos(x(:,1))+MZ3.*cos(x(:,2))-2.*L4.*cos(x(:,3)).*M4+MZ4.*cos(x(:,3))- ...
L3.*cos(x(:,1)).*M4+L3.*cos(x(:,2)).*M4+MZ4.*cos(x(:,4)))/(M1+2.*M3+ ...
2.*M4);
else
  cmh = (L3*sin(x(1))*M1+L4*sin(x(3))*M1-sin(x(5))*MZ1+cos(x(5))*MY1+2*L3* ...
sin(x(1))*M3+2*L4*sin(x(3))*M3-MZ3*sin(x(1))-MZ3*sin(x(2))+2*L4* ...
sin(x(3))*M4-MZ4*sin(x(3))+L3*sin(x(1))*M4-L3*sin(x(2))*M4-MZ4* ...
sin(x(4)))/(M1+2*M3+2*M4);
  cmv = (-L3*cos(x(1))*M1-L4*cos(x(3))*M1+cos(x(5))*MZ1+sin(x(5))*MY1-2*L3* ...
cos(x(1))*M3-2*L4*cos(x(3))*M3+MZ3*cos(x(1))+MZ3*cos(x(2))-2*L4* ...
cos(x(3))*M4+MZ4*cos(x(3))-L3*cos(x(1))*M4+L3*cos(x(2))*M4+MZ4* ...
cos(x(4)))/(M1+2*M3+2*M4);
end
