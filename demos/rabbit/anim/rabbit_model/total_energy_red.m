function E = total_energy_red(x)
% TOTAL_ENERGY_RED    Total energy of robot, E = KE + PE.
%    E = TOTAL_ENERGY_RED(X)

%Eric Westervelt
%24-Mar-2001 10:46:13

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]= modelParameters;

[n,m] = size(x);

if n ~= 1 & m ~= 1
  E = -x(:,10).*MY1.*L4.*x(:,8).*sin(x(:,5)-x(:,3))-x(:,10).*MY1.*L3.* ...
x(:,6).*sin(x(:,5)-x(:,1))-MZ3.*x(:,6).*L4.*x(:,8).*cos(-x(:,1)+ ...
x(:,3))-x(:,10).*MZ1.*L3.*x(:,6).*cos(x(:,5)-x(:,1))-x(:,10).*MZ1.* ...
L4.*x(:,8).*cos(x(:,5)-x(:,3))-MZ3.*x(:,7).*L3.*x(:,6).*cos(x(:,2)- ...
x(:,1))-MZ3.*x(:,7).*L4.*x(:,8).*cos(-x(:,2)+x(:,3))-MZ4.*x(:,9).*L4.* ...
x(:,8).*cos(-x(:,4)+x(:,3))+MZ4.*x(:,9).*L3.*x(:,7).*cos(-x(:,4)+ ...
x(:,2))-MZ4.*x(:,9).*L3.*x(:,6).*cos(-x(:,4)+x(:,1))+M1.*L3.*x(:,6).* ...
L4.*x(:,8).*cos(-x(:,1)+x(:,3))-M4.*L3.^2.*x(:,6).*x(:,7).*cos(x(:,2)- ...
x(:,1))+2.*M3.*L3.*x(:,6).*L4.*x(:,8).*cos(-x(:,1)+x(:,3))-M4.*L3.* ...
x(:,7).*L4.*x(:,8).*cos(-x(:,2)+x(:,3))+M4.*L3.*x(:,6).*L4.*x(:,8).* ...
cos(-x(:,1)+x(:,3))+1/2.*M4.*L3.^2.*x(:,6).^2+M3.*L4.^2.*x(:,8).^2- ...
MZ3.*x(:,6).^2.*L3+M3.*L3.^2.*x(:,6).^2-MZ4.*x(:,8).^2.*L4+M4.*L4.^2.* ...
x(:,8).^2+1/2.*M1.*L4.^2.*x(:,8).^2+1/2.*M1.*L3.^2.*x(:,6).^2+1/2.* ...
M4.*L3.^2.*x(:,7).^2+1/2.*XX3.*x(:,6).^2+1/2.*XX4.*x(:,9).^2+1/2.* ...
XX4.*x(:,8).^2+1/2.*XX1.*x(:,10).^2+1/2.*XX3.*x(:,7).^2+g.*(-L3.* ...
cos(x(:,1)).*M1-L4.*cos(x(:,3)).*M1+cos(x(:,5)).*MZ1+sin(x(:,5)).*MY1- ...
2.*L3.*cos(x(:,1)).*M3-2.*L4.*cos(x(:,3)).*M3+MZ3.*cos(x(:,1))+MZ3.* ...
cos(x(:,2))-2.*L4.*cos(x(:,3)).*M4+MZ4.*cos(x(:,3))-L3.*cos(x(:,1)).* ...
M4+L3.*cos(x(:,2)).*M4+MZ4.*cos(x(:,4)));
else
  E = -x(10)*MY1*L4*x(8)*sin(x(5)-x(3))-x(10)*MY1*L3*x(6)*sin(x(5)-x(1))- ...
MZ3*x(6)*L4*x(8)*cos(-x(1)+x(3))-x(10)*MZ1*L3*x(6)*cos(x(5)-x(1))- ...
x(10)*MZ1*L4*x(8)*cos(x(5)-x(3))-MZ3*x(7)*L3*x(6)*cos(x(2)-x(1))-MZ3* ...
x(7)*L4*x(8)*cos(-x(2)+x(3))-MZ4*x(9)*L4*x(8)*cos(-x(4)+x(3))+MZ4* ...
x(9)*L3*x(7)*cos(-x(4)+x(2))-MZ4*x(9)*L3*x(6)*cos(-x(4)+x(1))+M1*L3* ...
x(6)*L4*x(8)*cos(-x(1)+x(3))-M4*L3^2*x(6)*x(7)*cos(x(2)-x(1))+2*M3*L3* ...
x(6)*L4*x(8)*cos(-x(1)+x(3))-M4*L3*x(7)*L4*x(8)*cos(-x(2)+x(3))+M4*L3* ...
x(6)*L4*x(8)*cos(-x(1)+x(3))+1/2*M4*L3^2*x(6)^2+M3*L4^2*x(8)^2-MZ3* ...
x(6)^2*L3+M3*L3^2*x(6)^2-MZ4*x(8)^2*L4+M4*L4^2*x(8)^2+1/2*M1*L4^2* ...
x(8)^2+1/2*M1*L3^2*x(6)^2+1/2*M4*L3^2*x(7)^2+1/2*XX3*x(6)^2+1/2*XX4* ...
x(9)^2+1/2*XX4*x(8)^2+1/2*XX1*x(10)^2+1/2*XX3*x(7)^2+g*(-L3*cos(x(1))* ...
M1-L4*cos(x(3))*M1+cos(x(5))*MZ1+sin(x(5))*MY1-2*L3*cos(x(1))*M3-2*L4* ...
cos(x(3))*M3+MZ3*cos(x(1))+MZ3*cos(x(2))-2*L4*cos(x(3))*M4+MZ4* ...
cos(x(3))-L3*cos(x(1))*M4+L3*cos(x(2))*M4+MZ4*cos(x(4)));
end
