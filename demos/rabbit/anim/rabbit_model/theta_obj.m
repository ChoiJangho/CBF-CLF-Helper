function f = theta_obj(theta1)
%THETA_OBJ     Function to find theta.
%
%    F = theta_obj(THETA1)

%Eric Westervelt
%01-May-2001 05:42:40

a = controlParameters;

aN=a(7); bN=a(14); cN=a(21); dN=a(28);

[g,L1,L3,L4] = modelParameters;

f = -L3.*cos(-1./2.*cN+theta1)-L4.*cos(1./2.*cN+theta1)+L3.*cos(bN- ...
    1./2.*cN+theta1)+L4.*cos(bN-1./2.*cN+dN+theta1);