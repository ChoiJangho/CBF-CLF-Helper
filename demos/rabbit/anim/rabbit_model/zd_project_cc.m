function z = zd_project_cc(x)
%ZD_PROJECT_CC    Projects the zero dynamics to the zero dynamics.
%
%    Z = ZD_PROJECT_CC(X)

%Eric Westervelt
%26-Feb-2001 20:35:19

% first project
z = zd_project(x);
dz = zd(z);
x_zd = zd_lift(z(1),dz(1));

% theta
z(1) = 1/2*(x_zd(1)+x_zd(3));

% dtheta
z(2) = 1/2*(x_zd(6)+x_zd(8));