function z =zd_project(x)
%ZD_PROJECT    Projects the zero dynamics to the zero dynamics.
%
%    Z = ZD_PROJECT(X)

%Eric Westervelt
%01-May-2001 07:44:48

q31=x(1); q32=x(2); q41=x(3); q42=x(4); q1=x(5);
dq31=x(6); dq32=x(7); dq41=x(8); dq42=x(9); dq1=x(10);

% theta (in absolute coordinates)
z(1) = (q31+q41)/2;

% gamma (in absolute coordinates)
z(2) = (21/4-239/250*cos(-q32+q31)+1361/250*cos(q31-q41)-41/250*cos(-q42+q31)+2/25*sin(-q1+q31)-8/5*cos(-q1+q31))*dq31+(-239/250*cos(-q32+q31)+381/500-239/250*cos(-q32+q41)+41/250*cos(q42-q32))*dq32+(1361/250*cos(q31-q41)-239/250*cos(-q32+q41)+1543/250-41/250*cos(q42-q41)-8/5*cos(q1-q41)-2/25*sin(q1-q41))*dq41+(-41/250*cos(-q42+q31)+41/250*cos(q42-q32)-41/250*cos(q42-q41)+1/10)*dq42+(2/25*sin(-q1+q31)-8/5*cos(-q1+q31)-8/5*cos(q1-q41)-2/25*sin(q1-q41)+111/50)*dq1;