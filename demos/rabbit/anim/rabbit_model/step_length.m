function h = step_length(x)
% STEP_LENGTH    The step length.
%    H = STEP_LENGTH(X)

%Eric Westervelt
%24-Mar-2001 10:46:13

[g,L1,L3,L4] = modelParameters;

q31=x(1); q32=x(2); q41=x(3); q42=x(4); q1=x(5);
h = L3*sin(q31)+L4*sin(q41)-L3*sin(q32)-L4*sin(q42);