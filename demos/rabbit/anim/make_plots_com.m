function make_plots_com(t,x)
n=10;
%% plot s
% [g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = modelParameters;
q1=x(:,1);q2=x(:,2);q3=x(:,3);q4=x(:,4);q5=x(:,5);
dq1=x(:,6);dq2=x(:,7);dq3=x(:,8);dq4=x(:,9);dq5=x(:,10);

[a,b,m] = controlParameters;
q31=q1;q41=q3;
s=(1/2*q31+1/2*q41-b)/m; % this s was tested !
n=n+1;figure(n); plot(t,s); title('s');

%% plot COM
[cmh,cmv] = center_of_mass(x);
n=n+1;figure(n); plot(cmh,cmv); title('COM traj'); axis('equal');

% plot COM angle arctan(cmv/cmh)
n=n+1;figure(n); com_angle=atan(cmh./cmv); plot(t,com_angle);title('COM angle');