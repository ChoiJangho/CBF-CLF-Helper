function hs = get_hs(obj, x)
%% Get the two distance functions (to the circular boundary of the trajectory)
% that are used to define the CBFs.

qrel = x(1:7);
dqrel = x(8:end);

L3=0.4;
L4=0.4;

%% Coordinate transformation
T = [-1   -1   0  0  0;
     -1    0   0 -1  0;
     -1   -1  -1  0  0;
	 -1    0   0 -1 -1;
     -1    0   0  0  0]; 
q = T*qrel(3:end) + 2*pi*[ones(4,1);0];
dq = T*dqrel(3:end);

%% compute parameter for barrier function
l_min = obj.step_min/2;
R1 = 0.5;
R2 = 2;

lf=L3*sin(q(1))+L4*sin(q(3))-L3*sin(q(2))-L4*sin(q(4));

hf=-L3*cos(q(1))-L4*cos(q(3))+L3*cos(q(2))+L4*cq4;

h1=R1+obj.step_max-sqrt(hf^2+(R1+lf)^2);

h2=sqrt((R2+hf)^2+(lf-l_min)^2)-sqrt(R2^2+l_min^2);

hs = [h1;h2];
end