function Bs = cbf(obj, x)
%% Defines two CBFs for the stepping stone problem.
% For more details, please refer to Nguyen et al., Dynamic Walking on Stepping Stones
% with Gait Library and Control Barrier Functions.

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

sq1=sin(q(1));sq2=sin(q(2));sq3=sin(q(3));sq4=sin(q(4));sq5=sin(q(5));
cq1=cos(q(1));cq2=cos(q(2));cq3=cos(q(3));cq4=cos(q(4));cq5=cos(q(5));

lf=L3*sin(q(1))+L4*sin(q(3))-L3*sin(q(2))-L4*sin(q(4));
dlf=L3*cos(q(1))*dq(1)+L4*cos(q(3))*dq(3)-L3*cos(q(2))*dq(2)-L4*cos(q(4))*dq(4);

hf=-L3*cos(q(1))-L4*cos(q(3))+L3*cq2+L4*cq4;
dhf=L3*sq1*dq(1)+L4*sq3*dq(3)-L3*sq2*dq(2)-L4*sq4*dq(4);

h1=R1+obj.step_max-sqrt(hf^2+(R1+lf)^2);
dh1=-(hf*dhf+(R1+lf)*dlf)/sqrt(hf^2+(R1+lf)^2);

h2=sqrt((R2+hf)^2+(lf-l_min)^2)-sqrt(R2^2+l_min^2);
dh2=((R2+hf)*dhf+(lf-l_min)*dlf)/sqrt((R2+hf)^2+(lf-l_min)^2);

gb1=obj.gamma_b*h1+dh1;
gb2=obj.gamma_b*h2+dh2;
Bs=[gb1;gb2;];
end