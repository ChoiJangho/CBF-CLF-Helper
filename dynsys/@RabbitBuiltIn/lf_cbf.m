function LfBs = lf_cbf(obj, x)
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

[D, C, G, B] = obj.get_dynamics_matrices_in_reduced_relative_coord(x);
inv_D = inv(D);

sq1=sin(q(1));sq2=sin(q(2));sq3=sin(q(3));sq4=sin(q(4));sq5=sin(q(5));
cq1=cos(q(1));cq2=cos(q(2));cq3=cos(q(3));cq4=cos(q(4));cq5=cos(q(5));

lf=L3*sin(q(1))+L4*sin(q(3))-L3*sin(q(2))-L4*sin(q(4));
dlf=L3*cos(q(1))*dq(1)+L4*cos(q(3))*dq(3)-L3*cos(q(2))*dq(2)-L4*cos(q(4))*dq(4);
N_d2lf=-L3*dq(1)^2*sin(q(1))-L4*dq(3)^2*sin(q(3))+L3*dq(2)^2*sin(q(2))+L4*dq(4)^2*sin(q(4));
M_d2lf=[L3*cos(q(1)) -L3*cos(q(2)) L4*cos(q(3)) -L4*cos(q(4)) 0];
L2f_lf=N_d2lf-M_d2lf*inv_D*(C*dq+G);

hf=-L3*cos(q(1))-L4*cos(q(3))+L3*cq2+L4*cq4;
dhf=L3*sq1*dq(1)+L4*sq3*dq(3)-L3*sq2*dq(2)-L4*sq4*dq(4);
N_d2hf=L3*dq(1)^2*cos(q(1))+L4*dq(3)^2*cos(q(3))-L3*dq(2)^2*cos(q(2))-L4*dq(4)^2*cos(q(4));
M_d2hf=[L3*sin(q(1)) -L3*sin(q(2)) L4*sin(q(3)) -L4*sin(q(4)) 0];
L2f_hf=N_d2hf-M_d2hf*inv_D*(C*dq+G);

dh1=-(hf*dhf+(R1+lf)*dlf)/sqrt(hf^2+(R1+lf)^2);
N_d2h1=(hf^2+(R1+lf)^2)^(-3/2)*(hf*dhf+(R1+lf)*dlf)^2-(hf^2+(R1+lf)^2)^(-1/2)*(dhf^2+dlf^2);
M_d2h1=-(hf^2+(R1+lf)^2)^(-1/2);
L2f_h1=N_d2h1+M_d2h1*(hf*L2f_hf+(R1+lf)*L2f_lf);


dh2=((R2+hf)*dhf+(lf-l_min)*dlf)/sqrt((R2+hf)^2+(lf-l_min)^2);
N_d2h2=-((R2+hf)^2+(lf-l_min)^2)^(-3/2)*((R2+hf)*dhf+(lf-l_min)*dlf)^2+((R2+hf)^2+(lf-l_min)^2)^(-1/2)*(dhf^2+dlf^2);
M_d2h2=((R2+hf)^2+(lf-l_min)^2)^(-1/2);
L2f_h2=N_d2h2+M_d2h2*((R2+hf)*L2f_hf+(lf-l_min)*L2f_lf);

Lf_gb1=obj.gamma_b*dh1+L2f_h1;
Lf_gb2=obj.gamma_b*dh2+L2f_h2;
LfBs=[Lf_gb1; Lf_gb2];
end