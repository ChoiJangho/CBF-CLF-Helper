function LgBs = lg_cbf(obj, x)
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

lf=L3*sin(q(1))+L4*sin(q(3))-L3*sin(q(2))-L4*sin(q(4));
M_d2lf=[L3*cos(q(1)) -L3*cos(q(2)) L4*cos(q(3)) -L4*cos(q(4)) 0];
LgLf_lf=M_d2lf*inv_D*B;

hf=-L3*cos(q(1))-L4*cos(q(3))+L3*cos(q(2))+L4*cos(q(4));
M_d2hf=[L3*sin(q(1)) -L3*sin(q(2)) L4*sin(q(3)) -L4*sin(q(4)) 0];
LgLf_hf=M_d2hf*inv_D*B;

M_d2h1=-(hf^2+(R1+lf)^2)^(-1/2);
LgLf_h1=M_d2h1*(hf*LgLf_hf+(R1+lf)*LgLf_lf);

M_d2h2=((R2+hf)^2+(lf-l_min)^2)^(-1/2);
LgLf_h2=M_d2h2*((R2+hf)*LgLf_hf+(lf-l_min)*LgLf_lf);

LgBs=[LgLf_h1; LgLf_h2];
end