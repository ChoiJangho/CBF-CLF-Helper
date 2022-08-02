function D = D_gen_rel(obj,q_rel)
    scale = obj.params.scale;
    torso_mass = obj.params.torso_add;

    T = [ 0   0   -1   -1   0  0  0;
          0   0   -1   0   0  -1  0;
          0   0   -1   -1   -1  0  0;
          0   0   -1   0   0  -1  -1;
          1   0   0   0   0  0  0;
          0   1   0   0   0  0  0;
          0   0   -1   0   0  0  0];

    q = T*q_rel + [2*pi*ones(4,1);zeros(3,1)];
    q31=q(1); q32=q(2); q41=q(3); q42=q(4); y=q(5); z=q(6); q1=q(7);

    % gravity
    g=9.81;

    % lengths
    L1=0.625;
    L3=0.4;
    L4=0.4;

    % mass
    % M1=20*scale;
    % M3=6.8*scale;
    % M4=3.2*scale;
    M1=12*scale + torso_mass;
    M3=6.8*scale;
    M4=3.2*scale;

    % com
    MY1=.2;
    MZ1=4.;
    MZ3=1.11;
    MZ4=.41;

    % link inertia
    % XX1=2.22*scale;
    % XX3=.25*scale;
    % XX4=.10*scale;

    % XX1=1.33*scale;
    % XX3=.47*scale;
    % XX4=.2*scale;

    XX1=2.22*scale + 2.22*torso_mass/12;
    XX3=1.08*scale;
    XX4=0.93*scale;

    % D matrix
    D=zeros(7);
    D(1,1)=XX3+M4*L3^2;
    D(1,3)=MZ4*L3*cos(-q41+q31);
    D(1,5)=-cos(q31)*(M4*L3+MZ3);
    D(1,6)=-sin(q31)*(M4*L3+MZ3);
    D(2,2)=XX3+M4*L3^2;
    D(2,4)=MZ4*L3*cos(-q42+q32);
    D(2,5)=-cos(q32)*(M4*L3+MZ3);
    D(2,6)=-sin(q32)*(M4*L3+MZ3);
    D(3,1)=MZ4*L3*cos(-q41+q31);
    D(3,3)=XX4;
    D(3,5)=-MZ4*cos(q41);
    D(3,6)=-MZ4*sin(q41);
    D(4,2)=MZ4*L3*cos(-q42+q32);
    D(4,4)=XX4;
    D(4,5)=-MZ4*cos(q42);
    D(4,6)=-MZ4*sin(q42);
    D(5,1)=-cos(q31)*(M4*L3+MZ3);
    D(5,2)=-cos(q32)*(M4*L3+MZ3);
    D(5,3)=-MZ4*cos(q41);
    D(5,4)=-MZ4*cos(q42);
    D(5,5)=2*M3+2*M4+M1;
    D(5,7)=-cos(q1)*MZ1-sin(q1)*MY1;
    D(6,1)=-sin(q31)*(M4*L3+MZ3);
    D(6,2)=-sin(q32)*(M4*L3+MZ3);
    D(6,3)=-MZ4*sin(q41);
    D(6,4)=-MZ4*sin(q42);
    D(6,6)=2*M3+2*M4+M1;
    D(6,7)=-sin(q1)*MZ1+cos(q1)*MY1;
    D(7,5)=-cos(q1)*MZ1-sin(q1)*MY1;
    D(7,6)=-sin(q1)*MZ1+cos(q1)*MY1;
    D(7,7)=XX1;

    D = T'*D*T;
end