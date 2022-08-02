function C = C_gen_rel(obj, q_rel,dq_rel)
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
    dq = T*dq_rel;
    q31=q(1); q32=q(2); q41=q(3); q42=q(4); y=q(5); z=q(6); q1=q(7);
    dq31=dq(1); dq32=dq(2); dq41=dq(3); dq42=dq(4); dy=dq(5); dz=dq(6); dq1=dq(7);

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

    % C matrix
    C=zeros(7);
    C(1,3)=MZ4*L3*sin(-q41+q31)*dq41;
    C(2,4)=MZ4*L3*sin(-q42+q32)*dq42;
    C(3,1)=-MZ4*L3*sin(-q41+q31)*dq31;
    C(4,2)=-MZ4*L3*sin(-q42+q32)*dq32;
    C(5,1)=sin(q31)*(M4*L3+MZ3)*dq31;
    C(5,2)=sin(q32)*(M4*L3+MZ3)*dq32;
    C(5,3)=MZ4*sin(q41)*dq41;
    C(5,4)=MZ4*sin(q42)*dq42;
    C(5,7)=(sin(q1)*MZ1-cos(q1)*MY1)*dq1;
    C(6,1)=-cos(q31)*(M4*L3+MZ3)*dq31;
    C(6,2)=-cos(q32)*(M4*L3+MZ3)*dq32;
    C(6,3)=-MZ4*cos(q41)*dq41;
    C(6,4)=-MZ4*cos(q42)*dq42;
    C(6,7)=(-cos(q1)*MZ1-sin(q1)*MY1)*dq1;

    C = T'*C*T;
end