function G = G_gen_rel(obj, q_rel)
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

    % G matrix
    G=zeros(7,1);
    G(1)=-g*sin(q31)*(MZ3+L3*M4);
    G(2)=-g*sin(q32)*(MZ3+L3*M4);
    G(3)=-g*MZ4*sin(q41);
    G(4)=-g*MZ4*sin(q42);
    G(6)=g*(M1+2*M3+2*M4);
    G(7)=-g*(sin(q1)*MZ1-cos(q1)*MY1);

    G = T'*G;
end