function [D, C, G, B] = get_dynamics_matrices_in_reduced_relative_coord(obj, x)
    scale = obj.params.scale;

    %% Parameters
    % gravity
    g=9.81;
    % lengths
    L1=0.625;
    L3=0.4;
    L4=0.4;
    % mass
    M1=12*scale;
    M3=6.8*scale;
    M4=3.2*scale;
    % com
    MY1=.2;
    MZ1=4.;
    MZ3=1.11;
    MZ4=.41;
    % link inertia
    XX1=2.22*scale;
    XX3=1.08*scale;
    XX4=.93*scale;

    q31=x(1); q32=x(2); q41=x(3); q42=x(4); q1=x(5);
    dq31=x(6); dq32=x(7); dq41=x(8); dq42=x(9); dq1= x(10);

    % D matrix
    D=zeros(5);
    D(1,1)=M4*L3^2+M1*L3^2-2*MZ3*L3+2*M3*L3^2+XX3;
    D(1,2)=-L3*cos(q32-q31)*(M4*L3+MZ3);
    D(1,3)=L4*cos(-q31+q41)*(-MZ3+2*M3*L3+M1*L3+M4*L3);
    D(1,4)=-MZ4*L3*cos(-q42+q31);
    D(1,5)=-L3*(MY1*sin(q1-q31)+MZ1*cos(q1-q31));
    D(2,1)=-L3*cos(q32-q31)*(M4*L3+MZ3);
    D(2,2)=M4*L3^2+XX3;
    D(2,3)=-L4*cos(-q32+q41)*(M4*L3+MZ3);
    D(2,4)=MZ4*L3*cos(-q42+q32);
    D(3,1)=L4*cos(-q31+q41)*(-MZ3+2*M3*L3+M1*L3+M4*L3);
    D(3,2)=-L4*cos(-q32+q41)*(M4*L3+MZ3);
    D(3,3)=-2*MZ4*L4+2*M4*L4^2+M1*L4^2+2*M3*L4^2+XX4;
    D(3,4)=-MZ4*L4*cos(-q42+q41);
    D(3,5)=-L4*(MZ1*cos(q1-q41)+MY1*sin(q1-q41));
    D(4,1)=-MZ4*L3*cos(-q42+q31);
    D(4,2)=MZ4*L3*cos(-q42+q32);
    D(4,3)=-MZ4*L4*cos(-q42+q41);
    D(4,4)=XX4;
    D(5,1)=-L3*(MY1*sin(q1-q31)+MZ1*cos(q1-q31));
    D(5,3)=-L4*(MZ1*cos(q1-q41)+MY1*sin(q1-q41));
    D(5,5)=XX1;

    % C matrix
    C=zeros(5);
    C(1,2)=L3*sin(q32-q31)*(M4*L3+MZ3)*dq32;
    C(1,3)=-L4*sin(-q31+q41)*(-MZ3+2*M3*L3+M1*L3+M4*L3)*dq41;
    C(1,4)=-MZ4*L3*sin(-q42+q31)*dq42;
    C(1,5)=-L3*(MY1*cos(q1-q31)-MZ1*sin(q1-q31))*dq1;
    C(2,1)=-L3*sin(q32-q31)*(M4*L3+MZ3)*dq31;
    C(2,3)=L4*sin(-q32+q41)*(M4*L3+MZ3)*dq41;
    C(2,4)=MZ4*L3*sin(-q42+q32)*dq42;
    C(3,1)=L4*sin(-q31+q41)*(-MZ3+2*M3*L3+M1*L3+M4*L3)*dq31;
    C(3,2)=-L4*sin(-q32+q41)*(M4*L3+MZ3)*dq32;
    C(3,4)=-MZ4*L4*sin(-q42+q41)*dq42;
    C(3,5)=-L4*(-MZ1*sin(q1-q41)+MY1*cos(q1-q41))*dq1;
    C(4,1)=MZ4*L3*sin(-q42+q31)*dq31;
    C(4,2)=-MZ4*L3*sin(-q42+q32)*dq32;
    C(4,3)=MZ4*L4*sin(-q42+q41)*dq41;
    C(5,1)=L3*(MY1*cos(q1-q31)-MZ1*sin(q1-q31))*dq31;
    C(5,3)=L4*(-MZ1*sin(q1-q41)+MY1*cos(q1-q41))*dq41;

    % G matrix
    G=zeros(5,1);
    G(1)=g*sin(q31)*(L3*M1+2*L3*M3-MZ3+L3*M4);
    G(2)=-g*sin(q32)*(MZ3+L3*M4);
    G(3)=g*sin(q41)*(L4*M1+2*L4*M3+2*L4*M4-MZ4);
    G(4)=-g*MZ4*sin(q42);
    G(5)=-g*(sin(q1)*MZ1-cos(q1)*MY1);

    % B matrix
    B=zeros(5,4);
    B(1,1)=1;
    B(1,3)=-1;
    B(2,2)=1;
    B(2,4)=-1;
    B(3,3)=1;
    B(4,4)=1;
    B(5,1)=-1;
    B(5,2)=-1;
end