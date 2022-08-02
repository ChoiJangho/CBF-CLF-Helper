function [D,C,G,B,E]=dyn_mod_abs(x)
% DYN_MOD_ABS    Grizzle's full model of kneed biped walker model.
%    [DE,CE,GE,BE,EE] = DYN_MOD_ABS(X) is the kneed
%    biped walking model. (x is of dimension 14)
%Eric Westervelt
%16-Feb-2001 16:13:37

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]= modelParameters;

q31=x(1); q32=x(2); q41=x(3); q42=x(4); y=x(5); z=x(6); q1=x(7);
dq31=x(8); dq32=x(9); dq41=x(10); dq42=x(11); dy=x(12); dz=x(13); dq1=x(14);

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

% G matrix
G=zeros(7,1);
G(1)=-g*sin(q31)*(MZ3+L3*M4);
G(2)=-g*sin(q32)*(MZ3+L3*M4);
G(3)=-g*MZ4*sin(q41);
G(4)=-g*MZ4*sin(q42);
G(6)=g*(M1+2*M3+2*M4);
G(7)=-g*(sin(q1)*MZ1-cos(q1)*MY1);

% B matrix
B=zeros(7,4);
B(1,1)=1;
B(1,3)=-1;
B(2,2)=1;
B(2,4)=-1;
B(3,3)=1;
B(4,4)=1;
B(7,1)=-1;
B(7,2)=-1;

% E matrix
E=zeros(7,4);
E(1,1)=-L3*cos(q31);
E(1,2)=-L3*sin(q31);
E(2,3)=-L3*cos(q32);
E(2,4)=-L3*sin(q32);
E(3,1)=-L4*cos(q41);
E(3,2)=-L4*sin(q41);
E(4,3)=-L4*cos(q42);
E(4,4)=-L4*sin(q42);
E(5,1)=1;
E(5,3)=1;
E(6,2)=1;
E(6,4)=1;
