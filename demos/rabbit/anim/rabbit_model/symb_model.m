%SYMB_MODEL.M

%From J.W. Grizzle and F. Plestan
%2/15/01

clear

% This is for a robot with an upright trunk, two legs and knees. The model is 
% for five degrees of freedom of the robot, with Foot1 touching the ground
%
syms q1 q31 q32 q41 q42 y z dq1 dq31 dq32 dq41 dq42 dy dz real
syms g L1 L3 L4 M1 M3 M4 real
syms MY1 MZ1 MZ3 MZ4 XX1 XX3 XX4 real 
%
% reference = absolute angles for everything; trigonometric sense for all
%              angles, measured from the vertical,
%
q=[q31 q32 q41 q42 y z q1].';
dq=[dq31 dq32 dq41 dq42 dy dz dq1].';
%
% 
pH=[y;z];
pG1 = pH + L3*[-sin(q31);cos(q31)];
pG2 = pH + L3*[-sin(q32);cos(q32)];
%
%
vH =  jacobian(pH,q)*dq;
vG1 = jacobian(pG1,q)*dq;
vG2 = jacobian(pG2,q)*dq;
%
%
R_T=[cos(q1) -sin(q1); sin(q1) cos(q1)];
vH_RT = R_T.'*vH;
R_Fem1=[cos(q31) -sin(q31); sin(q31) cos(q31)];
vH_Fem1 = R_Fem1.'*vH;
R_Fem2=[cos(q32) -sin(q32); sin(q32) cos(q32)];
vH_Fem2 = R_Fem2.'*vH;
R_Tib1=[cos(q41) -sin(q41); sin(q41) cos(q41)];
vG1_Tib1 = R_Tib1.'*vG1;
R_Tib2=[cos(q42) -sin(q42); sin(q42) cos(q42)];
vG2_Tib2 = R_Tib2.'*vG2;
%
%
KET    = simplify((1/2)*M1*vH.'*vH + vH_RT.'*[-MZ1;MY1]*dq1 ...
		  +(1/2)*XX1*dq1^2);
KEFem1 = simplify((1/2)*M3*vH.'*vH + vH_Fem1.'*[-MZ3;0]*(dq31) ...
		  +(1/2)*XX3*(dq31)^2);
KEFem2 = simplify((1/2)*M3*vH.'*vH + vH_Fem2.'*[-MZ3;0]*(dq32) ...
		  +(1/2)*XX3*(dq32)^2);
KETib1 = simplify((1/2)*M4*vG1.'*vG1 + vG1_Tib1.'*[-MZ4;0]*(dq41) ...
		  +(1/2)*XX4*(dq41)^2);
KETib2 = simplify((1/2)*M4*vG2.'*vG2 + vG2_Tib2.'*[-MZ4;0]*(dq42) ...
		  +(1/2)*XX4*(dq42)^2);
%
%
KE = simplify(KET + KEFem1 + KEFem2 + KETib1 + KETib2);
KE=simple(KE);
%
%
% centers of gravity and positions of the various members 
%
%pT = pH + (1/M1)*(MY1^2+MZ1^2)^(.5)*[-sin(q1-atan(MY1/MZ1));cos(q1-atan(MY1/MZ1))];
pT    = pH + (1/M1)*[-(sin(q1)*MZ1-cos(q1)*MY1);(cos(q1)*MZ1+sin(q1)*MY1)];
pFem1 = pH + (MZ3/M3)*[-sin(q31);cos(q31)];
pFem2 = pH + (MZ3/M3)*[-sin(q32);cos(q32)];
pG1   = pH +[-L3*sin(q31);L3*cos(q31)];
pG2   = pH +[-L3*sin(q32);L3*cos(q32)];
pTib1 = pG1 + [-(MZ4/M4)*sin(q41);(MZ4/M4)*cos(q41)];
pTib2 = pG2 + [-(MZ4/M4)*sin(q42);(MZ4/M4)*cos(q42)];
pFoot1= pG1 + [-L4*sin(q41);L4*cos(q41)];
pFoot2= pG2 + [-L4*sin(q42);L4*cos(q42)];
%
%
%
vFoot1=jacobian(pFoot1,q)*dq;
vT=jacobian(pT,q)*dq;
%
%
PE = g*(M1*pT(2) + M3*pFem1(2) + M3*pFem2(2) + M4*pTib1(2) + M4*pTib2(2));
PE=simple(PE);
%
%
% Model NOTATION: Spong and Vidyasagar, page 142, Eq. (6.3.12)
%                 D(q)ddq + C(q,dq)*dq + G(q) = B*tau + E2*F_external
%
L=KE-PE;
%save mat_files/work_symb_model_abs
%return
G=jacobian(PE,q).';
G=simple(G);
D=simple(jacobian(KE,dq).');
D=simple(jacobian(D,dq));
%C=jacobian(L,dq).';
%C=jacobian(C,q)-(1/2)*jacobian(D*dq,q).';
%C=simple(C);
%temp1=C*dq+G;
%temp2=jacobian(jacobian(L,dq).',q)*dq-jacobian(L,q).';
%check=temp1-temp2;
%check=simple(check);
%check
%save mat_files/work_symb_model_abs
%return
syms C real
n=max(size(q));
for k=1:n
   for j=1:n
      C(k,j)=0*g;
      for i=1:n
         C(k,j)=C(k,j)+(1/2)*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)))*dq(i);
      end
   end
end
C=simple(C);
%
% Compute the matrix for the input torques and then the contact points.
%
Phi_0=[q31-q1;q32-q1;q41-q31;q42-q32;y;z;q1];
B=jacobian(Phi_0,q);
B=B.'*[eye(4,4);zeros(3,4)];
%
%
% F_ext = [F_T^1;F_N^1;F_T^2;F_N^2];
%
Phi_1=[pFoot1;pFoot2];  %HERE why is pFoot1 HERE?
E=jacobian(Phi_1,q);
E=E.';

save mat_files/work_symb_model_abs

if 0
  %
  % For later use, compute D^{-1} in the form of DI/det(D);
  % 
  d=simple(det(D));
  DI=inv(D);
  DI=simplify(d*DI);
  DI=simple(DI);
  save mat_files/work_symb_model_abs_red
end

N=max(size(q));

%% First, output model to a file called 
%%%%    dyn_mod_abs.m
%%
%

%% write file header
%
fcn_name = 'dyn_mod_abs';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function [D,C,G,B,E]=%s(x)\n',fcn_name);
fprintf(fid,['%% %s    Grizzle''s full model of kneed biped walker' ...
	     ' model.\n'],upper(fcn_name));

fprintf(fid,'%%    [DE,CE,GE,BE,EE] = %s',upper(fcn_name));
fprintf(fid,'(X) is the kneed\n');
fprintf(fid,'%%    biped walking model. (x is of dimension %s)\n',...
    num2str(2*N));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

%% Read in constants
%
fprintf(fid,['[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=' ...
	     ' modelParameters;\n\n']);

%% Reassign configuration parameters
%
fprintf(fid,'q31=x(1); q32=x(2); q41=x(3); q42=x(4);');
fprintf(fid,' y=x(5); z=x(6); q1=x(7);\n');
fprintf(fid,'dq31=x(8); dq32=x(9); dq41=x(10); dq42=x(11);');
fprintf(fid,' dy=x(12); dz=x(13); dq1=x(14);\n\n');

%% Model output
%
fprintf(fid,'%% D matrix\n');
fprintf(fid,'D=zeros(%s);\n',num2str(N));
for k=1:N,
   for j=1:N,
      if D(k,j)~=0
         ttt=char(D(k,j));
         fprintf(fid,'D(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
      end
   end
end

fprintf(fid,'\n%% C matrix\n');
fprintf(fid,'C=zeros(%s);\n',num2str(N));
for k=1:N,
   for j=1:N,
      if C(k,j)~=0
         ttt=char(C(k,j));
         fprintf(fid,'C(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
      end
   end
end

fprintf(fid,'\n%% G matrix\n');
fprintf(fid,'G=zeros(%s,1);\n',num2str(N));
for k=1:N,
  if G(k)~=0
    ttt=char(G(k));
    fprintf(fid,'G(%s)=%s;\n',num2str(k),ttt);
  end
end

fprintf(fid,'\n%% B matrix\n');
[N,M]=size(B);
fprintf(fid,'B=zeros(%s,%s);\n',num2str(N),num2str(M));
for k=1:N,
  for j=1:M,
    if B(k,j)~=0
      ttt=char(B(k,j));
      fprintf(fid,'B(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
    end
  end
end

fprintf(fid,'\n%% E matrix\n');
[N,M]=size(E);
fprintf(fid,'E=zeros(%s,%s);\n',num2str(N),num2str(M));
for k=1:N,
  for j=1:M,
    if E(k,j)~=0
      ttt=char(E(k,j));
      fprintf(fid,'E(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
    end
  end
end

fclose(fid);
%
%
%
if 0
  %% write file header
  %
  fcn_name = 'stance_force';
  disp(['[creating ',upper(fcn_name),'.m]']);
  fid = fopen([fcn_name,'.m'],'w');
  fprintf(fid,'function [f_tan,f_norm] = %s(x,dx,u)\n',fcn_name);
  fprintf(fid,['%% %s    Calculated the forces on the stance leg during' ...
	       ' impact.\n'],upper(fcn_name));
  fprintf(fid,['%%    [F_TAN,F_NORM] = %s(X,DX,U) are the forces on' ...
	       ' the stance leg at impact.\n\n'],upper(fcn_name));
  fprintf(fid,'%%Eric Westervelt\n');
  fprintf(fid,'%%%s\n\n',datestr(now));
  
  N=max(size(q));
  N_F=max(size(q_F));
  
  %% Read in constants
  %
  fprintf(fid,['[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=' ...
	       ' modelParameters;\n\n']);
  fprintf(fid,'[a,b,m] = controlParameters;\n\n');
  
  %% Reassign configuration parameters
  %
  fprintf(fid,['q31=x(1); q32=x(2); q41=x(3); q42=x(4); q1=x(5);\' ...
	       ' n']);
  
  fprintf(fid,['dq31=x(6); dq32=x(7); dq41=x(8); dq42=x(9); dq1=' ...
	       ' x(10);\n']);
  
  %% Model output
  %
  fprintf(fid,'%% D_Fs11 matrix\n');
  fprintf(fid,'D_Fs11=zeros(%s,%s);\n',num2str(N),num2str(N));
  for k=1:N,
    for j=1:N,
      if D_Fs(k,j)~=0
	ttt=char(D_Fs(k,j));
	fprintf(fid,'D_Fs11(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
      end
    end
  end
  
  fprintf(fid,'\n%% D_Fs12 matrix\n');
  fprintf(fid,'D_Fs12=zeros(%s,%s);\n',num2str(N),num2str(N_F-N));
  for k=1:N,
    for j=1:N_F-N,
      if D_Fs(k,j+N)~=0
	ttt=char(D_Fs(k,j+N));
	fprintf(fid,'D_Fs12(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
      end
    end
  end
  
  fprintf(fid,'\n%% D_Fs22 matrix\n');
  fprintf(fid,'D_Fs22=zeros(%s,%s);\n',num2str(N_F-N),num2str(N_F-N));
  for k=1:N_F-N,
    for j=1:N_F-N,
      if D_Fs(k+N,j+N)~=0
	ttt=char(D_Fs(k+N,j+N));
	fprintf(fid,'D_Fs22(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
      end
    end
  end
  
  fprintf(fid,'\n%% C_Fs11 matrix\n');
  fprintf(fid,'C_Fs11=zeros(%s,%s);\n',num2str(N),num2str(N));
  for k=1:N,
    for j=1:N,
      if C_Fs(k,j)~=0
	ttt=char(C_Fs(k,j));
	fprintf(fid,'C_Fs11(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
      end
    end
  end
  
  fprintf(fid,'\n%% C_Fs21 matrix\n');
  fprintf(fid,'C_Fs21=zeros(%s,%s);\n',num2str(N_F-N),num2str(N));
  for k=1:N_F-N,
    for j=1:N,
      if C_Fs(k+N,j)~=0
	ttt=char(C_Fs(k+N,j));
	fprintf(fid,'C_Fs21(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
      end
    end
  end
  
  fprintf(fid,'\n%% G_Fs1 matrix\n');
  fprintf(fid,'G_Fs1=zeros(%s,%s);\n',num2str(N),num2str(1));
  for k=1:N,
    if G_Fs(k)~=0
      ttt=char(G_Fs(k));
      fprintf(fid,'G_Fs1(%s,%s)=%s;\n',num2str(k),num2str(1),ttt);
    end
  end
  
  fprintf(fid,'\n%% G_Fs2 matrix\n');
  fprintf(fid,'G_Fs2=zeros(%s,%s);\n',num2str(N_F-N),num2str(1));
  for k=1:N_F-N,
    if G_Fs(k+N)~=0
      ttt=char(G_Fs(k+N));
      fprintf(fid,'G_Fs2(%s,%s)=%s;\n',num2str(k),num2str(1),ttt);
    end
  end
  
  fprintf(fid,'\n%% B matrix\n');
  [N,M]=size(B);
  fprintf(fid,'B=zeros(%s,%s);\n',num2str(N),num2str(M));
  for k=1:N,
    for j=1:M,
      if B(k,j)~=0
	ttt=char(B(k,j));
	fprintf(fid,'B(%s,%s)=%s;\n',num2str(k),num2str(j),ttt);
      end
    end
  end
  
  fprintf(fid,'\n%% See my notes, 9/13/200 for equations...\n');
  
  fprintf(fid,'DD=inv((D_Fs12*inv(D_Fs22)).''*D_Fs12...\n');
  fprintf(fid,'   *inv(D_Fs22))*(D_Fs12*inv(D_Fs22)).'';\n');
  
  fprintf(fid,'F=DD*(-(D_Fs11-D_Fs12*inv(D_Fs22)*D_Fs12.'')...\n');
  fprintf(fid,'  *dx(6:10)+(D_Fs12*inv(D_Fs22)*C_Fs21-C_Fs11)...\n');
  fprintf(fid,'  *dx(1:5)+D_Fs12*inv(D_Fs22)*G_Fs2-G_Fs1+B*u);\n\n');
  
  fprintf(fid,'f_tan=F(1);\n');
  fprintf(fid,'f_norm=F(2);\n');
  fclose(fid);
end