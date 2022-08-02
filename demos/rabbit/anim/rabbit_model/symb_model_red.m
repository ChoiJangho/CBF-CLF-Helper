function symb_model_red
%SYMB_MODEL_RED.M

%From J.W. Grizzle and F. Plestan
%2/15/01

clear

% This is for a robot with an upright trunk, two legs and knees. The
% model is for five degrees of freedom of the robot, with Foot1
% touching the ground
%
syms q1 q31 q32 q41 q42 dq1 dq31 dq32 dq41 dq42 real
syms g L1 L3 L4 M1 M3 M4 real
syms MY1 MZ1 MZ3 MZ4 XX1 XX3 XX4 real 
%
% reference = absolute angles for everything; trigonometric sense for
%             all angles, measured from the vertical,
%
q=[q31 q32 q41 q42 q1].';
dq=[dq31 dq32 dq41 dq42 dq1].';
%
% 
pH = -(L3*[-sin(q31); cos(q31)] + L4*[-sin(q41); cos(q41)]);
pG1 = pH + L3*[-sin(q31); cos(q31)];
pG2 = pH + L3*[-sin(q32); cos(q32)];
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
KET    = simplify((1/2)*M1*vH.'*vH + vH_RT.'*[-MZ1;MY1]*dq1 + ...
    (1/2)*XX1*dq1^2);
KEFem1 = simplify((1/2)*M3*vH.'*vH + vH_Fem1.'*[-MZ3;0]*(dq31) + ...
    (1/2)*XX3*(dq31)^2);
KEFem2 = simplify((1/2)*M3*vH.'*vH + vH_Fem2.'*[-MZ3;0]*(dq32) + ...
    (1/2)*XX3*(dq32)^2);
KETib1 = simplify((1/2)*M4*vG1.'*vG1 + ...
    vG1_Tib1.'*[-MZ4;0]*(dq41) + (1/2)*XX4*(dq41)^2);
KETib2 = simplify((1/2)*M4*vG2.'*vG2 + ...
    vG2_Tib2.'*[-MZ4;0]*(dq42) +(1/2)*XX4*(dq42)^2);
%
%
KE = simplify(KET + KEFem1 + KEFem2 + KETib1 + KETib2);
KE=simple(KE);
%
% centers of gravity and positions of the various members 
%
pT    = pH + (1/M1)*[-(sin(q1)*MZ1-cos(q1)*MY1);
		    (cos(q1)*MZ1+sin(q1)*MY1)];
pHead = pH + L1*[-sin(q1); cos(q1)];
pFem1 = pH + (MZ3/M3)*[-sin(q31);cos(q31)];
pFem2 = pH + (MZ3/M3)*[-sin(q32);cos(q32)];
pTib1 = pG1 + (MZ4/M4)*[-sin(q41); cos(q41)];
pTib2 = pG2 + (MZ4/M4)*[-sin(q42); cos(q42)];
pFoot1= pG1 + L4*[-sin(q41); cos(q41)];
pFoot2= pG2 + L4*[-sin(q42); cos(q42)];
%
%
vFoot1=jacobian(pFoot1,q)*dq;
vFoot2=jacobian(pFoot2,q)*dq;
vT=jacobian(pT,q)*dq;
%
%
PE = g*(M1*pT(2) + M3*pFem1(2) + M3*pFem2(2) + M4*pTib1(2) + ...
    M4*pTib2(2));
PE = simple(PE);
%
%
% Center of mass calculation...may want to use this for the theta1
% parameter to watch the center of mass trajectory (see notes
% 3/5/01 or Principles of Dynamics by Greenwood, page 136)
%
%
M_total = M1 + 2*M3 + 2*M4;
cm = 1/M_total*(M1*pT + M3*pFem1 + M3*pFem2 + M4*pTib1 + M4*pTib2);
cm = simple(cm);
%
%
% Model NOTATION: Spong and Vidyasagar, page 142, Eq. (6.3.12)
%                 D(q)ddq + C(q,dq)*dq + G(q) = B*tau + E2*F_external
%

%L=KE-PE;

G=jacobian(PE,q).';
G=simple(G);
D=simple(jacobian(KE,dq).');
D=simple(jacobian(D,dq));

syms C real
n=max(size(q));
for k=1:n
  for j=1:n
    C(k,j)=0*g;
    for i=1:n
      C(k,j)=C(k,j)+1/2*(diff(D(k,j),q(i)) + ...
			 diff(D(k,i),q(j)) - ...
			 diff(D(i,j),q(k)))*dq(i);
    end
  end
end
C=simple(C);
%
% Compute the matrix for the input torques and then the contact
% points.
%
Phi_0=[q31-q1;q32-q1;q41-q31;q42-q32;q1];
B=jacobian(Phi_0,q);
B=B.'*[eye(4,4);zeros(1,4)];
%
%
% F_ext = [F_T^1;F_N^1;F_T^2;F_N^2];
%
Phi_1=[pFoot2];
E=jacobian(Phi_1,q);
E=E.';

save mat_files/work_symb_model_abs_red


N=max(size(q));

%% First, output model to a file called 
%%%%    dynamics+<model type>.m
%%
%

%% write file header
%
fcn_name = 'dyn_mod_abs_red';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function [D,C,G,B]=%s(x)\n',fcn_name);
fprintf(fid,['%% %s    Grizzle''s model of kneed biped walker' ...
	     ' model.\n'],upper(fcn_name));

fprintf(fid,'%%    [D,C,G,B] = %s',upper(fcn_name));
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
fprintf(fid,'q31=x(1); q32=x(2); q41=x(3); q42=x(4); q1=x(5);\n');
fprintf(fid,['dq31=x(6); dq32=x(7); dq41=x(8); dq42=x(9); dq1=' ...
	     ' x(10);\n\n']);

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
fclose(fid);


%% write file header
%
fcn_name = 'swing_foot_height';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function h = %s(x)\n',fcn_name);
fprintf(fid,'%% %s    The swing foot height.\n',upper(fcn_name));
fprintf(fid,'%%    H = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'[g,L1,L3,L4] = modelParameters;\n\n');

fprintf(fid,'[n,m] = size(x);\n\n');
fprintf(fid,'if n ~= 1 & m ~= 1\n');
ttt = char(pFoot2(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  h = %s;\n',ttt);
fprintf(fid,'else\n');
ttt = char(pFoot2(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  h = %s;\n',ttt);
fprintf(fid,'end\n');
fclose(fid);


%% write file header
%
fcn_name = 'head_height';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function h = %s(x)\n',fcn_name);
fprintf(fid,'%% %s    Head height.\n',upper(fcn_name));
fprintf(fid,'%%    H = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'[g,L1,L3,L4] = modelParameters;\n\n');

fprintf(fid,'[n,m] = size(x);\n\n');
fprintf(fid,'if n ~= 1 & m ~= 1\n');
ttt = char(pHead(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  h = %s;\n',ttt);
fprintf(fid,'else\n');
ttt = char(pHead(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  h = %s;\n',ttt);
fprintf(fid,'end\n');
fclose(fid);


%% write file header
%
fcn_name = 'limb_position';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function out = %s(x,pH_horiz)\n',fcn_name);
fprintf(fid,'%% %s   Head height.\n',upper(fcn_name));
fprintf(fid,'%%   out = %s(X,PH_HORIZ)\n',upper(fcn_name));
fprintf(fid,['%%   The members are pH, pT, pHead, pFem1, pFem2, pG1,' ...
	     ' pG2, pTib1,\n%%   pTib2, pFoot1, and pFoot2.\n\n']);

fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,['[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4]=' ...
	     ' modelParameters;\n\n']);

pH_tmp = [0; pH(2)];

fprintf(fid,'[n,m] = size(x);\n');
fprintf(fid,'if n == 10, k = m; else, k = n; end\n\n');
fprintf(fid,'if n ~= 1 & m ~= 1\n');
fprintf(fid,'  out.pH1 = pH_horiz*ones(k,1);\n');
ttt = char(pH(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pH2 = %s;\n',ttt);
pT_tmp = pT - pH;
ttt = char(pT_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pT1 = out.pH1 + %s;\n',ttt);
ttt = char(pT_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pT2 = out.pH2 + %s;\n',ttt);
pHead_tmp = pHead - pH;
ttt = char(pHead_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pHead1 = out.pH1 + %s;\n',ttt);
ttt = char(pHead_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pHead2 = out.pH2 + %s;\n',ttt);
pFem1_tmp = pFem1 - pH;
ttt = char(pFem1_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pFem11 = out.pH1 + %s;\n',ttt);
ttt = char(pFem1_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pFem12 = out.pH2 + %s;\n',ttt);
pFem2_tmp = pFem2 - pH;
ttt = char(pFem2_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pFem21 = out.pH1 + %s;\n',ttt);
ttt = char(pFem2_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pFem22 = out.pH2 + %s;\n',ttt);
pG1_tmp = pG1 - pH;
ttt = char(pG1_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pG11 = out.pH1 + %s;\n',ttt);
ttt = char(pG1_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pG12 = out.pH2 + %s;\n',ttt);
pG2_tmp = pG2 - pH;
ttt = char(pG2_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pG21 = out.pH1 + %s;\n',ttt);
ttt = char(pG2_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pG22 = out.pH2 + %s;\n',ttt);
pTib1_tmp = pTib1 - pH;
ttt = char(pTib1_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pTib11 = out.pH1 + %s;\n',ttt);
ttt = char(pTib1_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pTib12 = out.pH2 + %s;\n',ttt);
pTib2_tmp = pTib2 - pH;
ttt = char(pTib2_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pTib21 = out.pH1 + %s;\n',ttt);
ttt = char(pTib2_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pTib22 = out.pH2 + %s;\n',ttt);
pFoot1_tmp = pFoot1 - pH;
ttt = char(pFoot1_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pFoot11 = out.pH1 + %s;\n',ttt);
ttt = char(pFoot1_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pFoot12 = out.pH2 + %s;\n',ttt);
pFoot2_tmp = pFoot2 - pH;
ttt = char(pFoot2_tmp(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pFoot21 = out.pH1 + %s;\n',ttt);
ttt = char(pFoot2_tmp(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  out.pFoot22 = out.pH2 + %s;\n',ttt);
fprintf(fid,'else\n');
fprintf(fid,'  out.pH1 = pH_horiz;\n');
ttt = char(pH(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pH2 = %s;\n',ttt);
pT_tmp = pT - pH;
ttt = char(pT_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pT1 = out.pH1 + %s;\n',ttt);
ttt = char(pT_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pT2 = out.pH2 + %s;\n',ttt);
pHead_tmp = pHead - pH;
ttt = char(pHead_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pHead1 = out.pH1 + %s;\n',ttt);
ttt = char(pHead_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pHead2 = out.pH2 + %s;\n',ttt);
pFem1_tmp = pFem1 - pH;
ttt = char(pFem1_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pFem11 = out.pH1 + %s;\n',ttt);
ttt = char(pFem1_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pFem12 = out.pH2 + %s;\n',ttt);
pFem2_tmp = pFem2 - pH;
ttt = char(pFem2_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pFem21 = out.pH1 + %s;\n',ttt);
ttt = char(pFem2_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pFem22 = out.pH2 + %s;\n',ttt);
pG1_tmp = pG1 - pH;
ttt = char(pG1_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pG11 = out.pH1 + %s;\n',ttt);
ttt = char(pG1_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pG12 = out.pH2 + %s;\n',ttt);
pG2_tmp = pG2 - pH;
ttt = char(pG2_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pG21 = out.pH1 + %s;\n',ttt);
ttt = char(pG2_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pG22 = out.pH2 + %s;\n',ttt);
pTib1_tmp = pTib1 - pH;
ttt = char(pTib1_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pTib11 = out.pH1 + %s;\n',ttt);
ttt = char(pTib1_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pTib12 = out.pH2 + %s;\n',ttt);
pTib2_tmp = pTib2 - pH;
ttt = char(pTib2_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pTib21 = out.pH1 + %s;\n',ttt);
ttt = char(pTib2_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pTib22 = out.pH2 + %s;\n',ttt);
pFoot1_tmp = pFoot1 - pH;
ttt = char(pFoot1_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pFoot11 = out.pH1 + %s;\n',ttt);
ttt = char(pFoot1_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pFoot12 = out.pH2 + %s;\n',ttt);
pFoot2_tmp = pFoot2 - pH;
ttt = char(pFoot2_tmp(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pFoot21 = out.pH1 + %s;\n',ttt);
ttt = char(pFoot2_tmp(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  out.pFoot22 = out.pH2 + %s;\n',ttt);
fprintf(fid,'end\n');
fclose(fid);


%% write file header
%
fcn_name = 'swing_foot_velocity';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function [vh,vv] = %s(x)\n',fcn_name);
fprintf(fid,'%% %s    The swing foot vertical velocity.\n', ...
	upper(fcn_name));
fprintf(fid,'%%    [VH,VV] = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'[g,L1,L3,L4] = modelParameters;\n\n');

fprintf(fid,'[n,m] = size(x);\n\n');
fprintf(fid,'if n ~= 1 & m ~= 1\n');
ttt = char(vFoot2(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  vh = %s;\n',ttt);
ttt = char(vFoot2(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  vv = %s;\n',ttt);
fprintf(fid,'else\n');
ttt = char(vFoot2(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  vh = %s;\n',ttt);
ttt = char(vFoot2(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  vv = %s;\n',ttt);
fprintf(fid,'end');
fclose(fid);


%% write file header
%
fcn_name = 'step_length';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function h = %s(x)\n',fcn_name);
fprintf(fid,'%% %s    The step length.\n',upper(fcn_name));
fprintf(fid,'%%    H = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));
fprintf(fid,'[g,L1,L3,L4] = modelParameters;\n\n');

%% Reassign configuration parameters
%
fprintf(fid,'q31=x(1); q32=x(2); q41=x(3); q42=x(4); q1=x(5);\n');

fprintf(fid,'h = %s;',fixlength(char(pFoot2(1)-pFoot1(1)),'+',70));
fclose(fid);


%% write file header
%
fcn_name = 'hip_pos';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function [pHh,pHv] = %s(x)\n',fcn_name);
fprintf(fid,'%% %s    The hip position.\n',upper(fcn_name));
fprintf(fid,'%%    [pHv,pHh] = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));
fprintf(fid,'[g,L1,L3,L4] = modelParameters;\n\n');

fprintf(fid,'[n,m] = size(x);\n\n');
fprintf(fid,'if n ~= 1 & m ~= 1\n');
ttt = char(pH(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  pHh = %s;\n',ttt);
ttt = char(pH(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  pHv = %s;\n',ttt);
fprintf(fid,'else\n');
ttt = char(pH(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  pHh = %s;\n',ttt);
ttt = char(pH(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  pHv = %s;\n',ttt);
fprintf(fid,'end\n');
fclose(fid);


%% write file header
%
fcn_name = 'hip_vel';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function [vHh,vHv] = %s(x)\n',fcn_name);
fprintf(fid,'%% %s    The hip velocity.\n',upper(fcn_name));
fprintf(fid,'%%    [vHh,vHv] = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));
fprintf(fid,'[g,L1,L3,L4] = modelParameters;\n\n');
ttt = char(vH(1));
ttt = rep_str(ttt,1);
fprintf(fid,'vHh = %s;\n\n',fixlength(ttt,'+',70));
ttt = char(vH(2));
ttt = rep_str(ttt,1);
fprintf(fid,'vHv = %s;\n',fixlength(ttt,'+',70));
fclose(fid);


%% write file header
%
fcn_name = 'total_energy_red';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function E = %s(x)\n',fcn_name);
fprintf(fid,'%% %s    Total energy of robot, E = KE + PE.\n', ...
	upper(fcn_name));
fprintf(fid,'%%    E = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));
fprintf(fid,['[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=' ...
	     ' modelParameters;\n\n']);

fprintf(fid,'[n,m] = size(x);\n\n');
fprintf(fid,'if n ~= 1 & m ~= 1\n');
ttt = char(KE+PE);
ttt = rep_str(ttt,1);
fprintf(fid,'  E = %s;\n',fixlength(ttt,'*+-',70));
fprintf(fid,'else\n');
ttt = char(KE+PE);
ttt = rep_str(ttt,2);
fprintf(fid,'  E = %s;\n',fixlength(ttt,'*+-',70));
fprintf(fid,'end\n');
fclose(fid);


%% write file header
%
fcn_name = 'center_of_mass';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function [cmh,cmv] = %s(x)\n',fcn_name);
fprintf(fid,'%% %s    Robot center of mass.\n', ...
	upper(fcn_name));
fprintf(fid,'%%    [CMH,CMV] = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));
fprintf(fid,['[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=' ...
	     ' modelParameters;\n\n']);

fprintf(fid,'[n,m] = size(x);\n\n');
fprintf(fid,'if n ~= 1 & m ~= 1\n');
ttt = char(cm(1));
ttt = rep_str(ttt,1);
fprintf(fid,'  cmh = %s;\n',fixlength(ttt,'*+-',70));
ttt = char(cm(2));
ttt = rep_str(ttt,1);
fprintf(fid,'  cmv = %s;\n',fixlength(ttt,'*+-',70));
fprintf(fid,'else\n');
ttt = char(cm(1));
ttt = rep_str(ttt,2);
fprintf(fid,'  cmh = %s;\n',fixlength(ttt,'*+-',70));
ttt = char(cm(2));
ttt = rep_str(ttt,2);
fprintf(fid,'  cmv = %s;\n',fixlength(ttt,'*+-',70));
fprintf(fid,'end\n');
fclose(fid);


%------------------------------------------------------

function ttt_out = rep_str(ttt,type)

if type ==1
  ttt = strrep(ttt,'dq31','x(:,6)');
  ttt = strrep(ttt,'dq32','x(:,7)');
  ttt = strrep(ttt,'dq41','x(:,8)');
  ttt = strrep(ttt,'dq42','x(:,9)');
  ttt = strrep(ttt,'dq1','x(:,10)');
  ttt = strrep(ttt,'q31','x(:,1)');
  ttt = strrep(ttt,'q32','x(:,2)');
  ttt = strrep(ttt,'q41','x(:,3)');
  ttt = strrep(ttt,'q42','x(:,4)');
  ttt = strrep(ttt,'q1','x(:,5)');
  ttt = strrep(ttt,'*','.*');
  ttt = strrep(ttt,'^','.^');
else
  ttt = strrep(ttt,'dq31','x(6)');
  ttt = strrep(ttt,'dq32','x(7)');
  ttt = strrep(ttt,'dq41','x(8)');
  ttt = strrep(ttt,'dq42','x(9)');
  ttt = strrep(ttt,'dq1','x(10)');
  ttt = strrep(ttt,'q31','x(1)');
  ttt = strrep(ttt,'q32','x(2)');
  ttt = strrep(ttt,'q41','x(3)');
  ttt = strrep(ttt,'q42','x(4)');
  ttt = strrep(ttt,'q1','x(5)');
end

ttt_out = ttt;