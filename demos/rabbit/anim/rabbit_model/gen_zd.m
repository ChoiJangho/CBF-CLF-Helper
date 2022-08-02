function gen_zd
%GEN_ZD    Derive the zero dynamics for the kneed biped walker with
%    Franck's offset masses model using Bezier functions.

%Eric Westervelt
%1/8/01

if isempty(dir('mat_files/work_symb_zero_dynamics.mat'))
  load mat_files/work_symb_control_abs_red_test
  model_data = load('mat_files/work_symb_model_abs_red');
  load mat_files/work_symb_model_abs_red
  
  % First step is to do a partial feedback linearization of the
  % dynamics. This will simplify some of the computations and,
  % hopefully, improve the numerical conditioning. See Reyhanoglu,
  % McClamroch and van der Schaft, IEEE T-AC, Sept. 1999, pp. 1663.
  
  % First, change coordinates into all relative plus one absolute
  syms q1_rel q2_rel q3_rel q4_rel q5_rel
  syms dq1_rel dq2_rel dq3_rel dq4_rel dq5_rel
  q_rel = [q1_rel; q2_rel; q3_rel; q4_rel; q5_rel];
  dq_rel = [dq1_rel; dq2_rel; dq3_rel; dq4_rel; dq5_rel];
  
  A = [ 1  0   0   0  -1;
        0  1   0   0  -1;
       -1  0   1   0   0;
        0 -1   0   1   0;
        0  0   0   0   1];
  % old absolute coordinates in terms of relative coords
  q_b = inv(A)*q_rel; 
  dq_b = inv(A)*dq_rel;

%   D = replace_constants(D);
%   C = replace_constants(C);
%   G = replace_constants(G);

%   A_inv_trans = inv(A)';
%   
%   D_rel = subs(A_inv_trans*D, ...
% 	       {q31,q32,q41,q42,q1}, ...
% 	       {q_b(1),q_b(2),q_b(3),q_b(4),q_b(5)},0);
%   C_rel = subs(A_inv_trans*C, ...
% 	       {q31,q32,q41,q42,q1,dq31,dq32,dq41,dq42,dq1}, ...
% 	       {q_b(1),q_b(2),q_b(3),q_b(4),q_b(5), ...
% 		dq_b(1),dq_b(2),dq_b(3),dq_b(4),dq_b(5)},0);
%   G_rel = subs(A_inv_trans*G, ...
% 	       {q31,q32,q41,q42,q1}, ...
% 	       {q_b(1),q_b(2),q_b(3),q_b(4),q_b(5)},0);
%   B_rel = A_inv_trans*B;

  PE = replace_constants(model_data.PE);
  KE = replace_constants(KE);  
  
  PE_rel = subs(PE, ...
		{q31,q32,q41,q42,q1}, ...
		{q_b(1),q_b(2),q_b(3),q_b(4),q_b(5)},0);

  KE_rel = subs(KE, ...
		{q31,q32,q41,q42,q1,dq31,dq32,dq41,dq42,dq1}, ...
		{q_b(1),q_b(2),q_b(3),q_b(4),q_b(5), ...
		 dq_b(1),dq_b(2),dq_b(3),dq_b(4),dq_b(5)},0);

  G_rel = jacobian(PE_rel,q_rel).';
  G_rel = simple(G_rel);
  D_rel = jacobian(KE_rel,dq_rel).';
  D_rel = jacobian(D_rel,dq_rel);

  syms C real
  n=max(size(q_rel));
  for k=1:n
    for j=1:n
      C_rel(k,j)=0*g;
      for i=1:n
	C_rel(k,j)=C_rel(k,j)+1/2*(diff(D_rel(k,j),q_rel(i)) + ...
				   diff(D_rel(k,i),q_rel(j)) - ...
				   diff(D_rel(i,j),q_rel(k)))*dq_rel(i);
      end
    end
  end
  C_rel = C_rel;
  
  Phi_0_rel = subs(Phi_0, ...
		   {q31,q32,q41,q42,q1}, ...
		   {q_b(1),q_b(2),q_b(3),q_b(4),q_b(5)},0);

  Phi_0_rel = subs(Phi_0, ...
		   {q31,q32,q41,q42,q1}, ...
		   {q_b(1),q_b(2),q_b(3),q_b(4),q_b(5)},0);
  B_rel = jacobian(Phi_0_rel,q_rel);
  B_rel = B_rel'*[eye(4);zeros(1,4)];
  
  [n,m] = size(B_rel);
  F = -C_rel*dq_rel-G_rel;
  D21_rel = D_rel((m+1):n,1:m);
  D22_rel = D_rel((m+1):n,(m+1):n);
  F1 = F(1:m);
  F2 = F((m+1):n);
  
  %  M_bar = simple(M11-M12*inv(M22)*M21);
  %  F_bar = simple(F1-M12*inv(M22)*F2);
  %  B_bar = N*B;
  %  B_bar = B_bar(1:m,:);
  %  J = simple(-inv(M22)*M21);
  %  R = simple(-inv(M22)*F2); % u=inv(B_bar)*[M_bar*v + F_bar]
  %  F_new = [dq;zeros(4,1);R];
  %  G_new = [zeros(5,4);eye(4);J];
  
  %needed to complete coordinate change...solution is a consequence
  %of Frobenius' Theorem...see below
  %
  D_bar = [eye(4) zeros(4,1); D21_rel D22_rel];
  CG_bar = [eye(4,1); F2];
  
  % the output function and its time derivative
  H_rel = subs(h, ...
	     {q31,q32,q41,q42,q1}, ...
	     {q_b(1),q_b(2),q_b(3),q_b(4),q_b(5)},0);
  LfH_rel = jacobian(H_rel,q_rel)*dq_rel;
  
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Change of coordinates -- attempting to complete with a result of
  % Frobenius's Theorem
  %
  xh = [H_rel;LfH_rel;(q_b(1)+q_b(3))/2];
  
  % compute augmented B matrix
  B_aug = [B_rel,[zeros(4,1);1]];
  
  % compute last entry of change of coordinates as last row of ...
  tmp = inv(B_aug)*D_bar*dq_rel;
  xh = [xh; tmp(5)];

  tmp_abs = inv([B [zeros(4,1);1]])*D*dq;
  gamma_abs = replace_constants(tmp_abs(5));
  
  xh = replace_constants(xh);
  
  xh = xh;
  disp('[Computed change of coordinates]');
  

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Invert map
  %
  syms dz1 dz2 z1 z2 real
  
  syms P1 P2 P3 P4 real % other terms are zero
  P_tmp = [P1; P2; P3; P4];
  
  Ah_rel = Ah*inv(A);
  
  q_rel_s_zd = inv(Ah_rel)*[P_tmp;z1];
  
  q_rel_s_zd = simple(q_rel_s_zd);
  
  disp(['[computed position coordinates restricted to zero dynamics' ...
	' manifold]'])
  
  syms jac_P1 jac_P2 jac_P3 jac_P4 real
  temp_1 = [jac_P1; jac_P2; jac_P3; jac_P4];
  theta = (q_b(1)+q_b(3))/2;
  temp_1 = [-temp_1*jacobian(theta,q_rel); ...
	    D21_rel D22_rel];
  
  disp(['[computed parts needed to compute velocity coordinates' ...
	' restricted to zero dynamics manifold]']);
  
  dq_rel_s_zd_almost = inv([Ah_rel(1:4,:);zeros(1,5)] + temp_1)* ...
      [zeros(4,1); z2];
  
  dq_rel_s_zd = subs(dq_rel_s_zd_almost, q1_rel, q_rel_s_zd(1),0);
  dq_rel_s_zd = subs(dq_rel_s_zd, q2_rel, q_rel_s_zd(2),0);
  dq_rel_s_zd = subs(dq_rel_s_zd, q3_rel, q_rel_s_zd(3),0);
  dq_rel_s_zd = subs(dq_rel_s_zd, q4_rel, q_rel_s_zd(4),0);
  dq_rel_s_zd = subs(dq_rel_s_zd, q5_rel, q_rel_s_zd(5),0);
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Take time derivative in new coordinates...the system with the
  % change of coordinates and decoupling feedback (minus the last
  % two states)...we know this is the form because this is what
  % feedback linearization does!...
  %
  %syms v1 v2 v3 v4 real % inputs
  %dxh = [xh_5; xh_6; xh_7; xh_8; v1; v2; v3; v4];
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Now for the zero dynamics -- assume all outputs are zero
  %
  disp('[Calculating zero dynamics...]');
  
  dz1 = simple(jacobian(theta,q_rel)*dq_rel_s_zd)
  F2 = replace_constants(F2);
  dz2 = -G_rel(n);

  % now dxh has all its entries
  %dxh_zd = [dxh_zd, dz2];
  
  dz2 = subs(dz2,q1_rel,q_rel_s_zd(1),0);
  dz2 = subs(dz2,q2_rel,q_rel_s_zd(2),0);
  dz2 = subs(dz2,q3_rel,q_rel_s_zd(3),0);
  dz2 = subs(dz2,q4_rel,q_rel_s_zd(4),0);
  dz2 = subs(dz2,q5_rel,q_rel_s_zd(5),0);
  dz2 = simple(dz2)
  disp('[The zero dynamics in z1, z2 coordinates !]');

  save mat_files/work_symb_zero_dynamics
  disp('[save mat_files/work_symb_zero_dynamics]');
else
  load mat_files/work_symb_zero_dynamics
  disp('[DONE loading work_symb_zero_dynamics.mat]')
end

file_name = 'dz1';
disp(['[creating ',upper(file_name),'.txt]']);
fid = fopen(['doc/',file_name,'.txt'],'w');
[r,s] = subexpr(dz1,'s');
fprintf(fid,'%s\n',num2str(length(s)));
fprintf(fid,'%s\n',latex(r));
for k = 1:length(s)
  fprintf(fid,'s(%s)&=&%s\n',num2str(k),latex(s(k)));
end
fclose(fid);

file_name = 'dz2';
disp(['[creating ',upper(file_name),'.txt]']);
fid = fopen(['doc/',file_name,'.txt'],'w');
fprintf(fid,'0\n');
fprintf(fid,'%s\n',latex(dz2));
fclose(fid);

fcn_name = 'zd_project';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
n=max(size(q));
fprintf(fid,'function z =%s(x)\n',fcn_name); 
fprintf(fid,['%%%s    Projects the zero dynamics to the zero' ...
	     ' dynamics.\n'],upper(fcn_name));
fprintf(fid,'%%\n%%    Z = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

% Reassign configuration parameters
fprintf(fid,'q31=x(1); q32=x(2); q41=x(3); q42=x(4); q1=x(5);\n');
fprintf(fid,'dq31=x(6); dq32=x(7); dq41=x(8); dq42=x(9); dq1=x(10);\n\n');

fprintf(fid,'%% theta (in absolute coordinates)\n');
fprintf(fid,'z(1) = (q31+q41)/2;\n\n');

fprintf(fid,'%% gamma (in absolute coordinates)\n');
fprintf(fid,'z(2) = %s;',char(gamma_abs));
fclose(fid);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Automatically generate .m-files needed for simulation
%
fcn_name = 'zd';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
n=max(size(q));
header=['function dz = ',fcn_name,'(z)'];
fprintf(fid,'%s\n',header); 
fprintf(fid,'%%%s\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'[n,m] = size(z);\n');
fprintf(fid,'if n == 1 | m == 1\n');
fprintf(fid,'  z1 = z(1); z2 = z(2);\n');
fprintf(fid,'else\n');
fprintf(fid,'  z1 = z(:,1); z2 = z(:,2);\n');
fprintf(fid,'end\n\n');

fprintf(fid,'[a,b,m] = controlParameters;\n\n');

a = char('a');
for j = 0:3
  for k = 0:NN
    fprintf(fid,[char(a+j),num2str(k),'=a(',num2str(1+k+j*(NN+1)),'); ']);
  end
  fprintf(fid,'\n');
end
fprintf(fid,'\n');

jac_P = jacobian(P,theta1);
jac_P = replace_constants(jac_P);
jac_P = subs(jac_P,theta1,z1);
P = replace_constants(P);
P = subs(P,theta1,z1);

ttt = vectorize(char(P(1)));
fprintf(fid,'P1 = %s;\n\n',fixlength(ttt,'*+-',65,'     '));

ttt = char(vectorize(P(2)));
fprintf(fid,'P2 = %s;\n\n',fixlength(ttt,'*+-',65,'     '));

ttt = char(vectorize(P(3)));
fprintf(fid,'P3 = %s;\n\n',fixlength(ttt,'*+-',65,'     '));

ttt = char(vectorize(P(4)));
fprintf(fid,'P4 = %s;\n\n',fixlength(ttt,'*+-',65,'     '));

ttt = char(vectorize(jac_P(1)));
fprintf(fid,'jac_P1 = %s;\n\n',fixlength(ttt,'*+-',65,'         '));

ttt = char(vectorize(jac_P(2)));
fprintf(fid,'jac_P2 = %s;\n\n',fixlength(ttt,'*+-',65,'         '));

ttt = char(vectorize(jac_P(3)));
fprintf(fid,'jac_P3 = %s;\n\n',fixlength(ttt,'*+-',65,'         '));

ttt = char(vectorize(jac_P(4)));
fprintf(fid,'jac_P4 = %s;\n\n',fixlength(ttt,'*+-',65,'         '));


% State one
%
clear r s
[r,s] = subexpr(dz1,'s');
if length(s) == 0
elseif length(s) > 1
  for k = 1:length(s)
    ttt = char(vectorize(s(k)));
    fprintf(fid,'s(%s) = %s;\n',num2str(k), ...
	    fixlength(ttt,'*+-',65,'       '));
  end
else
  ttt = char(vectorize(s));
  fprintf(fid,'s = %s;\n',fixlength(ttt,'*+-',65,'    '));
end
fprintf(fid,'\n');

ttt = char(vectorize(r));
fprintf(fid,'%% dz1\n');
fprintf(fid,'dz_1 = %s;\n\n',fixlength(ttt,'*+-',65,'       '));


% State two
%
ttt = char(vectorize(dz2));
fprintf(fid,'%% dz2\n');
fprintf(fid,'dz_2 = %s;\n\n',fixlength(ttt,'*+-',65,'       '));

fprintf(fid,'[n,m] = size(z);\n');
fprintf(fid,'if n == 1 | m == 1\n');
fprintf(fid,'  dz = [dz_1; dz_2];\n');
fprintf(fid,'else\n');
fprintf(fid,'  dz = [dz_1, dz_2];\n');
fprintf(fid,'end');
fclose(fid);


% ------------------------------------------------------

function y = replace_constants(x)

syms g b m
syms L1 L3 L4 M1 M3 M4 MY1 MZ1 MZ3 MZ4 XX1 XX3 XX4

[a,b_,m_] = controlParameters;

[g_,L1_,L3_,L4_,M1_,M3_,M4_,MY1_,MZ1_,MZ3_,MZ4_,XX1_,XX3_,XX4_] = ...
    modelParameters;

y = subs(x,{g, ...
	    L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4}, ...
	 {g_, ...
	  L1_,L3_,L4_,M1_,M3_,M4_,MY1_,MZ1_,MZ3_,MZ4_,XX1_, ...
	  XX3_,XX4_},0);