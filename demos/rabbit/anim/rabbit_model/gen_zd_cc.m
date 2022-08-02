function gen_zd_cc
%GEN_ZD_CC    Derive a change of coordinates for the zero dynamics
%    to go from [theta_1 gamma] coordinates to [theta; dtheta]
%    coordinates.

%Eric Westervelt
%2/15/01

clear *

if isempty(dir('mat_files/work_symb_zero_dynamics.mat'))
  error('WORK_SYMB_ZERO_DYNAMICS.MAT not found...run GEN_ZD.M first');
else
  load mat_files/work_symb_zero_dynamics
end

fcn_name = 'zd_project_cc';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
n=max(size(q));
fprintf(fid,'function z = %s(x)\n',fcn_name);
fprintf(fid,['%%%s    Projects the zero dynamics to the zero' ...
	     ' dynamics.\n'],upper(fcn_name));
fprintf(fid,'%%\n');
fprintf(fid,'%%    Z = %s(X)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'%% first project\n');
fprintf(fid,'z = zd_project(x);\n');
fprintf(fid,'dz = zd(z);\n');
fprintf(fid,'x_zd = zd_lift(z(1),dz(1));\n\n');

fprintf(fid,'%% theta\n');
fprintf(fid,'z(1) = 1/2*(x_zd(1)+x_zd(3));\n\n');

fprintf(fid,'%% dtheta\n');
fprintf(fid,'z(2) = 1/2*(x_zd(6)+x_zd(8));');
fclose(fid);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Automatically generate .m-file needed for simulation
%
fcn_name = 'zd_cc';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
n=max(size(q));
fprintf(fid,'function dz = %s(z)\n',fcn_name);
fprintf(fid,'%%%s\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'z1 = z(1); z2 = z(2);\n\n');

fprintf(fid,'[a,b,m] = controlParameters;\n\n');

fprintf(fid,['a0=a(5); a1=a(6); a2=a(7); a3=a(8); a4=a(9);' ...
	     ' a5=a(10); a6=a(11);\n']); 
fprintf(fid,['b0=a(12); b1=a(13); b2=a(14); b3=a(15); b4=a(16);' ...
	     ' b5=a(17); b6=a(18);\n']); 
fprintf(fid,['c0=a(19); c1=a(20); c2=a(21); c3=a(22); c4=a(23);' ...
	     ' c5=a(24); c6=a(25);\n']); 
fprintf(fid,['d0=a(26); d1=a(27); d2=a(28); d3=a(29); d4=a(30);' ...
	     ' d5=a(31); d6=a(32);\n\n']); 

jac_Ph = jacobian(Ph,theta1);
jac_Ph = replace_constants(jac_Ph);
jac_Ph = subs(jac_Ph,theta1,z1);
Ph = replace_constants(Ph);
Ph = subs(Ph,theta1,z1);

ttt = char(Ph(1));
fprintf(fid,'Ph1 = %s;\n\n',fixlength(ttt,'*',75));

ttt = char(Ph(2));
fprintf(fid,'Ph2 = %s;\n\n',fixlength(ttt,'*',75));

ttt = char(Ph(3));
fprintf(fid,'Ph3 = %s;\n\n',fixlength(ttt,'*',75));

ttt = char(Ph(4));
fprintf(fid,'Ph4 = %s;\n\n',fixlength(ttt,'*',75));


% In new coordinates!!  theta1, dtheta1; but still need beta (was
% called dz_2) for calculation of dz_2 in new coordinates...see
% below
%
% (see notes 1/22/01 for calculations)
%
% stuff needed to calculate zerod dynamics in coordinates of
%
% z = [theta; dtheta]
%
disp('[calculating the new coordinates]');
alpha = dz1/z2;
alpha_subs = subs(alpha, ...
		  {Ph1,Ph2,Ph3,Ph4,jac_Ph1,jac_Ph2,jac_Ph3,jac_Ph4}, ...
		  {Ph(1),Ph(2),Ph(3),Ph(4),jac_Ph(1),jac_Ph(2), ...
		   jac_Ph(3),jac_Ph(4)},0);
alpha_subs = subs(alpha_subs,theta1,z1,0);
jac_alpha_subs = jacobian(alpha_subs,z1);
save mat_files/work_symb_zero_dynamics_cc
disp('[save mat_files/work_symb_zero_dynamics_cc]');

[r_alpha_subs,s_alpha_subs] = subexpr(alpha_subs,'s');
if length(s_alpha_subs) == 0
elseif length(s_alpha_subs) > 1
  for k = 1:length(s_alpha_subs)
    ttt = char(s_alpha_subs(k));
    fprintf(fid,'s(%s) = %s;\n',num2str(k),fixlength(ttt,'*',75));
  end
else
  ttt = char(s_alpha_subs);
  fprintf(fid,'s = %s;\n',fixlength(ttt,'*',75));
end
fprintf(fid,'\n');
ttt = char(r_alpha_subs);
fprintf(fid,'%% alpha\n');
ttt_fl = fixlength(ttt,'*',75);
fprintf(fid,'alpha = %s;\n\n',ttt_fl);

[r_jac_alpha_subs,s_jac_alpha_subs] = subexpr(jac_alpha_subs,'s');
if length(s_jac_alpha_subs) == 0
elseif length(s_jac_alpha_subs) > 1
  for k = 1:length(s_jac_alpha_subs)
    ttt = char(s_jac_alpha_subs(k));
    fprintf(fid,'s(%s) = %s;\n',num2str(k),fixlength(ttt,'*',75));
  end
else
  ttt = char(s_jac_alpha_subs);
  fprintf(fid,'s = %s;\n',fixlength(ttt,'*',75));
end
fprintf(fid,'\n');
ttt = char(r_jac_alpha_subs);
ttt_fl = fixlength(ttt,'*',75);
fprintf(fid,'jac_alpha = %s;\n\n',ttt_fl);

fprintf(fid,'%% change to original z coordinates\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% NOTE! z1 (in old coords) = z1 (in new coords)\n');
fprintf(fid,'z_old = [z1; 1/alpha*z2];\n\n');
fprintf(fid,'dz_old = zd(z_old);\n\n');
fprintf(fid,'dz = [z2; jac_alpha/alpha*z2^2+alpha*dz_old(2)];');
fclose(fid);


% ------------------------------------------------------

function y = replace_constants(x)

syms ah1 ah2 ah3 ah4
syms g b m
syms L1 L3 L4 M1 M3 M4 MY1 MZ1 MZ3 MZ4 XX1 XX3 XX4

[a,b_,m_] = controlParameters;

[g_,L1_,L3_,L4_,M1_,M3_,M4_,MY1_,MZ1_,MZ3_,MZ4_,XX1_,XX3_,XX4_] = ...
    modelParameters;

ah1_ = a(1); ah2_ = a(2); ah3_ = a(3); ah4_ = a(4);

y = subs(x,{g,ah1,ah2,ah3,ah4,b,m,...
	    L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4}, ...
	 {g_,ah1_,ah2_,ah3_,ah4_,b_,m_ ...
	  L1_,L3_,L4_,M1_,M3_,M4_,MY1_,MZ1_,MZ3_,MZ4_,XX1_, ...
	  XX3_,XX4_},0);