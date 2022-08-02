function gen_zd_diff
%GEN_ZD_DIFF    Derive the first order sensitivies of the zero
%    dynamics to the control parameters a0,...,a5, b0,...b5, and
%    c0,...,c5 for the kneed biped walker with Franck's offset masses
%    model using Bezier functions.

%Eric Westervelt
%1/10/01

clear *

if isempty(dir('mat_files/zd_work5.mat'))
  error('ZD_WORK5.MAT not found...run GEN_ZD.M first');
else
  load mat_files/zd_work5
end

for k = 1:3
  disp(' ');
  disp(['[WARNING!!  This script can take over 30 minutes to' ...
	' run.]']);
end
disp(' ');

if 1
  jac_Ph = jacobian(Ph,theta1);
  jac_Ph = replace_constants(jac_Ph);
  jac_Ph = subs(jac_Ph,theta1,z1);
  Ph = replace_constants(Ph);
  Ph = subs(Ph,theta1,z1);
  
  dz1_new = subs(dz1,Ph1,Ph(1));
  dz1_new = subs(dz1_new,Ph2,Ph(2));
  dz1_new = subs(dz1_new,Ph3,Ph(3));
  dz1_new = subs(dz1_new,Ph4,Ph(4));
  
  dz1_new = subs(dz1_new,jac_Ph1,jac_Ph(1));
  dz1_new = subs(dz1_new,jac_Ph2,jac_Ph(2));
  dz1_new = subs(dz1_new,jac_Ph3,jac_Ph(3));
  dz1_new = subs(dz1_new,jac_Ph4,jac_Ph(4));
  
  dz2_new = subs(dz2,Ph1,Ph(1));
  dz2_new = subs(dz2_new,Ph2,Ph(2));
  dz2_new = subs(dz2_new,Ph3,Ph(3));
  dz2_new = subs(dz2_new,Ph4,Ph(4));
  
  save mat_files/zd_work7
  disp('[save mat_files/zd_work7]');
    
  syms a0_0 a1_0 a2_0 a3_0 a4_0 a5_0
  syms b0_0 b1_0 b2_0 b3_0 b4_0 b5_0
  syms c0_0 c1_0 c2_0 c3_0 c4_0 c5_0
  syms d0_0 d1_0 d2_0 d3_0 d4_0 d5_0
  
  dz_partial_a0 = compute_diff_sens(dz1_new, dz2_new, a0, a0_0);
  dz_partial_a1 = compute_diff_sens(dz1_new, dz2_new, a1, a1_0);
  dz_partial_a2 = compute_diff_sens(dz1_new, dz2_new, a2, a2_0);
  dz_partial_a3 = compute_diff_sens(dz1_new, dz2_new, a3, a3_0);
  dz_partial_a4 = compute_diff_sens(dz1_new, dz2_new, a4, a4_0);
  dz_partial_a5 = compute_diff_sens(dz1_new, dz2_new, a5, a5_0);

  disp(' ')
  
  dz_partial_b0 = compute_diff_sens(dz1_new, dz2_new, b0, b0_0);
  dz_partial_b1 = compute_diff_sens(dz1_new, dz2_new, b1, b1_0);
  dz_partial_b2 = compute_diff_sens(dz1_new, dz2_new, b2, b2_0);
  dz_partial_b3 = compute_diff_sens(dz1_new, dz2_new, b3, b3_0);
  dz_partial_b4 = compute_diff_sens(dz1_new, dz2_new, b4, b4_0);
  dz_partial_b5 = compute_diff_sens(dz1_new, dz2_new, b5, b5_0);

  disp(' ')
  
  dz_partial_c0 = compute_diff_sens(dz1_new, dz2_new, c0, c0_0);
  dz_partial_c1 = compute_diff_sens(dz1_new, dz2_new, c1, c1_0);
  dz_partial_c2 = compute_diff_sens(dz1_new, dz2_new, c2, c2_0);
  dz_partial_c3 = compute_diff_sens(dz1_new, dz2_new, c3, c3_0);
  dz_partial_c4 = compute_diff_sens(dz1_new, dz2_new, c4, c4_0);
  dz_partial_c5 = compute_diff_sens(dz1_new, dz2_new, c5, c5_0);

  disp(' ')
  
  dz_partial_d0 = compute_diff_sens(dz1_new, dz2_new, d0, d0_0);
  dz_partial_d1 = compute_diff_sens(dz1_new, dz2_new, d1, d1_0);
  dz_partial_d2 = compute_diff_sens(dz1_new, dz2_new, d2, d2_0);
  dz_partial_d3 = compute_diff_sens(dz1_new, dz2_new, d3, d3_0);
  dz_partial_d4 = compute_diff_sens(dz1_new, dz2_new, d4, d4_0);
  dz_partial_d5 = compute_diff_sens(dz1_new, dz2_new, d5, d5_0);

  save mat_files/zd_work8
  disp('[save mat_files/zd_work8]');
end
load mat_files/zd_work8
disp('[DONE computing/loading]')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Automatically generate .m-file needed for simulation
%
fcn_name = 'zd_diff';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
n=max(size(q));
fprintf(fid,'function dz = %s(z,opt)\n',fcn_name); 
fprintf(fid,'%%%s\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'[a,b,m] = controlParameters;\n\n');

error('HERE -- need to fix the parameters...see SYMB_CONTROL.M')

fprintf(fid,['a0=a(5); a1=a(6); a2=a(7); a3=a(8); a4=a(9);' ...
	     ' a5=a(10);\n']); 
fprintf(fid,['b0=a(11); b1=a(12); b2=a(13); b3=a(14); b4=a(15);' ...
	     ' b5=a(16);\n']); 
fprintf(fid,['c0=a(17); c1=a(18); c2=a(19); c3=a(20); c4=a(21);' ...
	     ' c5=a(22);\n']); 
fprintf(fid,['d0=a(23); d1=a(24); d2=a(25); d3=a(26); d4=a(27);' ...
	     ' d5=a(28);\n\n']); 

fprintf(fid,['a0_0=a(5); a1_0=a(6); a2_0=a(7); a3_0=a(8); a4_0=a(9);' ...
	     ' a5_0=a(10);\n']); 
fprintf(fid,['b0_0=a(11); b1_0=a(12); b2_0=a(13); b3_0=a(14); b4_0=a(15);' ...
	     ' b5_0=a(16);\n']); 
fprintf(fid,['c0_0=a(17); c1_0=a(18); c2_0=a(19); c3_0=a(20); c4_0=a(21);' ...
	     ' c5_0=a(22);\n']); 
fprintf(fid,['d0_0=a(23); d1_0=a(24); d2_0=a(25); d3_0=a(26); d4_0=a(27);' ...
	     ' d5_0=a(28);\n\n']); 

fprintf(fid,'z1 = z(1); z2 = z(2);\n\n');

fprintf(fid,'switch opt\n');

num = 1;

num = print_diff_sens(fid,dz_partial_a0, num);
num = print_diff_sens(fid,dz_partial_a1, num);
num = print_diff_sens(fid,dz_partial_a2, num);
num = print_diff_sens(fid,dz_partial_a3, num);
num = print_diff_sens(fid,dz_partial_a4, num);
num = print_diff_sens(fid,dz_partial_a5, num);

num = print_diff_sens(fid,dz_partial_b0, num);
num = print_diff_sens(fid,dz_partial_b1, num);
num = print_diff_sens(fid,dz_partial_b2, num);
num = print_diff_sens(fid,dz_partial_b3, num);
num = print_diff_sens(fid,dz_partial_b4, num);
num = print_diff_sens(fid,dz_partial_b5, num);

num = print_diff_sens(fid,dz_partial_c0, num);
num = print_diff_sens(fid,dz_partial_c1, num);
num = print_diff_sens(fid,dz_partial_c2, num);
num = print_diff_sens(fid,dz_partial_c3, num);
num = print_diff_sens(fid,dz_partial_c4, num);
num = print_diff_sens(fid,dz_partial_c5, num);

num = print_diff_sens(fid,dz_partial_d0, num);
num = print_diff_sens(fid,dz_partial_d1, num);
num = print_diff_sens(fid,dz_partial_d2, num);
num = print_diff_sens(fid,dz_partial_d3, num);
num = print_diff_sens(fid,dz_partial_d4, num);
num = print_diff_sens(fid,dz_partial_d5, num);

fprintf(fid,'  otherwise\n');
fprintf(fid,'    error(''opt parameter out of range'');\n');
fprintf(fid,'end\n');
fclose(fid);


% ------------------------------------------------------

function  dz_partial_xx = compute_diff_sens(dz1, dz2, xx, xx_0)

xx_sym = sym(inputname(3));

% State one
dz_partial_xx(1) = jacobian(dz1,xx_sym);
dz_partial_xx(1) = subs(dz_partial_xx(1),xx_sym,xx_0,0);
disp(['[computed partial dz1/partial ',inputname(3),']'])

% State two
dz_partial_xx(2) = jacobian(dz2,xx_sym);
dz_partial_xx(2) = subs(dz_partial_xx(2),xx_sym,xx_0,0);
disp(['[computed partial dz2/partial ',inputname(3),']'])


% ------------------------------------------------------

function  new_num = print_diff_sens(fid, dz_partial_xx, num)

new_num = num + 1;

fprintf(fid,'  case %s,\n', num2str(num));

% State one
fprintf(fid,'   %% %s (state one)\n',inputname(2));
clear r s
[r,s] = subexpr(dz_partial_xx(1),'s');
for k = 1:length(s)
  fprintf(fid,'   s(%s) = %s;\n',num2str(k), ...
	  fixlength(char(vpa(s(k),10)),'*+^',65));
end
fprintf(fid,'\n');
fprintf(fid,'   dz(1) = %s;\n\n', ...
	num2str(num), ...
	fixlength(char(vpa(r,10)),'*+^',65));

% State two
fprintf(fid,'\n');
fprintf(fid,'   %% %s (state two)\n',inputname(2));
fprintf(fid,'   dz(2) = %s;\n\n', ...
	num2str(num), ...
	fixlength(char(vpa(dz_partial_xx(2),10)),'*+^',65));

disp(['[printed ',inputname(2),']']);


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