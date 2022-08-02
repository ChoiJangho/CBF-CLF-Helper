function gen_zd_diff_cc
%GEN_ZD_CC    Derive a change of coordinates for the zero dynamics
%    to go from [theta_1 gamma] coordinates to [theta; dtheta]
%    coordinates.

%Eric Westervelt
%2/15/01

clear *

global ALPHA_SUBS JAC_ALPHA_SUBS BETA_SUBS

if isempty(dir('mat_files/zd_work5.mat'))
  error('ZD_WORK5.MAT not found...run GEN_ZD.M first');
else
  load mat_files/zd_work5
end

if 0
  if 1
    syms a0_0 a1_0 a2_0 a3_0 a4_0 a5_0
    syms b0_0 b1_0 b2_0 b3_0 b4_0 b5_0
    syms c0_0 c1_0 c2_0 c3_0 c4_0 c5_0
    syms d0_0 d1_0 d2_0 d3_0 d4_0 d5_0
    
    jac_Ph = jacobian(Ph,theta1);
    jac_Ph = replace_constants(jac_Ph);
    jac_Ph = subs(jac_Ph,theta1,z1);
    Ph = subs(Ph,theta1,z1);
    Ph = replace_constants(Ph);
    
    % In new coordinates!!  theta1, dtheta1; but still need beta (was
    % called dz_2) for calculation of dz_2 in new coordinates...see below
    %
    % (see notes 2/26/01 for calculations)
    %
    % stuff needed to calculate zerod dynamics in coordinates of
    %
    % z = [theta; dtheta]
    %
    disp('[calculating the new coordinates]');
    alpha = dz1/z2;
    ALPHA_SUBS = subs(alpha, ...
		      {Ph1,Ph2,Ph3,Ph4,jac_Ph1,jac_Ph2,jac_Ph3,jac_Ph4}, ...
		      {Ph(1),Ph(2),Ph(3),Ph(4),jac_Ph(1),jac_Ph(2), ...
		       jac_Ph(3),jac_Ph(4)},0);
    ALPHA_SUBS = subs(ALPHA_SUBS,theta1,z1,0);
    ALPHA_SUBS = replace_constants(ALPHA_SUBS);
    JAC_ALPHA_SUBS = jacobian(ALPHA_SUBS,z1);
    BETA_SUBS = subs(dz2,Ph1,Ph(1));
    BETA_SUBS = subs(BETA_SUBS,Ph2,Ph(2));
    BETA_SUBS = subs(BETA_SUBS,Ph3,Ph(3));
    BETA_SUBS = subs(BETA_SUBS,Ph4,Ph(4));
    BETA_SUBS = replace_constants(BETA_SUBS);
    save mat_files/zd_workA
    disp('[save mat_files/zd_workA]');
  end
  load mat_files/zd_workA
  
  partial_a0 = compute_diff_sens(a0, a0_0);
  partial_a1 = compute_diff_sens(a1, a1_0);
  partial_a2 = compute_diff_sens(a2, a2_0);
  partial_a3 = compute_diff_sens(a3, a3_0);
  partial_a4 = compute_diff_sens(a4, a4_0);
  partial_a5 = compute_diff_sens(a5, a5_0);
  
  disp(' ')
  
  partial_b0 = compute_diff_sens(b0, b0_0);
  partial_b1 = compute_diff_sens(b1, b1_0);
  partial_b2 = compute_diff_sens(b2, b2_0);
  partial_b3 = compute_diff_sens(b3, b3_0);
  partial_b4 = compute_diff_sens(b4, b4_0);
  partial_b5 = compute_diff_sens(b5, b5_0);
  
  disp(' ')
  
  partial_c0 = compute_diff_sens(c0, c0_0);
  partial_c1 = compute_diff_sens(c1, c1_0);
  partial_c2 = compute_diff_sens(c2, c2_0);
  partial_c3 = compute_diff_sens(c3, c3_0);
  partial_c4 = compute_diff_sens(c4, c4_0);
  partial_c5 = compute_diff_sens(c5, c5_0);
  
  disp(' ')
  
  partial_d0 = compute_diff_sens(d0, d0_0);
  partial_d1 = compute_diff_sens(d1, d1_0);
  partial_d2 = compute_diff_sens(d2, d2_0);
  partial_d3 = compute_diff_sens(d3, d3_0);
  partial_d4 = compute_diff_sens(d4, d4_0);
  partial_d5 = compute_diff_sens(d5, d5_0);
  
  save mat_files/zd_workB
  disp('[save mat_files/zd_workB]');
end
load mat_files/zd_workB
disp('[DONE computing/loading]')


if 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Automatically generate .m-file needed for simulation
%
fcn_name = 'zd_diff_cc';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
n=max(size(q));
fprintf(fid,'function dz = %s(z,opt)\n',fcn_name); 
fprintf(fid,'%%%s\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'[a,b,m] = controlParameters;\n\n');

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

clear r s
[r,s] = subexpr(ALPHA_SUBS,'s');
if length(s) == 0
elseif length(s) > 1
  for k = 1:length(s)
    ttt = char(s(k));
    fprintf(fid,'s(%s) = %s;\n',num2str(k),fixlength(ttt,'*',75));
  end
else
  fprintf(fid,'s = %s;\n',fixlength(char(s),'*',75));
end
fprintf(fid,'\n%% alpha\n');
ttt_fl = fixlength(char(r),'*',75);
fprintf(fid,'alpha = %s;\n\n',ttt_fl);

clear r s
[r,s] = subexpr(BETA_SUBS,'s');
if length(s) == 0
elseif length(s) > 1
  for k = 1:length(s)
    ttt = char(s(k));
    fprintf(fid,'s(%s) = %s;\n',num2str(k),fixlength(ttt,'*',75));
  end
else
  fprintf(fid,'s = %s;\n',fixlength(char(s),'*',75));
end
fprintf(fid,'\n%% beta\n');
ttt_fl = fixlength(char(r),'*',75);
fprintf(fid,'beta = %s;\n\n',ttt_fl);

clear r s
[r,s] = subexpr(JAC_ALPHA_SUBS,'s');
if length(s) == 0
elseif length(s) > 1
  for k = 1:length(s)
    ttt = char(s(k));
    fprintf(fid,'s(%s) = %s;\n',num2str(k),fixlength(ttt,'*',75));
  end
else
  fprintf(fid,'s = %s;\n',fixlength(char(s),'*',75));
end
ttt = char(r);
ttt_fl = fixlength(ttt,'*',75);
fprintf(fid,'\njac_alpha = %s;\n\n',ttt_fl);

fprintf(fid,'switch opt\n');
for k = 1:24
  fprintf(fid,'  case %s,\n',num2str(k));
  switch floor((k-1)/6)
   case 0, chr = 'a';
   case 1, chr = 'b';
   case 2, chr = 'c';
   case 3, chr = 'd';
  end
  fprintf(fid,['    [partial_alpha,partial_beta,partial_jac_alpha] =' ...
	       ' zd_diff_cc_',chr,'%s(z1);\n'],num2str(mod(k-1,6)));
end
fprintf(fid,'  otherwise\n');
fprintf(fid,'    error(''OPT parameter out of range'');\n');
fprintf(fid,'end\n\n');

% State one
fprintf(fid,'%% state one\n');
fprintf(fid,'dz(1) = 0\n\n');%z2*partial_alpha;\n\n');

% State two
fprintf(fid,'%% state two\n');
%fprintf(fid,'dz(2) = partial_jac_alpha*z2^2*alpha ...\n');
%fprintf(fid,'        + jac_alpha*z2^2*partial_alpha ...\n');
%fprintf(fid,'        + partial_alpha*beta ...\n');
%fprintf(fid,'        + alpha*partial_beta;');
fprintf(fid,'dz(2) = z2^2/alpha^2*partial_alpha*jac_alpha ...\n');
fprintf(fid,'        + z2^2/alpha*partial_jac_alpha ...\n');
fprintf(fid,'        + partial_alpha*beta ...\n');
fprintf(fid,'        + alpha*partial_beta;');
fclose(fid);
end
%
%
%
return
%
for k = 1:24
  switch floor((k-1)/6)
   case 0, chr = 'a';
   case 1, chr = 'b';
   case 2, chr = 'c';
   case 3, chr = 'd';
  end
  fcn_name = ['zd_diff_cc_',chr,num2str(mod(k-1,6))];
  disp(['[creating ',upper(fcn_name),'.m]']);
  fid = fopen([fcn_name,'.m'],'w');
  n=max(size(q));
  fprintf(fid,['function [partial_alpha,partial_beta,partial_jac_alpha] =' ...
	       ' %s(z1)\n'],fcn_name);
  fprintf(fid,'%%%s\n\n',upper(fcn_name));
  fprintf(fid,'%%Eric Westervelt\n');
  fprintf(fid,'%%%s\n\n',datestr(now));
  
  fprintf(fid,'[a,b,m] = controlParameters;\n\n');
  
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
  fprintf(fid,['b0_0=a(11); b1_0=a(12); b2_0=a(13); b3_0=a(14);' ...
	       ' b4_0=a(15); b5_0=a(16);\n']); 
  fprintf(fid,['c0_0=a(17); c1_0=a(18); c2_0=a(19); c3_0=a(20);' ...
	       ' c4_0=a(21); c5_0=a(22);\n']); 
  fprintf(fid,['d0_0=a(23); d1_0=a(24); d2_0=a(25); d3_0=a(26);' ...
	       ' d4_0=a(27); d5_0=a(28);\n\n']); 
  
  switch k
   case 1, print_diff_sens(fid,partial_a0);
   case 2, print_diff_sens(fid,partial_a1);
   case 3, print_diff_sens(fid,partial_a2);
   case 4, print_diff_sens(fid,partial_a3);
   case 5, print_diff_sens(fid,partial_a4);
   case 6, print_diff_sens(fid,partial_a5);
  
   case 7, print_diff_sens(fid,partial_b0);
   case 8, print_diff_sens(fid,partial_b1);
   case 9, print_diff_sens(fid,partial_b2);
   case 10, print_diff_sens(fid,partial_b3);
   case 11, print_diff_sens(fid,partial_b4);
   case 12, print_diff_sens(fid,partial_b5);
  
   case 13, print_diff_sens(fid,partial_c0);
   case 14, print_diff_sens(fid,partial_c1);
   case 15, print_diff_sens(fid,partial_c2);
   case 16, print_diff_sens(fid,partial_c3);
   case 17, print_diff_sens(fid,partial_c4);
   case 18, print_diff_sens(fid,partial_c5);
  
   case 19, print_diff_sens(fid,partial_d0);
   case 20, print_diff_sens(fid,partial_d1);
   case 21, print_diff_sens(fid,partial_d2);
   case 22, print_diff_sens(fid,partial_d3);
   case 23, print_diff_sens(fid,partial_d4);
   case 24, print_diff_sens(fid,partial_d5);
  end
end


% ------------------------------------------------------

function  partial_xx = compute_diff_sens(xx, xx_0)

global ALPHA_SUBS JAC_ALPHA_SUBS BETA_SUBS

xx_sym = sym(inputname(1));

% partial alpha/partial k_i
partial_xx(1) = jacobian(ALPHA_SUBS,xx_sym);
partial_xx(1) = subs(partial_xx(1),xx_sym,xx_0,0);
disp(['[computed partial alpha/partial ',inputname(1),']']);

% partial beta/partial k_i
partial_xx(2) = jacobian(BETA_SUBS,xx_sym);
partial_xx(2) = subs(partial_xx(2),xx_sym,xx_0,0);
disp(['[computed partial beta/partial ',inputname(1),']']);

% partial/partial k_i (partial alpha/partial z_1)
partial_xx(3) = jacobian(JAC_ALPHA_SUBS,xx_sym);
partial_xx(3) = subs(partial_xx(3),xx_sym,xx_0,0);
disp(['[computed partial/partial ',inputname(1),...
      ' (partial alpha/partial z_1)]']);


% ------------------------------------------------------

function  print_diff_sens(fid, partial_xx)

% partial_alpha
clear r s
[r,s] = subexpr(partial_xx(1),'s');
for k = 1:length(s)
  fprintf(fid,'s(%s) = %s;\n',num2str(k), ...
	  fixlength(char(vpa(s(k),10)),'*+^',65));
end
fprintf(fid,'\n');
fprintf(fid,'partial_alpha = %s;\n\n', ...
	fixlength(char(vpa(r,10)),'*+^',65));

% partial_beta
clear r s
[r,s] = subexpr(partial_xx(2),'s');
for k = 1:length(s)
  fprintf(fid,'s(%s) = %s;\n',num2str(k), ...
	  fixlength(char(vpa(s(k),10)),'*+^',65));
end
fprintf(fid,'\n');
fprintf(fid,'partial_beta = %s;\n\n', ...
	fixlength(char(vpa(r,10)),'*+^',65));

% partial_jac_alpha
clear r s
[r,s] = subexpr(partial_xx(3),'s');
for k = 1:length(s)
  fprintf(fid,'s(%s) = %s;\n',num2str(k), ...
	  fixlength(char(vpa(s(k),10)),'*+^',65));
end
fprintf(fid,'\n');
fprintf(fid,'partial_jac_alpha = %s;\n\n', ...
	fixlength(char(vpa(r,10)),'*+^',65));


% ------------------------------------------------------

function y = replace_constants(x)

syms ah1 ah2 ah3 ah4

a = controlParameters;

ah1_ = a(1); ah2_ = a(2); ah3_ = a(3); ah4_ = a(4);

y = subs(x,{ah1,ah2,ah3,ah4},{ah1_,ah2_,ah3_,ah4_},0);