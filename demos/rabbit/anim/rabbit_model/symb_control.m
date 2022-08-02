%SYMB_CONTROL.M

%Eric Westervelt
%2/15/01

clear

if isempty(dir('mat_files/work_symb_control_abs_red_test.mat'))
  load mat_files/work_symb_model_abs_red

  which_one = 1;
  switch which_one
   case 1 % using bezier curves
    syms u theta1 dtheta1 real
    
    syms a0 a1 a2 a3 a4 a5 a6
    syms b0 b1 b2 b3 b4 b5 b6
    syms c0 c1 c2 c3 c4 c5 c6
    syms d0 d1 d2 d3 d4 d5 d6
    
    a = [a0 a1 a2 a3 a4 a5 a6];
    b = [b0 b1 b2 b3 b4 b5 b6];
    c = [c0 c1 c2 c3 c4 c5 c6];
    d = [d0 d1 d2 d3 d4 d5 d6];
    
    p1 = 0;
    p2 = 0;
    p3 = 0;
    p4 = 0;
    
    % form bezier parameterization of polynomials
    NN = 6;
    for k = 1:NN+1
      kk = k - 1;
      p1 = p1 + a(k)*factorial(NN)/ ...
	   factorial(kk)/factorial(NN-kk)*u^(kk)*(1-u)^(NN-kk);
      p2 = p2 + b(k)*factorial(NN)/ ...
	   factorial(kk)/factorial(NN-kk)*u^(kk)*(1-u)^(NN-kk);
      p3 = p3 + c(k)*factorial(NN)/ ...
	   factorial(kk)/factorial(NN-kk)*u^(kk)*(1-u)^(NN-kk);
      p4 = p4 + d(k)*factorial(NN)/ ...
	   factorial(kk)/factorial(NN-kk)*u^(kk)*(1-u)^(NN-kk);
    end
    
    syms b m

    p1 = subs(p1,u,(theta1-b)/m);
    p2 = subs(p2,u,(theta1-b)/m);
    p3 = subs(p3,u,(theta1-b)/m);
    p4 = subs(p4,u,(theta1-b)/m);
    
    A = [ 0  0  0  0  1;
	 -1  1  0  0  0;
	 -1  0  1  0  0;
	  0 -1  0  1  0];
    
    P = [p1;
	 p2;
	 p3;
	 p4];
    
    % The output function:
    %
    %   a linear term, constant term, plus a nonlinear term
    %   parameterized by theta
    %
    % A and P implement
    %
    % h = [q1 - p1;
    %      q32-q31 - p2;
    %      q41-q31 - p3;
    %      q42-q32 - p4];
    %
    h = A*q-P;
    Ah = [A; 1/2 0 1/2 0 0];
    
    Ah_inv = inv(Ah);
    
    %syms xh1 xh2 xh3 xh4 xh5
    %xh = [xh1; xh2; xh3; xh4; xh5];
    
    %syms xh6 xh7 xh8 xh9 xh10
    %dxh = [xh6; xh7; xh8; xh9; xh10];
    
    q_sigma_zd = Ah_inv*[P;theta1];
    dq_sigma_zd = Ah_inv*[jacobian(P, theta1)*dtheta1;dtheta1];
    
    %q_sigma_zd = subs(q_sigma, ...
    %{xh1,xh2,xh3,xh4,xh5,xh6,xh7,xh8,xh9,xh10}, ...
    %{0,0,0,0,theta1,0,0,0,0,dtheta1},0);
    %dq_sigma_zd = subs(dq_sigma, ...
    %{xh1,xh2,xh3,xh4,xh5,xh6,xh7,xh8,xh9,xh10}, ...
    %{0,0,0,0,theta1,0,0,0,0,dtheta1},0);
    
    %q32_sigma = simple(subs(q_sigma_zd(2),theta1,1/2*(q31+q41)));
    %q42_sigma = simple(subs(q_sigma_zd(4),theta1,1/2*(q31+q41)));
    %q1_sigma  = q_sigma_zd(5);
		    
    dq31_sigma = dq_sigma_zd(1);
    dq32_sigma = dq_sigma_zd(2);
    dq41_sigma = dq_sigma_zd(3);
    dq42_sigma = dq_sigma_zd(4);
    dq1_sigma  = dq_sigma_zd(5);
    
    syms aN bN cN dN
    q_end = Ah_inv*[aN; bN; cN; dN; theta1];
    
    obj_fun_foot_on_ground = subs(pFoot2(2),{q31,q32,q41,q42},...
				  {q_end(1),q_end(2),q_end(3),q_end(4)},0);
  end

  h = subs(h,theta1,1/2*(q31+q41));

  diffeo = [h; 1/2*(q31+q41)];
  
  jac_h = jacobian(h,q);         % y_dot  = jac_h * dq
  
  % the next command only made things worse
  %  jac_h = simple(jac_h);
  lfh = jac_h*dq;

  gamma = jacobian(jac_h*dq,q)*dq; % y_ddot = gamma + jac_h * ddq
  
  % the next command only made things worse
  %  gamma = simple(gamma);
  
  save mat_files/work_symb_control_abs_red_test
else
  load mat_files/work_symb_control_abs_red_test
  disp('[DONE loading work_symb_control_abs_red.mat]')
end

%------------------------------------------------------
%
% Automatically generate .m-files needed for simulation
%
fcn_name = 'decouple_abs_red';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
n=max(size(q));
fprintf(fid,['function [h,lfh,l2fh,lglfh,jac_h,inv_D,C,G,B]=' ...
	     ' %s(q,dq)\n'],fcn_name);
fprintf(fid,'%%%s\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));
fprintf(fid,'[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=');
fprintf(fid,' modelParameters;\n\n');

fprintf(fid,'[a,b,m] = controlParameters;\n\n');

a = char('a');
for j = 0:3
  for k = 0:NN
    fprintf(fid,[char(a+j),num2str(k),'=a(',num2str(1+k+j*(NN+1)),'); ']);
  end
  fprintf(fid,'\n');
end
fprintf(fid,'\n');

fprintf(fid,'q31=q(1);q32=q(2);q41=q(3);q42=q(4);q1=q(5);');
fprintf(fid,'\ndq31=dq(1);dq32=dq(2);dq41=dq(3);');
fprintf(fid,'dq42=dq(4);dq1=dq(5);');
fprintf(fid,'\n\ntheta1=1/2*(q31+q41);\n');
p=max(size(h));
fprintf(fid,'\n%s',['h=zeros(',num2str(p),',1);']);
for i=1:p
   Temp1=char(h(i));
   Temp2=['h(',num2str(i),')=',Temp1,';'];
   fprintf(fid,'\n%s',Temp2);
end
fprintf(fid,'\n%s','%%');
fprintf(fid,'\n%s','%%');
fprintf(fid,'\n%s','%%');
p=max(size(lfh));
fprintf(fid,'\n%s',['lfh=zeros(',num2str(p),',1);']);
for i=1:p
   Temp1=char(lfh(i));
   Temp2=['lfh(',num2str(i),')=',Temp1,';'];
   fprintf(fid,'\n%s',Temp2);
end
fprintf(fid,'\n%%');
fprintf(fid,'\n%%');
fprintf(fid,'\n%%');
[p,n]=size(jac_h);
fprintf(fid,'\n%s',['jac_h=zeros(',num2str(p),',',num2str(n),');']);
for i=1:p
   for j=1:n
      Temp0=jac_h(i,j);
      if Temp0 ~= 0
         Temp1=char(Temp0);
         Temp2=['jac_h(',num2str(i),',',num2str(j),')=',Temp1,';'];
         fprintf(fid,'\n%s',Temp2);
      end
   end
end
fprintf(fid,'\n%%');
fprintf(fid,'\n%%');
fprintf(fid,'\n%%');
p=max(size(gamma));
fprintf(fid,'\n%s',['gamma=zeros(',num2str(p),',1);']);
for i=1:p
   Temp1=char(gamma(i));
   Temp2=['gamma(',num2str(i),')=',Temp1,';'];
   fprintf(fid,'\n%s',Temp2);
end
fprintf(fid,'\n%%');
fprintf(fid,'\n%%');
fprintf(fid,'\n[D,C,G,B]=dyn_mod_abs_red([q;dq]);');
fprintf(fid,'\ninv_D=inv(D);');
fprintf(fid,'\nl2fh=gamma-jac_h*inv_D*(C*dq+G);');
fprintf(fid,'\nlglfh=jac_h*inv_D*B;');
fclose(fid);
%
%
%
fcn_name = 'sigma';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function x0 = %s(dtheta1,theta1)\n',fcn_name);
fprintf(fid,['%%%s    Maps horizontal velocity of hips just before' ...
	     ' impact to\n'],upper(fcn_name));
fprintf(fid,'%%    state of the system just before impact.\n');
fprintf(fid,'%%\n%%    X0 = %s(DTHETA1,THETA1)\n\n', ...
	upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'[a,b,m] = controlParameters;\n\n');

fprintf(fid,'Ah_inv = zeros(5);\n');
for k=1:5,
   for j=1:5,
      if Ah_inv(k,j)~=0
	ttt=num2str(Ah_inv(k,j));
	fprintf(fid,'Ah_inv(%s,%s)=%s;\n',num2str(k), ...
		num2str(j),ttt);
      end
   end
end
fprintf(fid,['\naN = a([',num2str(1+NN),';', ...
	     num2str(1+2*(NN+1)-1),';', ...
	     num2str(1+3*(NN+1)-1),';', ...
	     num2str(1+4*(NN+1)-1),']);\n']);
fprintf(fid,['aN_minus_1 = a([',num2str(1+NN-1),';', ...
	     num2str(1+2*(NN+1)-2),';', ...
	     num2str(1+3*(NN+1)-2),';', ...
	     num2str(1+4*(NN+1)-2),']);\n\n']);
fprintf(fid,'if nargin == 2\n');
fprintf(fid,'  q = Ah_inv*[aN; theta1];\n');
fprintf(fid,'else\n');
fprintf(fid,'  q = Ah_inv*[aN; b+m];\n');
fprintf(fid,'end\n\n');
fprintf(fid,['dq = Ah_inv*[-',num2str(NN),'*(aN_minus_1-aN)/m;' ...
		    ' 1]*dtheta1;\n\n']);
fprintf(fid,'x0 = [q; dq];');
fclose(fid);
%
%
%
fcn_name = 'theta_obj';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function f = %s(theta1)\n',fcn_name);
fprintf(fid,['%%%s     Function to find theta.\n%%\n'], ...
	upper(fcn_name));
fprintf(fid,'%%    F = %s(THETA1)\n',fcn_name);
fprintf(fid,'\n');
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'a = controlParameters;\n\n');

fprintf(fid,['aN=a(',num2str(1+NN),'); ', ...
	     'bN=a(',num2str(1+2*(NN+1)-1),'); ', ...
	     'cN=a(',num2str(1+3*(NN+1)-1),'); ', ...
	     'dN=a(',num2str(1+4*(NN+1)-1),');\n\n']); 

fprintf(fid,'[g,L1,L3,L4] = modelParameters;\n\n');
ttt=fixlength(char(vectorize(obj_fun_foot_on_ground)), ...
	      '*+-',65,'    ');
fprintf(fid,'f = %s;',ttt);
fclose(fid);
%
%
%
fcn_name = 'sigma_vel';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function dq = %s(dtheta1)\n',fcn_name);
fprintf(fid,['%%%s    Maps horizontal velocity of hips just before' ...
	     ' impact to\n'],upper(fcn_name));
fprintf(fid,'%%    state of the system just before impact.\n');
fprintf(fid,'%%\n%%    DQ = %s(DTHETA1)\n\n',upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'[a,b,m] = controlParameters;\n\n');

fprintf(fid,'Ah_inv = zeros(5);\n');
for k=1:5,
   for j=1:5,
      if Ah_inv(k,j)~=0
	ttt=num2str(Ah_inv(k,j));
	fprintf(fid,'Ah_inv(%s,%s)=%s;\n',num2str(k), ...
		num2str(j),ttt);
      end
   end
end
fprintf(fid,['\naN = a([',num2str(1+NN),';', ...
	     num2str(1+2*(NN+1)-1),';', ...
	     num2str(1+3*(NN+1)-1),';', ...
	     num2str(1+4*(NN+1)-1),']);\n']);
fprintf(fid,['aN_minus_1 = a([',num2str(1+NN-1),';', ...
	     num2str(1+2*(NN+1)-2),';', ...
	     num2str(1+3*(NN+1)-2),';', ...
	     num2str(1+4*(NN+1)-2),']);\n\n']);
fprintf(fid,['dq = Ah_inv*[-',num2str(NN),'*(aN_minus_1-aN)/m;' ...
		    ' 1]*dtheta1;']);
fclose(fid);
%
%
%
fcn_name = 'zd_lift';
disp(['[creating ',upper(fcn_name),'.m]']);
fid = fopen([fcn_name,'.m'],'w');
fprintf(fid,'function x = %s(theta1,dtheta1)\n',fcn_name);
fprintf(fid,['%%%s    Lifts the zero dynamics to the full state of' ...
	     ' the system.\n'],upper(fcn_name));
fprintf(fid,'%%\n%%    X = %s(THETA1,DTHETA1)\n\n', ...
	upper(fcn_name));
fprintf(fid,'%%Eric Westervelt\n');
fprintf(fid,'%%%s\n\n',datestr(now));

fprintf(fid,'[a,b,m] = controlParameters;\n\n');

a = char('a');
for j = 0:3
  for k = 0:NN
    fprintf(fid,[char(a+j),num2str(k),'=a(',num2str(1+k+j*(NN+1)),'); ']);
  end
  fprintf(fid,'\n');
end
fprintf(fid,'\n');

ttt = char(vectorize(q_sigma_zd(1)));
fprintf(fid,'q31 = %s;\n\n',fixlength(ttt,'*+-',65,'      '));

ttt = char(vectorize(q_sigma_zd(2)));
fprintf(fid,'q32 = %s;\n\n',fixlength(ttt,'*+-',65,'      '));

ttt = char(vectorize(q_sigma_zd(3)));
fprintf(fid,'q41 = %s;\n\n',fixlength(ttt,'*+-',65,'      '));

ttt = char(vectorize(q_sigma_zd(4)));
fprintf(fid,'q42 = %s;\n\n',fixlength(ttt,'*+-',65,'      '));

ttt = char(vectorize(q_sigma_zd(5)));
fprintf(fid,'q1 = %s;\n\n',fixlength(ttt,'*+-',65,'     '));

ttt = char(vectorize(dq31_sigma));
fprintf(fid,'dq31 = %s;\n\n',fixlength(ttt,'*+-',65,'       '));

ttt = char(vectorize(dq32_sigma));
fprintf(fid,'dq32 = %s;\n\n',fixlength(ttt,'*+-',65,'       '));

ttt = char(vectorize(dq41_sigma));
fprintf(fid,'dq41 = %s;\n\n',fixlength(ttt,'*+-',65,'       '));

ttt = char(vectorize(dq42_sigma));
fprintf(fid,'dq42 = %s;\n\n',fixlength(ttt,'*+-',65,'       '));

ttt = char(vectorize(dq1_sigma));
fprintf(fid,'dq1 = %s;\n\n',fixlength(ttt,'*+-',65,'      '));

fprintf(fid,['x = [q31; q32; q41; q42; q1; dq31; dq32; dq41; dq42;' ...
	     ' dq1];']);
fclose(fid);