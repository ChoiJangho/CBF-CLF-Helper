function bezier_opt_jwg(filename)


if ~isunix
addpath C:\d_drive\mat_docs\robot_dir\Robot_Genoux\Westervelt_simulator_5_1_01
addpath C:\d_drive\mat_docs\robot_dir\Robot_Genoux\Westervelt_simulator_5_1_01\mat_files
rep='C:\d_drive\mat_docs\robot_dir\Robot_Genoux\Westervelt_simulator_5_1_01\mat_files\';
else
addpath /win/d_drive/mat_docs/robot_dir/Robot_Genoux/Westervelt_simulator_5_1_01
addpath /win/d_drive/mat_docs/robot_dir/Robot_Genoux/Westervelt_simulator_5_1_01/mat_files
rep='/win/d_drive/mat_docs/robot_dir/Robot_Genoux/Westervelt_simulator_5_1_01/mat_files/';
end    


%BEZIER_OPT
%
%   Generate initial polynomial fit of state trajectories before performing
%   open loop optimization.
%
%Eric Westervelt
%2/1/01
%5/1/01 - made into a function

global C WHICH_ONE FILENAME

FILENAME = filename;

data = load(['mat_files/',FILENAME]); 

[te,xe] = even_sample(data.t,data.x,1000);

te = te - te(1);

th1 = 1/2*(xe(:,1)+xe(:,3));
dth1=1/2*(xe(:,6)+xe(:,8));

nt = length(th1); 

q31 = xe(:,1);
q32 = xe(:,2);
q41 = xe(:,3);
q42 = xe(:,4);
q1 = xe(:,5);
dq31 = xe(:,6);
dq32 = xe(:,7);
dq41 = xe(:,8);
dq42 = xe(:,9);
dq1 = xe(:,10);

tt = linspace(0,1,2);
phi = [tt' ones(length(tt),1)];
C = inv(phi'*phi)*phi'*[th1(1); th1(nt)];  m=C(1); b=C(2);
u = 1/C(1)*(th1-C(2));
%plot(th1,u)

figure(1); clf

for WHICH_ONE = 1:4
  switch WHICH_ONE
   case 1
    traj = q1-q31;
    dtraj=dq1-dq31;
   case 2
    traj = q1-q32;
    dtraj=dq1-dq32;
   case 3
    traj = q41-q31;
    dtraj=dq41-dq31;
   case 4
    traj = q42-q32;
    dtraj=dq42-dq32;
  end

  % change the number of parameters in this vector to change the
  % number of optimiztion parameters (the total is length(a)+2) -- 7
  % seems to be best to fit to old trajectories
  a = [traj(1), traj(1), traj(nt)];
  %a = [traj(1), traj(1), traj(1), traj(nt), traj(nt)];
  %a = [traj(1), traj(1), traj(nt) traj(nt)]; % 6 params
  %a = [traj(1), traj(1), traj(nt)]; % 5 params

  options = optimset('Display', 'iter',...
		     'TolX', 1e-15,...
		     'TolFun', 1e-6,...
		     'TolCon', 1e-8,...
		     'MaxFunEvals', 1e3,...
		     'MaxIter', 1e2);
  
  [a_final_tmp,f] = fminunc('bezier_obj_jwg', a, options);
  
  % impose symmetry
  switch WHICH_ONE
   case 0
    a_final(WHICH_ONE,:) = [traj(1) traj(1)+dtraj(1)*m/(6*dth1(1)) a_final_tmp traj(1)-dtraj(1)*m/(6*dth1(nt)) traj(1)]; % changed by grizzle to remove symmetry on torso
   case {1,2,3,4}
    a_final(WHICH_ONE,:) = [traj(1) traj(1)+dtraj(1)*m/(6*dth1(1)) a_final_tmp traj(nt)-dtraj(nt)*m/(6*dth1(nt)) traj(nt)]; 
  end
  
  %whos a_final
  
  fit = u*0;
  
  N = length(a_final(WHICH_ONE,:));
  NN = N - 1;
  for k = 1:N
    kk = k - 1;
    fit = fit + a_final(WHICH_ONE,k)'.*factorial(NN)/ ...
	  factorial(kk)/ ...
	  factorial(NN-kk)*u.^(kk) ...
	  .*(1-u).^(NN-kk);
  end

  th1_prime = C(1)*linspace(0,1,N)+C(2);

  if 1
    subplot(2,2,WHICH_ONE)
    plot(th1,traj,th1,fit,th1_prime,a_final(WHICH_ONE,:),'o')
    line(th1_prime(1:2),a_final(WHICH_ONE,1:2),'Color','r')
    line(th1_prime(NN:N),a_final(WHICH_ONE,NN:N),'Color','r')
    switch WHICH_ONE
     case 1
      title('Torso to femur_1 angle (done)')
      ylabel('q_1-q_{31}');
     case 2
      title('Torso to femur2 (done)')
      ylabel('q_1-q_{32}');
     case 3
      title('Stance knee deflection (done)')
      ylabel('q_{41}-q_{31}');
     case 4
      title('Swing knee deflection (done)')
      ylabel('q_{42}-q_{32}');
    end
    xlabel('\theta_1 (rad)');
    drawnow;
  end
end

disp(' ');
disp('[copy this stuff to CONTROLPARAMETERS.M]');
disp(' ');
disp(['  m = ',num2str(C(1)),';']);
disp(['  b = ',num2str(C(2)),';']);
disp(' ');

a = char('a');
for WHICH_ONE = 1:4
  % print out in a format to be copied to CONTROLPARAMETERS.M
  %
  b = [char(a+WHICH_ONE-1) char(a+WHICH_ONE-1)];
  disp(['  ',b,' = [',num2str(a_final(WHICH_ONE,1)),'; ', ...
	num2str(a_final(WHICH_ONE,2)),'; ', ...
	num2str(a_final(WHICH_ONE,3)),'; ', ...
	num2str(a_final(WHICH_ONE,4)),'; ', ...
	num2str(a_final(WHICH_ONE,5)),'; ', ...
	num2str(a_final(WHICH_ONE,6)),'; ', ...
	num2str(a_final(WHICH_ONE,7)),'];']);
end