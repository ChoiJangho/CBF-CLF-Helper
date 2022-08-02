function f = bezier_obj_jwg(param)
%BEZIER_OBJ   Function to minimize while performing open-loop
%    optimization.
%
%    [F] = BEZIER_OBJ(PARAM) cost associated with having
%    state trajectories associated with PARAM.

%Eric Westervelt
%2/1/01

global C WHICH_ONE FILENAME
persistent counter iter_count

if isempty(counter), counter = 0; end
if isempty(iter_count), iter_count = 0; end

iter_count = iter_count + 1;
counter = counter + 1;
counter_max = 20;
if counter > counter_max
    counter = counter - counter_max;
end

data = load(['mat_files/',FILENAME]);

[te,xe] = even_sample(data.t,data.x,1000);

te = te - te(1);

th1 = 1/2*(xe(:,1)+xe(:,3));
dth1=1/2*(xe(:,6)+xe(:,8));

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

nt = length(traj);

u = 1/C(1)*(th1-C(2));  
m=C(1);

fit = u*0;

param_tmp = [traj(1) traj(1)+dtraj(1)*m/(6*dth1(1)) param traj(nt)-dtraj(nt)*m/(6*dth1(nt)) traj(nt)];

N = length(param_tmp);
NN = N - 1;
for k = 1:N
    kk = k - 1;
    fit = fit + param_tmp(k)'.*factorial(NN)/ ...
        factorial(kk)/ ...
        factorial(NN-kk)*u.^(kk) ...
        .*(1-u).^(NN-kk);
end

f = norm(traj - fit);

if counter == 1
    u = u*te(nt);
    u_prime = linspace(0,1,N)*te(nt);
    
    subplot(2,2,WHICH_ONE)
    plot(u,fit,u,traj,u_prime,param_tmp,'o')
    line(u_prime(1:2),param_tmp(1:2),'Color','r')
    line(u_prime(NN:N),param_tmp(NN:N),'Color','r')
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
    xlabel('t (sec)');
    drawnow
end