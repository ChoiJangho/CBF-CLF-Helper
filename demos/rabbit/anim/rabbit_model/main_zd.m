%MAIN_ZD.M
%
%   Run simulation of walker with offset torso.

%Eric Westervelt
%3/12/01

global RUN_NAME

RUN_NAME = 't';

disp(['RUN_NAME = ',RUN_NAME]);

res = [];

if ~isempty(dir([RUN_NAME,'_zd_data.mat']))
  while isempty(res)
    res = input('Do you want to over run this trial? (0/1)');
  end
  if res == 0, return, end
end

T = 3;

initialize; 

z0 = zd_project(x0); % project initial condition onto zero dynamics

sim('zd_model',T);

x = zd_lift_special(z);

save(['mat_files/',RUN_NAME,'_data'],'t','z','sim_error', ...
     'foot_touch','T');

anim(t,x,1/30);