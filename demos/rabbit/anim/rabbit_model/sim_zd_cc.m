function varargout = sim_zd_cc(t,x,flag,opt_param)
%SIM_ZD_CC   Simulate the zero dynamics of the kneed biped walker.

%Eric Westervelt
%1/22/01

if nargin == 0
   flag = 'demo';
end

switch flag
case ''                                 % Return dx/dt = dynamics(t,x).
   varargout{1} = f(t,x);
case 'init'                             % Return default [tspan,x0,options].
   [varargout{1:3}] = init;
case 'events'                           % Return [value,isterminal,direction].
   [varargout{1:3}] = events(t,x);
case 'poincare'                         % Return [lam_omega].
   varargout{1} = poincare(opt_param);
case 'demo'                             % Run a demo.
   [varargout{1:2}] = demo;
otherwise
   error(['Unknown flag ''' flag '''.']);
end


% --------------------------------------------------------------------------
% This is the system dynamics function

function dz = f(t,z)

dz = zd_cc(z);

% --------------------------------------------------------------------------
% This is the init function

function [tspan,x0,options] = init

error('Should not use init function without first setting it up.');


% --------------------------------------------------------------------------
% This is the events function

function [value,isterminal,direction] = events(t,z)
% Locate the time when critial angle of stance leg minus stance leg
% angle passes through zero in a decreasing direction and stop
% integration.

x = zd_lift(z(1),z(2))';

% when swing leg foot touches ground
value(1) = swing_foot_height(x);

isterminal(1) = 1;  % stop when this event occurs
direction(1)  = -1; % decreasing direction detection


% --------------------------------------------------------------------------
% This is the Poincare map function

function [lam_omega] = poincare(opt_param)
% Generate the Poincare map for a vector opt_param

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = ...
    modelParameters;

options = odeset('Events','on', ...
		 'Refine',5, ...
		 'RelTol',10^-5, ...
		 'AbsTol',10^-6);

n = length(opt_param);
lam_omega = [];

for k = 1:length(opt_param),
  disp([num2str(k/n*100,'%0.2f'),'% done generating Poincare map']);
  
  x0 = sigma(opt_param(k));
  out = impact_abs_red(x0).';
  x0 = out(1:10);
  
  % project i.c. onto zero dynamics
  z0 = zd_project_cc(x0);
  
  [t,z] = ode45('sim_zd_cc',[0 2],z0,options);
  nt = length(t);
  
  x = zd_lift(z(nt,1),z(nt,2));

  % point invalid if swing foot is behind stance foot at end of "step"
  if step_length(x) <= 0
    disp('Stepping error ... didn''t step properly..');
    lam_omega(k) = 0; % outside region of attraction
  else
    lam_omega(k) = 1/2*(x(6)+x(8));
  end
end


% --------------------------------------------------------------------------
% This is the demo function

function [tout,zout] = demo

if 0
  %THIS CODE HASN'T BEEN USED IN A WHILE...MAKE TAKE SOME DOING TO
  %RESURRECT IT (2/16/01)

  global N % specified in <poincare_zd_compare_cc.m>

  s = load('mat_files/high_gain_data');
  z0 = zd_project_cc(s.x(N,:))';
  
  tstart = 0;
  tfinal = s.t(length(s.t)) - s.t(N);
  num_steps = 1;
  zout = z0';
else
  % initial condtion in full coordinates
  initialize;
  
  % project initial condition onto zero dynamics
  z0 = zd_project_cc(x0);
  num_steps = 2;
  zout = z0;
end
  
tstart = 0;
tfinal = 50;

tout = tstart;

step_t = [];

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = ...
    modelParameters;

options = odeset('Events','on', ...
		 'Refine',4, ...
		 'RelTol',10^-5, ...
		 'AbsTol',10^-6);

for i = 1:num_steps
  % Solve until the first terminal event.
  [t,z] = ode45('sim_zd_cc',[tstart tfinal],z0,options);
  
  nt = length(t);
  
  % Accumulate output.  This could be passed out as output arguments.
  tout = [tout; t(2:nt)];
  zout = [zout; z(2:nt,:)];

  if t(nt) <= tfinal
    % Set the new initial conditions (after impact).
    xf = zd_lift(z(nt,1),z(nt,2));
    x0 = impact_abs_red(xf);
    z0 = zd_project_cc(x0(1:10)');


    % monitor flag from Grizzle's impact function
    if x0(11)
      disp('Stepping error from Grizzle impact function.')
    end
    
    step_len = step_length(xf);
    
    % Diplay some useful info
    im = sprintf('%.5f',x0(12));
    dt = sprintf('%.5f',t(length(t))-t(1));
    sl = sprintf('%.5f',step_len);
    ra = sprintf('%.5f',step_len/(t(length(t))-t(1)));
    disp(['step: ',num2str(i),', impact: ',im, ...
	  ', delta_t: ',dt,', length: ',sl,', rate: ',ra]);
    
    step_t = [step_t t(length(t))];
    
    % Stop simulation if swing foot is behind stance foot at end of
    % "step"
    if step_len <= 0
      disp('Stepping error ... swing foot didn''t swing forward.');
    end
  end
  
  tstart = t(nt);
  STEP_TIME = t(nt);
  if tstart >= tfinal
    break
  end  
end


if 0
  xout = zd_lift(zout(:,1)',zout(:,2)').';
  anim(tout,xout,1/30)
  
  disp('[Done with simulation, about to draw zero dynamics'' plot]');
  
  % use this when you run a lot of steps to see the limit cycle
  % form in the zero dynamics!!
  fig_hl=figure(2);
  clf
  set(fig_hl,'Position',[50 290 400 400]);
  set(fig_hl,'PaperPosition',[1.25 1.5 6 8])
  
  vectfield('zd_cc',2.8:0.05:3.5,-2:.2:-.2,.999);
  hold on
  plot(zout(:,1),zout(:,2));
  xlabel('z_1 \rightarrow \theta_1');
  ylabel('z_2 \rightarrow d\theta_1');
  title('Limit cycle of zero dynamics overlayed on vector field');
  grid;
end