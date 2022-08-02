function varargout = sim_zd(t,x,flag,opt_param)
%SIM_ZD   Simulate the zero dynamics of the kneed biped walker.

%Eric Westervelt
%10/26/00

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
  [varargout{1:5}] = demo;
 otherwise
  error(['Unknown flag ''' flag '''.']);
end


% --------------------------------------------------------------------------
% This is the system dynamics function

function dz = f(t,z)

dz = zd(z);

%[dx,dx_approx] = zd(x);
%dx = dx_approx;


% --------------------------------------------------------------------------
% This is the init function

function [tspan,x0,options] = init

error('Should not use init function without first setting it up.');


% --------------------------------------------------------------------------
% This is the events function

function [value,isterminal,direction] = events(t,z)
%Locate the time when critial angle of stance leg minus stance leg
%angle passes through zero in a decreasing direction and stop
%integration.

global SIM_COUNTER

SIM_COUNTER = SIM_COUNTER + 1;

dz = zd(z);
x = zd_lift(z(1),dz(1)).';

if SIM_COUNTER > 200 & mod(t,.01)
  value(9) = 1;
else
  value(9) = 0;
end

[pHh,pHv]=hip_pos(x);

h = swing_foot_height(x');

value(1) = h;
value(2) = x(5) - pi/6;        % torso not too far backward
value(3) = x(5) + pi/6;        % torso not too far forward
value(4) = x(2) - 7*pi/10;     % swing femur not too far back
value(5) = x(2) - 13*pi/10;    % swing femur not too far forward
value(6) = pHv - .6;           % hips not too low
value(7) = x(1) - x(3) - pi/2; % stance knee not too bent
value(8) = x(2) - x(4) - pi/2; % swing knee not too bent

isterminal(1) = 1;  % stop when this event occurs
isterminal(2) = 1;  % stop  "
isterminal(3) = 1;  % stop  "
isterminal(4) = 1;  % stop  "
isterminal(5) = 1;  % stop  "
isterminal(6) = 1;  % stop  "
isterminal(7) = 1;  % stop  "
isterminal(8) = 1;  % stop  "
isterminal(9) = 1;  % stop  "

direction(1) = -1;  % decreasing direction
direction(2) = 0;  % decreasing  "
direction(3) = 0;  % decreasing  "
direction(4) = 0;  % decreasing  "
direction(5) = 0;  % increasing  "
direction(6) = 0;  % decreasing  "
direction(7) = 0;  % increasing  "
direction(8) = 0;  % increasing  "
direction(9) = 0;  % increasing  "


% --------------------------------------------------------------------------
% This is the Poincare map function

function [lam_omega] = poincare(opt_param)
% Generate the Poincare map for a vector opt_param

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = ...
    modelParameters;

options = odeset('Events','on', ...
		 'Refine',4, ...
         'MaxStep',.008, ...
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
  z0 = zd_project(x0);
  
  [t,z] = ode45('sim_zd',[0 2],z0,options);
  nt = length(t);

  x = zd_lift_special(z(nt,:));

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

function [tout,zout,teout,zeout,ieout] = demo

global SIM_COUNTER
global N % specified in <poincare_zd_compare.m>

SIM_COUNTER = 0;

if ~isempty(N)
  %THIS CODE HASN'T BEEN USED IN A WHILE...MAKE TAKE SOME DOING TO
  %RESURRECT IT (2/16/01)
  
  s = load('mat_files/high_gain_data');
  z0 = zd_project(s.x(N,:))';
  
  tstart = 0;
  tfinal = s.t(length(s.t)) - s.t(N);
  num_steps = 1;
  zout = z0';
else
  % initial condtion in full coordinates
  initialize; 

  % project initial condition onto zero dynamics
  z0 = zd_project(x0);
  
  num_steps = 1;
  zout = z0;
end
  
tstart = 0;
tfinal = 20;

tout = tstart;

step_t = [];

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = ...
    modelParameters;

options = odeset('Events','on', ...
		 'Refine',4, ...
         'MaxStep',.008, ...
		 'RelTol',10^-5, ...
		 'AbsTol',10^-6);

teout = []; zeout = []; ieout = [];

for i = 1:num_steps
  % Solve until the first terminal event.
  [t,z,te,ze,ie] = ode45('sim_zd',[tstart tfinal],z0,options);
  
  nt = length(t);
  
  % Accumulate output.  This could be passed out as output
  % arguments.
  tout = [tout; t(2:nt)];
  zout = [zout; z(2:nt,:)];
  teout = [teout; te];
  zeout = [zeout; ze];
  ieout = [ieout; ie];
  
  if t(nt) <= tfinal & i ~= num_steps
    % Set the new initial conditions (after impact).
    dz = zd(z(nt,:));
    xf = zd_lift(z(nt,1),dz(1));
    x0 = impact_abs_red(xf);
    z0 = zd_project(x0(1:10)');

    % monitor flag from Grizzle's impact function
    if x0(11), disp('stepping error -- IMPACT_ABS_RED.M'); end
    
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
      disp('stepping error - swing foot didn''t swing forward.');
    end
  end
  
  tstart = t(nt);
  STEP_TIME = t(nt);
  if tstart >= tfinal
    break
  end  
end

ieout

if 0
  xout = zd_lift_special(zout);
  anim(tout,xout,1/20)
  
  disp('[Done with simulation, about to draw zero dynamics'' plot]');
  
  % use this when you run a lot of steps to see the limit cycle
  % form in the zero dynamics!!
  fig_hl=figure(2);
  clf
  set(fig_hl,'Position',[100 100 350 700]);
  set(fig_hl,'PaperPosition',[1.25 1.5 6 8])
  
  vectfield('zd',2.8:0.04:3.5,-40:.5:-15,.999);
  hold on
  plot(zout(:,1),zout(:,2));
  xlabel('z_1 \rightarrow q41');
  ylabel('z_2 \rightarrow really messy expression');
  title('Limit cycle of zero dynamics overlayed on vector field');
  grid;
end