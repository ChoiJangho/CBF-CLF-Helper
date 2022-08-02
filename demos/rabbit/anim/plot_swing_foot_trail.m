function plot_swing_foot_trail%(t,x,ts,l_min_t,l_max_t,ground)
%to creat video with animate of MABEL stick code
% savename=['simulation\data\ctrl13_Nsteps10_test_no8_test_file_Aug_10'];
savename=['simulation\Aug_18_data\ctrl13_Nsteps10_test_no55_test_file_Aug_18'];
% load(savename);
load(savename);%,'t_2','x_2','torque','y', 'dy', 'force', 'CLF_V', 'CLF_Vdot', 'mu_added', 'SimConfig','l_min_t','l_max_t','CBF','ground');

% length(x)
ts=1/100;
%Eric Westervelt
%2/15/01
a=0.2;
if nargin < 3, ts = 1/20; end

[n,m] = size(x);
n_full=n;
l_min=zeros(n,1);l_max=zeros(n,1);
for i=1:n
    l_min(i,1)=l_min_t(i,1);l_max(i,1)=l_max_t(i,1);
end
pH_horiz = zeros(n,1);

if n == 1 | m == 1
  te = t;
  xe = x;
  q=xe(1:5);
  n = 1;
else
  % Estimate hip horizontal position by estimating integral of hip
  % velocity
  vH = hip_vel(x); % convert angles to horizontal hosition of hips
  for j=2:n
    pH_horiz(j)=pH_horiz(j-1)+(t(j)-t(j-1))*vH(j-1,1);
  end
  pH_horiz_full=pH_horiz;
   [te,l_min]=even_sample(t,l_min,1/ts);
  [te,l_max]=even_sample(t,l_max,1/ts);
  [te,pH_horiz]=even_sample(t,pH_horiz,1/ts);
  [te,xe]=even_sample(t,x,1/ts);
  [n,m]=size(xe);
  q=xe(1,1:5);
end

%save as a video
% % spwriter = VideoWriter('biped_walking_obscure.avi');
% % set(spwriter, 'FrameRate', 0.3/ts,'Quality',100);
% % open(spwriter);

k=0;

out = limb_position(q,pH_horiz(1));

fig1 = figure(1);

set(0,'Units','pixels')
scnsize = get(0,'ScreenSize');

screen_width = scnsize(3);
screen_height = scnsize(4);

%figure_x_limits = [-1 2];%[-1.5 4];
%figure_y_limits = [-0.5 1];%figure_y_limits = [-.5 2.8];
figure_x_limits = [-1.5 6];%[-1.5 4];
figure_y_limits = [-1 2.3];%
% find the minimum scaling factor

figure_x_size = figure_x_limits(2) - figure_x_limits(1);
figure_y_size = figure_y_limits(2) - figure_y_limits(1);

xfactor = screen_width/figure_x_size;
yfactor = screen_height/figure_y_size;

if (xfactor < yfactor)
  screen_factor = 0.5*xfactor;
else
  screen_factor = 0.5*yfactor;
end

% calculate screen offsets
screen_x_offset = (screen_width - screen_factor*figure_x_size)/2;
screen_y_offset = (screen_height - screen_factor*figure_y_size)/2;

% draw figure and axes
set(fig1,'Position', [screen_x_offset screen_y_offset screen_factor*figure_x_size screen_factor*figure_y_size]);
% set(fig1,'MenuBar', 'none');
axes1 = axes;
set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
set(axes1,'Position',[0 0 1 1]);
set(axes1,'Color','w');
set(axes1,'TickDir','out');

% draw first ground
lws=2;
line([out.pFoot21-0.2 out.pFoot11+0.2],[out.pFoot12 out.pFoot12],'LineWidth',lws,'Color','k');hold on;
line([out.pFoot11+0.2 out.pFoot11+0.2],[out.pFoot12 out.pFoot12-0.2],'LineWidth',lws,'Color','k');hold on;
% draw pre-located stones
[Nsteps,mg]=size(ground);
ls=0;
%gray=[0.5 0.5 0.5];
gray='k';%[0.7 0.7 0.7];
for i=1:Nsteps
    ls_min=ls+ground(i,1);
    ls_max=ls_min+0.05;
%     line([out.pFoot11+ls_min out.pFoot11+ls_max],[out.pFoot12 out.pFoot12],'LineWidth',0.5,'Color',gray,'LineStyle',':');hold on;
%     line([out.pFoot11+ls_min out.pFoot11+ls_min],[out.pFoot12 out.pFoot12-a],'LineWidth',0.5,'Color',gray,'LineStyle',':');hold on;
%     line([out.pFoot11+ls_max out.pFoot11+ls_max],[out.pFoot12 out.pFoot12-a],'LineWidth',0.5,'Color',gray,'LineStyle',':');hold on;
lw=2;    
line([out.pFoot11+ls_min out.pFoot11+ls_max],[out.pFoot12 out.pFoot12],'LineWidth',lw,'Color',gray);hold on;
    line([out.pFoot11+ls_min out.pFoot11+ls_min],[out.pFoot12 out.pFoot12-a],'LineWidth',lw,'Color',gray);hold on;
    line([out.pFoot11+ls_max out.pFoot11+ls_max],[out.pFoot12 out.pFoot12-a],'LineWidth',lw,'Color',gray);hold on;
ls=ls+ground(i,2);
end

drawone(out,1);

% Draw ground
buffer=5;
% ground=line([-buffer pH_horiz(n)+buffer],[-a -a]);
% set(ground,'Color',ground_color,'LineWidth',2);
% % for k=-buffer:floor(pH_horiz(n)+buffer)
% %   ref_tick(k+buffer+1)=line([k k],[-0.07-a 0-a]);
% %   set(ref_tick(k+buffer+1),'Color',ground_color);
% %   ref_label(k+buffer+1)=text(-0.03+k,-0.18-a,num2str(k));
% %   set(ref_label(k+buffer+1),'FontSize',15)
% % end

% Display mass
xt=(figure_x_limits(1)+figure_x_limits(2))/2+out.pH1+0.5;
yt=figure_y_limits(2)-0.5;
disp_mass=text(xt,yt,['m_{load}=',num2str(SimConfig.m_load),' [kg]']);
   set(disp_mass,'FontSize',15);
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%55
leg=1;
for k = 1:length(t);%n
  if n == 1 | m == 1, q=x(1:5); else, q=x(k,1:5); end

  out = limb_position(q,pH_horiz_full(k));

  if k>1
      q0=x(k-1,1:5);
      out0= limb_position(q0,pH_horiz_full(k-1));
      if abs(out.pFoot21-out0.pFoot21)>0.15
          leg=rem(leg+1,2);
%           drawone(out,leg);
      end
  end
  % swing leg
     a=[out.pFoot21;out.pFoot22];
     if leg
     plot(a(1),a(2),'r.','linewidth',1);
     else
         plot(a(1),a(2),'g.','linewidth',1);
     end

end
end

function drawone(out,leg)
    % Use actual relations between masses in animation

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=modelParameters;
scl=0.022; % factor to scale masses
mr_knees=M4^(1/3)*scl;  % radius of mass for knees
mr_femurs=M3^(1/3)*scl; % radius of mass for femurs
mr_torso=M1^(1/3)*scl;  % radius of mass for torso
if leg
leg1_color='g';
leg2_color='r';
else
    leg1_color='r';
leg2_color='g';
end
torso_color='b';
ground_color='k'; % a.k.a. black

% Approximate circular shape of mass
param=linspace(0,2*pi+2*pi/50,50);
xmass_knees=mr_knees*cos(param);
ymass_knees=mr_knees*sin(param);

xmass_femurs=mr_femurs*cos(param);
ymass_femurs=mr_femurs*sin(param);

xmass_torso=mr_torso*cos(param);
ymass_torso=mr_torso*sin(param);
% Draw stance leg
tibia1=pt_link2([out.pFoot11 ;out.pFoot12],[out.pG11 ;out.pG12],leg1_color,leg1_color);
femur1=pt_link2([out.pG11 ;out.pG12],[out.pH1; out.pH2],leg1_color,leg1_color);


% Draw swing leg
tibia2=pt_link2([out.pFoot21 ;out.pFoot22],[out.pG21; out.pG22],leg2_color,leg2_color);
femur2=pt_link2([out.pG21; out.pG22],[out.pH1 ;out.pH2],leg2_color,leg2_color);

% Draw torso
torso=pt_link2([out.pH1; out.pH2],[out.pHead1 ;out.pHead2],torso_color,torso_color);

% Draw joints
jointG1=patch(xmass_knees+out.pG11,ymass_knees+out.pG12, ...
		  'k');
jointG2=patch(xmass_knees+out.pG21,ymass_knees+out.pG22, ...
		  'k');
jointH=patch(xmass_knees+out.pH1,ymass_knees+out.pH2, ...
		  'k');
      % Draw mass
      global SimConfig
      scale=SimConfig.m_load/12+1; 
      rm=sqrt(scale-1)/10;
mass=pt_circle([(out.pH1+out.pHead1)/2;(out.pH2+out.pHead2)/2],rm,'y');


end

function [obj]=pt_circle(pt0,rad,color)
  i=0:0.1:2*pi;

  x=rad.*cos(i)+pt0(1);

  y=rad.*sin(i)+pt0(2);

  obj=patch(x,y,color);

  set(obj,'EdgeColor','none');
end