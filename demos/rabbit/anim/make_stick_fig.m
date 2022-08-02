function make_stick_fig(t,x,l_min_t,l_max_t,ground,savename)
%to creat video with animate of MABEL stick code
% savename=['simulation\data\ctrl13_Nsteps10_test_no8_test_file_Aug_10'];
% savename=['simulation\Aug_18_data\ctrl13_Nsteps10_test_no55_test_file_Aug_18'];
% load(savename);

% length(x)
ts=1/7;
%Eric Westervelt
%2/15/01
a=0.2;


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

axes1 = axes;
axis equal;
axis off;
axis tight;
% draw first ground
lws=1;
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
lw=1;    
line([out.pFoot11+ls_min out.pFoot11+ls_max],[out.pFoot12 out.pFoot12],'LineWidth',lw,'Color',gray);hold on;
    line([out.pFoot11+ls_min out.pFoot11+ls_min],[out.pFoot12 out.pFoot12-a],'LineWidth',lw,'Color',gray);hold on;
    line([out.pFoot11+ls_max out.pFoot11+ls_max],[out.pFoot12 out.pFoot12-a],'LineWidth',lw,'Color',gray);hold on;
ls=ls+ground(i,2);
end

drawone(out,1);

leg=1;
for k = 1:length(te);%n
  if n == 1 | m == 1, q=xe(1:5); else, q=xe(k,1:5); end

  out = limb_position(q,pH_horiz(k));
drawone(out,leg);
  if k>1
      q0=xe(k-1,1:5);
      out0= limb_position(q0,pH_horiz(k-1));
%       if abs(out.pFoot11-out0.pFoot11)<0.1
%           leg=leg;
%       else
%           leg=rem(leg+1,2);
% %           drawone(out,leg);
%       end
  end
end


pos=[0 0 9.69/2 1.5]*1;
set(gcf, 'PaperPosition', pos); 
set(gcf, 'PaperSize', pos(3:4));
saveas(gcf, savename, 'pdf')
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