function anim_flat_ground(t,x,ts)
%ANIM   Animate kneed biped walker.  Note:  (1) TS is an options
%       parameters and (2) T may be a scalar.

%Eric Westervelt
%2/15/01

if nargin < 3, ts = 1/20; end

[n,m] = size(x);
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
  
  [te,pH_horiz]=even_sample(t,pH_horiz,1/ts);
  [te,xe]=even_sample(t,x,1/ts);
  [n,m]=size(xe);
  q=xe(1,1:5);
end

k=0;

out = limb_position(q,pH_horiz(1));

fig_hl=figure(1);
%set(fig_hl,'Position', [440 700 250 250]); % extra small
%set(fig_hl,'Position', [440 500 250 250]); % for laptop
set(fig_hl,'Position', [20 80 250 250]); % for serefe
%set(fig_hl,'Position', [550 660 400 400]); % for making movies
set(fig_hl,'PaperPosition',[0 0 5 5]);     % for making movies
clf
%anim_axis=[-.8 .8 -1.5 1.4]; % for falling animation
anim_axis=[-.8 .8 -0.1 1.4];
axis off
axis(anim_axis)
axis equal
grid

% Use actual relations between masses in animation

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=modelParameters;
scl=0.022; % factor to scale masses
mr_knees=M4^(1/3)*scl;  % radius of mass for knees
mr_femurs=M3^(1/3)*scl; % radius of mass for femurs
mr_torso=M1^(1/3)*scl;  % radius of mass for torso
leg1_color='g';
leg2_color='r';
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

% Draw ground
buffer=5;
ground=line([-buffer pH_horiz(n)+buffer],[0 0]);
set(ground,'Color',ground_color,'LineWidth',2);
for k=-buffer:floor(pH_horiz(n)+buffer)
  ref_tick(k+buffer+1)=line([k k],[-0.07 0]);
  set(ref_tick(k+buffer+1),'Color',ground_color);
  ref_label(k+buffer+1)=text(-0.03+k,-0.18,num2str(k));
  set(ref_label(k+buffer+1),'FontSize',15)
end

% Draw stance leg
tibia1=line([out.pFoot11 out.pG11],[out.pFoot12 out.pG12]);
tibia_mass1=patch(xmass_knees+out.pTib11,ymass_knees+out.pTib12, ...
		  leg1_color);
femur1=line([out.pG11 out.pH1],[out.pG12 out.pH2]);
femur_mass1=patch(xmass_femurs+out.pFem11,ymass_femurs+out.pFem12, ...
		  leg1_color);
set(tibia_mass1,'EdgeColor',leg1_color)
set(tibia1,'LineWidth',2,'Color',leg1_color);
set(femur_mass1,'EdgeColor',leg1_color)
set(femur1,'LineWidth',2,'Color',leg1_color);

% Draw swing leg
tibia2=line([out.pFoot21 out.pG21],[out.pFoot22 out.pG22]);
tibia_mass2=patch(xmass_knees+out.pTib21,ymass_knees+out.pTib22, ...
		  leg2_color);
femur2=line([out.pG21 out.pH1],[out.pG22 out.pH2]);
femur_mass2=patch(xmass_femurs+out.pFem21,ymass_femurs+out.pFem22, ...
		  leg2_color);
set(tibia_mass2,'EdgeColor',leg2_color)
set(tibia2,'LineWidth',2,'Color',leg2_color);
set(femur_mass2,'EdgeColor',leg2_color)
set(femur2,'LineWidth',2,'Color',leg2_color);

% Draw torso
torso=line([out.pH1 out.pHead1],[out.pH2 out.pHead2]);
torso_mass=patch(xmass_torso+out.pT1,ymass_torso+out.pT2, ...
		 torso_color);
set(torso_mass,'EdgeColor',torso_color)
set(torso,'LineWidth',2,'Color',torso_color);

for k = 1:n
  if n == 1 | m == 1, q=xe(1:5); else, q=xe(k,1:5); end

  out = limb_position(q,pH_horiz(k));
  
  % stance leg
  set(tibia1,'XData',[out.pFoot11 out.pG11],...
	     'YData',[out.pFoot12 out.pG12]);
  set(tibia_mass1,'XData',xmass_knees+out.pTib11,...
		  'YData',ymass_knees+out.pTib12);
  set(femur1,'XData',[out.pG11 out.pH1],...
	     'YData',[out.pG12 out.pH2]);
  set(femur_mass1,'XData',xmass_femurs+out.pFem11,...
		  'YData',ymass_femurs+out.pFem12);
  
  % swing leg
  set(tibia2,'XData',[out.pFoot21 out.pG21],...
	    'YData',[out.pFoot22 out.pG22]);
  set(tibia_mass2,'XData',xmass_knees+out.pTib21,...
		  'YData',ymass_knees+out.pTib22);
  set(femur2,'XData',[out.pG21 out.pH1],...
	     'YData',[out.pG22 out.pH2]);
  set(femur_mass2,'XData',xmass_femurs+out.pFem21,...
		  'YData',ymass_femurs+out.pFem22);
  
  % torso
  set(torso,'XData',[out.pH1 out.pHead1],...
	    'YData',[out.pH2 out.pHead2]);
  set(torso_mass,'XData',xmass_torso+out.pT1,...
		 'YData',ymass_torso+out.pT2); 
  
  if n ~= 1 & m ~= 1
    hdl=title(['T_{est} = ',num2str(te(k),'%.2f')]);
    set(hdl,'FontSize',15);
  end
  
  new_axis=anim_axis+[out.pH1 out.pH1 0 0];
  axis(new_axis);
  
  for j = 1:length(ref_label)
    if (j-buffer-1<new_axis(1)) | (j-buffer-1>new_axis(2))
      set(ref_label(j),'Visible','off')
      set(ref_tick(j),'Visible','off')
    else
      set(ref_label(j),'Visible','on');
      set(ref_tick(j),'Visible','on')
    end
  end
  
  drawnow;
  
  % for making movies
  %print('-dppmraw',['frame.',sprintf('%d',k-1),'.ppm'])
end