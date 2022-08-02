function make_fig_stick_moving_stone_load(t,x,l_min_t,l_max_t,N_stones)

ts=1/10;

[n,m] = size(x);
n_full=n;
l_min=l_min_t(1:n);
l_max=l_max_t(1:n);

% Estimate hip horizontal position by estimating integral of hip
% velocity
pH_horiz = zeros(n,1);
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

fig1 = figure(1);

axes1 = axes;
% set(axes,'Color','w');
% set(axes,'TickDir','out');
axis equal;
axis off;
axis tight;


% draw first robot
drawone(axes1, xe, pH_horiz, l_min, l_max, 1);

% draw moving stones
draw_ground(xe, pH_horiz, l_min, l_max, N_stones);
hold on;

% save fig
savename='figures\stick_fig';

pos=[0 0 9.69/2 2]*2;
set(gcf, 'PaperPosition', pos); 
set(gcf, 'PaperSize', pos(3:4));
saveas(gcf, savename, 'pdf')
% saveas(gcf, savename, 'png')
% print('-f1','-depsc','-painters',savename);
end

function drawone(parent, xe, pH_horiz, l_min, l_max, k)
% delete previous screen
% tem = get(parent,'Children');
% delete(tem);
    
leg1_color='g';
leg2_color='r';
torso_color='b'; 

[n,m]=size(xe);

out = limb_position(xe(k,1:5),pH_horiz(k));

%draw stones 
%  draw_ground(xe, pH_horiz, l_min, l_max, k);

% stance leg
    % tibia1: Foot 1 to G1
     a=[out.pFoot11;out.pFoot12];b=[out.pG11;out.pG12];
     pt_link2(a,b,leg1_color,leg1_color);

    % femur 1: H to G1
    a=[out.pG11;out.pG12];b=[out.pH1;out.pH2];
    pt_link2(a,b,leg1_color,leg1_color);
  
% swing leg
    % tibia2: Foot 2 to G2
     a=[out.pFoot21;out.pFoot22];b=[out.pG21;out.pG22];
     pt_link2(a,b,leg2_color,leg2_color);

    % femur 2: H to G2
    a=[out.pG21;out.pG22];b=[out.pH1;out.pH2];
    pt_link2(a,b,leg2_color,leg2_color);


% torso: Head to H
    a=[out.pH1;out.pH2];b=[out.pHead1;out.pHead2];
    pt_link2(a,b,torso_color,torso_color);

 
% joints
    radius=0.028; color='k';
    % knee 1
    x_center=out.pG11; y_center=out.pG12;
    draw_circle(x_center,y_center,radius,color);

    % knee 2
    x_center=out.pG21; y_center=out.pG22; 
    draw_circle(x_center,y_center,radius,color);

    % hip
    x_center=out.pH1; y_center=out.pH2;
    draw_circle(x_center,y_center,radius,color);

    % Draw load

    global SimConfig
    m_load=SimConfig.m_load;
    rm=sqrt(m_load/12)/10; 

    pt_circle([(out.pH1+out.pHead1)/2;(out.pH2+out.pHead2)/2],rm,'y');

%     % Display load
%     figure_x_limits = [-1.5 4];%[-1.5 4];
%     figure_y_limits = [-1 2.3];%
%     
%     xt=(figure_x_limits(1)+figure_x_limits(2))/2+out.pH1+0.5;
%     yt=figure_y_limits(2)-0.5;
%     disp_mass=text(xt,yt,['Load=',num2str(m_load),' [kg]']);
%     set(disp_mass,'FontSize',15);
end

function draw_ground(xe, pH_horiz, l_min, l_max,N_stones)
[n,m]=size(xe);

lws=2;
% draw the first ground
out = limb_position(xe(1,1:5),pH_horiz(1));
line([out.pFoot21-0.2 out.pFoot11+0.15],[out.pFoot12 out.pFoot12],'LineWidth',lws,'Color','k');hold on;
line([out.pFoot11+0.15 out.pFoot11+0.15],[out.pFoot12 out.pFoot12-0.2],'LineWidth',lws,'Color','k');hold on;
  

%% determine step count
step_count=1;
step_start_index(1)=1;
for j=1:n
    out=limb_position(xe(j,1:5),pH_horiz(j));
    if j<n
    out2=limb_position(xe(j+1,1:5),pH_horiz(j+1));
    else
        out2=limb_position(xe(j,1:5),pH_horiz(j));
    end
    % check step count
    if (out.pFoot21>out.pFoot11)&&(out2.pFoot21<out2.pFoot11)
        step_count=step_count+1;
        step_start_index(step_count)=j;
        step_end_index(step_count-1)=j;
    end
end
step_end_index(step_count)=n;

%% plot stone for all steps
for i=1:N_stones
    km=step_start_index(i)+1;
    k=step_end_index(i);
    for j=km:k
        color1=0.7*[1 1 1]; % gray
%         color1=[0 0 1]; % blue
        color2=[1 1 1];% white
        if km==k
            color='w';
        elseif j==k
            color='k'; % make current stone position black
        else
        color=color1*(j-km)/(k-km)+color2*(k-j)/(k-km); % dim out past stone position
        end

        out = limb_position(xe(j,1:5),pH_horiz(j));
        line([out.pFoot11+l_min(j,1) out.pFoot11+l_max(j,1)],[out.pFoot12 out.pFoot12],'LineWidth',lws,'Color',color);hold on;
        line([out.pFoot11+l_min(j,1) out.pFoot11+l_min(j,1)],[out.pFoot12 out.pFoot12-0.2],'LineWidth',lws,'Color',color);hold on;
        line([out.pFoot11+l_max(j,1) out.pFoot11+l_max(j,1)],[out.pFoot12 out.pFoot12-0.2],'LineWidth',lws,'Color',color);hold on;
    end
end

end

function [obj]=pt_link2(a,b,maincolor,edgecolor)
R = inline('[cos(t) -sin(t); sin(t) cos(t)]');
d = [1/20; 0];
phi = pi/8;

c = b-a;
if c(1) > 0
	theta = atan(c(2)/c(1));
elseif c(1) < 0
	theta = atan(c(2)/c(1))+pi; 
else
	theta = atan2(c(2),c(1));
end
poly = [a, R(phi+theta)*d+a, b-R(-phi+theta)*d, b, b-R(phi+theta)*d, R(-phi+theta)*d+a];
obj=patch(poly(1,:), poly(2,:),maincolor);
set(obj,'EdgeColor',edgecolor);
end

function obj=draw_circle(x_center,y_center,radius,color)
param=linspace(0,2*pi+2*pi/50,50);
x=radius*cos(param);
y=radius*sin(param);
obj=patch(x+x_center,y+y_center,color);
end

function [obj]=pt_circle(pt0,rad,color)
  i=0:0.1:2*pi;

  x=rad.*cos(i)+pt0(1);

  y=rad.*sin(i)+pt0(2);

  obj=patch(x,y,color);

  set(obj,'EdgeColor','none');
end