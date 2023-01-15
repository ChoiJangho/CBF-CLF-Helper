function draw_rabbit(fig, x, draw_simple, transparency)
if nargin < 3
    draw_simple = false;
end
if nargin < 4
    transparency = 0.0;
end

if ndims(x) >= 2
    if size(x, 1) ~= 14 && size(x, 2) ~=1
        error("This function only accept 1D array.")
    end
end

% ours to quans
qrel = x(3:7);
dqrel = x(10:14);
T = [-1   -1   0  0  0;
     -1    0   0 -1  0;
     -1   -1  -1  0  0;
	 -1    0   0 -1 -1;
     -1    0   0  0  0];
 
q = T*qrel + 2*pi*[ones(4,1);0];
dq = T*dqrel;
% condensed transformed coordinate of x
x_ct = [q; dq];
% position of the hip (horizontal)
pH_horiz = x(1);
% velicoty of the hip
[vH_horiz, vH_vertical] = hip_vel(x_ct');
out = limb_position(q',pH_horiz);
out.vH_horiz = vH_horiz;
out.vH_vertical = vH_vertical;
set(fig,'color','w');
%anim_axis=[-.8 .8 -1.5 1.4]; % for falling animation
anim_axis=[-.8 .8 -0.1 1.4];
axis off
axis(anim_axis)
axis equal
grid

robot_color = get_robot_color();

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=modelParameters;
scl=0.022; % factor to scale masses
mr_knees=M4^(1/3)*scl;  % radius of mass for knees
mr_femurs=M3^(1/3)*scl; % radius of mass for femurs
mr_torso=M1^(1/3)*scl;  % radius of mass for torso
mr_torso = 1.4 * mr_torso;

% Approximate circular shape of mass
param=linspace(0,2*pi+2*pi/50,50);
out.xmass_knees=mr_knees*cos(param);
out.ymass_knees=mr_knees*sin(param);

out.xmass_femurs=mr_femurs*cos(param);
out.ymass_femurs=mr_femurs*sin(param);

out.xmass_torso=mr_torso*cos(param);
out.ymass_torso=mr_torso*sin(param);

draw_stance_leg(out, robot_color, draw_simple, transparency);
draw_swing_leg(out, robot_color, draw_simple, transparency);
draw_torso(out, robot_color, draw_simple, transparency);
end

function draw_stance_leg(out, robot_color, draw_simple, t)
white = [1, 1, 1];
tibia1=line([out.pFoot11 out.pG11],[out.pFoot12 out.pG12], 'LineWidth', 5);
femur1=line([out.pG11 out.pH1],[out.pG12 out.pH2], 'LineWidth', 5);
if draw_simple
    tibia_mass1=patch(out.xmass_knees+out.pTib11,out.ymass_knees+out.pTib12, ...
		  robot_color.leg1_joint_color, 'FaceAlpha', 1-t, 'EdgeColor', 'none');    
else    
    tibia_mass1=patch(out.xmass_knees+out.pTib11,out.ymass_knees+out.pTib12, ...
		  (1-t) * robot_color.leg1_joint_color + t * white, 'EdgeColor', ...
          (1-t) * robot_color.leg1_joint_edge_color + t * white, 'LineWidth', 1.5);
end
% set(tibia1,'LineWidth',5,'Color',(1-t) * robot_color.leg1_color + t * white);
set(tibia1,'LineWidth',10,'Color', [robot_color.leg1_color, 1-t]);
 
% set(femur1,'LineWidth',5,'Color',(1-t) * robot_color.leg1_color + t * white);
set(femur1,'LineWidth',10,'Color', [robot_color.leg1_color, 1-t]);
end

function draw_swing_leg(out, robot_color, draw_simple, t)
white = [1, 1, 1];

tibia2=line([out.pFoot21 out.pG21],[out.pFoot22 out.pG22], 'LineWidth', 5);
femur2=line([out.pG21 out.pH1],[out.pG22 out.pH2], 'LineWidth', 5);
if draw_simple
    tibia_mass2=patch(out.xmass_knees+out.pTib21,out.ymass_knees+out.pTib22, ...
		  robot_color.leg2_joint_color, 'FaceAlpha', 1-t, 'EdgeColor', 'none');
else
    tibia_mass2=patch(out.xmass_knees+out.pTib21,out.ymass_knees+out.pTib22, ...
		  (1-t) * robot_color.leg2_joint_color + t * white, 'EdgeColor', ...
          (1-t) * robot_color.leg2_joint_edge_color + t * white, 'LineWidth', 1.5);
end  

% set(tibia2,'LineWidth',5,'Color', (1-t) * robot_color.leg2_color + t * white);
set(tibia2,'LineWidth',10,'Color', [robot_color.leg2_color, 1-t]);
% set(femur2,'LineWidth',5,'Color', (1-t) * robot_color.leg2_color + t * white);
set(femur2,'LineWidth',10,'Color', [robot_color.leg2_color, 1-t]);

end

function draw_torso(out, robot_color, draw_simple, t)
white = [1, 1, 1];

torso=line([out.pH1 out.pHead1],[out.pH2 out.pHead2]);
% if draw_simple
%     torso_mass=patch(out.xmass_torso+out.pT1,out.ymass_torso+out.pT2, ...
% 		  (1-t) * robot_color.torso_mass_color + t * white, 'EdgeColor', 'none');
% %     torso_mass=patch(out.xmass_torso+out.pT1,out.ymass_torso+out.pT2, ...
% % 		  robot_color.torso_mass_color, 'FaceAlpha', 1-t, 'EdgeColor', 'none');
% 
% else
%     torso_mass=patch(out.xmass_torso+out.pT1,out.ymass_torso+out.pT2, ...
% 		  (1-t) * robot_color.torso_mass_color + t * white, 'EdgeColor', ...
%           (1-t) * robot_color.torso_mass_edge_color + t * white, 'LineWidth', 1.5);
% end

% set(torso,'LineWidth',7,'Color',(1-t) * robot_color.torso_color + t * white);
set(torso,'LineWidth',14,'Color',[robot_color.torso_color, 1-t]);

if draw_simple
    femur_mass2=patch(out.xmass_femurs+out.pFem21,out.ymass_femurs+out.pFem22, ...
		  robot_color.torso_mass_color, 'FaceAlpha', 1-t, 'EdgeColor', 'none');
else
    femur_mass2=patch(out.xmass_femurs+out.pFem21,out.ymass_femurs+out.pFem22, ...
		  (1-t) * robot_color.torso_mass_color + t * white, 'EdgeColor', ...
          (1-t) * robot_color.torso_mass_edge_color + t * white, 'LineWidth', 1.5);
end
hold on;

v_arrow = plot_arrow(out.pH1, out.pH2, out.pH1+0.25 * out.vH_horiz, out.pH2+0.25 * out.vH_vertical, ...
    'LineWidth', 5 * sqrt(out.vH_horiz^2 + out.vH_vertical^2), 'Color', [robot_color.leg1_color, 1-t], 'EdgeColor', 'none', ...
    'FaceColor', robot_color.leg1_color, 'FaceAlpha', 1-t, 'headheight', 0.15, 'headwidth', 0.2);
end