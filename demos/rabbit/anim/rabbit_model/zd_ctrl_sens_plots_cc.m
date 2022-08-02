function zd_ctrl_sens_plots_cc
%ZD_CTRL_SENS_PLOTS_CC

%Eric Westervelt
%1/23/01

global z OUT X_MIN X_MAX 
global DIFF_VECT_FIELD_SCL DIFF_VECT_FIELD_SCL_TORSO
global OMEGA_MIN

OMEGA_MIN=-1.5;

DIFF_VECT_FIELD_SCL = 5;
DIFF_VECT_FIELD_SCL_TORSO = 1;

X_MIN = 2.85;
X_MAX = 3.42;

WIDTH = 800;
HEIGHT = 800;

fig_hl = figure(2);
clf
subplot(5,5,1);
set(fig_hl,'Position', [100 50 WIDTH HEIGHT]);
set(fig_hl,'PaperPosition', [.25 .25 8 10.5]);

[t,z] = sim_zd_cc;

% display stats
x = zd_lift(z(1,1),z(1,2));
sl = step_length(x);
[tmp,hh] = hip_pos(x);

vectfield('zd_cc', ...
	  linspace(X_MIN,X_MAX,10), ...
	  linspace(OMEGA_MIN,0,10),1);
axis([X_MIN X_MAX OMEGA_MIN 0]);
hold on
h=plot(z(:,1),z(:,2),'r');
set(h,'linewidth',1.5)
ylabel('z2 \rightarrow 1/2(dq_{31}+dq_{41})');
title('zero dynamics vector field');
drawnow;

[OUT,nominal] = start_of_step_ctrl_sens(2); % specify METHOD here

uicontrol('Position',[0 0 700 25], ...
	  'HorizontalAlignment','left', ...
	  'Style','text', ...
	  'String',['step length = ', num2str(abs(sl)), ...
		    ', hip height = ', num2str(abs(hh)), ...
		    ', diff vect. field scl = ', ...
		    num2str(DIFF_VECT_FIELD_SCL), ...
		    ', diff vect. field scl_torso = ', ...
		    num2str(DIFF_VECT_FIELD_SCL_TORSO), ...
		    ', nominal = ', ...
		    num2str(nominal)]);


plot_stuff(1,'diff vector field a_0');
plot_stuff(2,'diff vector field a_1');
plot_stuff(3,'diff vector field a_2');
plot_stuff(4,'diff vector field a_3');
plot_stuff(5,'diff vector field a_4');
plot_stuff(6,'diff vector field a_5');

plot_stuff(7,'diff vector field b_0');
plot_stuff(8,'diff vector field b_1');
plot_stuff(9,'diff vector field b_2');
plot_stuff(10,'diff vector field b_3');
plot_stuff(11,'diff vector field b_4');
plot_stuff(12,'diff vector field b_5');

plot_stuff(13,'diff vector field c_0');
plot_stuff(14,'diff vector field c_1');
plot_stuff(15,'diff vector field c_2');
plot_stuff(16,'diff vector field c_3');
plot_stuff(17,'diff vector field c_4');
plot_stuff(18,'diff vector field c_5');

plot_stuff(19,'diff vector field d_0');
plot_stuff(20,'diff vector field d_1');
plot_stuff(21,'diff vector field d_2');
plot_stuff(22,'diff vector field d_3');
plot_stuff(23,'diff vector field d_4');
plot_stuff(24,'diff vector field d_5');

print plots/ctrl_sens.ps


%----------------------------------------------
function plot_stuff(ctrl_param_indx,title_string)

global z OUT X_MIN X_MAX 
global DIFF_VECT_FIELD_SCL DIFF_VECT_FIELD_SCL_TORSO
global WHICH_TO_TWEAK OMEGA_MIN

figure(2)

subplot(5,5,ctrl_param_indx+1);
if ctrl_param_indx < 7
  vectfield('zd_diff_cc', ...
	    linspace(X_MIN,X_MAX,10), linspace(OMEGA_MIN,0,10), ...
	    DIFF_VECT_FIELD_SCL_TORSO,ctrl_param_indx);
else
  vectfield('zd_diff_cc', ...
	    linspace(X_MIN,X_MAX,10), linspace(OMEGA_MIN,0,10), ...
	    DIFF_VECT_FIELD_SCL,ctrl_param_indx);
end  
axis([X_MIN X_MAX OMEGA_MIN 0]);

if ctrl_param_indx >= 20
  xlabel('z1 \rightarrow 1/2(q_{31}+q_{41})');
end
if mod(ctrl_param_indx,5) == 0
  ylabel('z2 \rightarrow 1/2(dq_{31}+dq_{41})');
end
title(title_string);
grid; drawnow;

hold on
h=plot(z(:,1),z(:,2),'r');
set(h,'linewidth',1.5)
text(X_MIN*1.01,-.2,num2str(OUT(ctrl_param_indx)));