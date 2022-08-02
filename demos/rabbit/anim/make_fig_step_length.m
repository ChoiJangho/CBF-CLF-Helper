function [] = make_fig_step_length(t,x,l_min_t,l_max_t)

% close all;

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = modelParameters;
q1=x(:,1);q2=x(:,2);q3=x(:,3);q4=x(:,4);

lf=L3*sin(q1)+L4*sin(q3)-L3*sin(q2)-L4*sin(q4);

global SimConfig 
controller=SimConfig.controller;
l_min=controller.CBF_l_min;
l_max=controller.CBF_l_max;

figure(1); 
h=plot(t,lf, t,l_min_t',t,l_max_t');
set(h(1),'linewidth',2);
% title('Step Length');
xlabel('Time (s)');
ylabel('[m]');
legend('l_s','l_{min}','l_{max}','Location','northoutside','Orientation','horizontal');
axis tight;

savename='figures\step_length';

pos=[0 0 9.69/2 2.5];
set(gcf, 'PaperPosition', pos); 
set(gcf, 'PaperSize', pos(3:4));
saveas(gcf, savename, 'pdf')

