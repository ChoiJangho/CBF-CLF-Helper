function [] = make_plots_quan_save_figs(t,torque,y,x,V,l_min_t,l_max_t,CBF)

close all;
% figure; plot(t,torque'); title('Intputs') ; grid ; xlabel('Time (s)') ;
%figure(3) ; plot(t, y') ; title('Outputs') ; grid ; xlabel('Time (s)') ;
[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = modelParameters;
q1=x(:,1);q2=x(:,2);q3=x(:,3);q4=x(:,4);q5=x(:,5);
dq1=x(:,6);dq2=x(:,7);dq3=x(:,8);dq4=x(:,9);dq5=x(:,10);
figure; plot(t,V','linewidth',2);
% ylabel('CLF');
% legend('CLF','Location','northwest');
xlabel('Time (s)');
h=legend('$V(x)$');
set(h,'Interpreter','latex');

fig = gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 5 2];
savename='simulation\Aug_18_data\CLF';
saveas(gcf,savename,'jpg');



lf=L3*sin(q1)+L4*sin(q3)-L3*sin(q2)-L4*sin(q4);
hf=-L3*cos(q1)-L4*cos(q3)+L3*cos(q2)+L4*cos(q4);
global SimConfig 
controller=SimConfig.controller;
l_min=controller.CBF_l_min;
l_max=controller.CBF_l_max;
R1=controller.CBF_R1;
R2=controller.CBF_R2;

%CBF=[gb;CBF1;CBF2;CBF1_dot;CBF2_dot;CBF1_condition;CBF2_condition]; 
gb1=CBF(:,1);gb2=CBF(:,2);
h1=CBF(:,3);h2=CBF(:,4);
%CBF1=CBF(:,3);CBF2=CBF(:,4);CBF1_condition=CBF(:,7);CBF2_condition=CBF(:,8);
figure; f=plot(t,lf,t,l_min_t',':',t,l_max_t','--');%title('Step Length');
set(f(1),'linewidth',2.5);
set(f(2),'linewidth',1.5);
set(f(3),'linewidth',1.5);
h=legend('$l_f$','$l_{min}$','$l_{max}$');
set(h,'Interpreter','latex');
set(h,'Location','northoutside','Orientation','horizontal');
xlabel('Time (s)');
% ylabel('Step Length');
axis('tight');

fig = gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 4.5 2.5];
savename='simulation\Aug_18_data\step_length';
saveas(gcf,savename,'jpg');

% % figure(6);
% % subplot(3,1,1);plot(t,V);title('CLF');
% % subplot(3,1,2);plot(t,CBF1_condition);title('CBF condition 1');
% % subplot(3,1,3);plot(t, CBF2_condition);title('CBF condition 2');
%figure(7); plot(t,h1,t,h2);title('upper and lower constraints');

%figure(6); plot(t,ls_b-ls);title('leg to stone distance');
figure; f=plot(t,h1,t,h2);
set(f(1),'linewidth',2.5,'linestyle',':');
set(f(2),'linewidth',1.5);
h=legend('$h_1(x)$','$h_2(x)$');
set(h,'Interpreter','latex');
set(h,'Location','northoutside','Orientation','horizontal');
% set(h,'Location','northwest');
xlabel('Time (s)');
% ylabel('Stepping Stone Constraints');
axis('tight');
fig = gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 4.5 2.5];
savename='simulation\Aug_18_data\stepping_stone_constraints';
saveas(gcf,savename,'jpg');
