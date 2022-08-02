function plot_contact_force(t,x,u)
global m_scale_factor j_scale_factor SimConfig
scale=SimConfig.m_load/SimConfig.m_torso+1; m_scale_factor=scale; j_scale_factor=scale;
N=length(t);
contact_force=zeros(N,2);
% m_robot=1;%plot accel_com only%32+m_load;
m_robot=32+SimConfig.m_load;
g=9.81;
for i=1:N
q=x(i,1:5)';dq=x(i,6:10)';
[D,C,G,B]=dyn_mod_abs_red(x(i,:));
ddq = inv(D)*[-C*dq-G+B*u(i,:)'];
[com_pos , com_vel , com_accel] = COM_pos_vel_accel(q, dq, ddq);
contact_force(i,:)=m_robot*([0;g]+com_accel)';%m_robot*(g+com_accel(2));
end

figure(10);
plot(t,contact_force(:,2),'b','linewidth',2);
ylabel('N (N)');
xlabel('Time(s)');
% fig = gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 5 2];
% savename='simulation\Aug_18_data\contact_force';
% saveas(gcf,savename,'jpg');

figure(11);
mu=contact_force(:,1)./contact_force(:,2);
plot(t,abs(mu),'b','linewidth',2);
ylabel('|F/N|');
xlabel('Time(s)');
% fig = gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 5 2];
% savename='simulation\Aug_18_data\friction';
% saveas(gcf,savename,'jpg');

end



