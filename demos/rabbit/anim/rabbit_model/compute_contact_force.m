function contact_force=compute_contact_force(x,u)
global m_scale_factor j_scale_factor SimConfig
scale=SimConfig.m_load/SimConfig.m_torso+1; m_scale_factor=scale; j_scale_factor=scale;

contact_force=zeros(2,1);
m_robot=1;%plot accel_com only%32+m_load;
g=9.81;

q=x(1:5);dq=x(6:10);
[D,C,G,B]=dyn_mod_abs_red(x);
ddq = inv(D)*[-C*dq-G+B*u];
[com_pos , com_vel , com_accel] = COM_pos_vel_accel(q, dq, ddq);
contact_force=m_robot*([0;g]+com_accel)';%m_robot*(g+com_accel(2));
end



