function [com_pos  com_vel  com_accel] = COM_pos_vel_accel(q, dq, ddq)
    [g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]= modelParameters;

    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5) ;
    dq1 = dq(1); dq2 = dq(2); dq3 = dq(3); dq4 = dq(4); dq5 = dq(5) ;
    ddq1 = ddq(1); ddq2 = ddq(2); ddq3 = ddq(3); ddq4 = ddq(4); ddq5 = ddq(5) ;

    com_pos = [(MY1*cos(q5) - MZ3*sin(q1) - MZ3*sin(q2) - MZ4*sin(q3) - MZ4*sin(q4) - MZ1*sin(q5) + L3*M1*sin(q1) + 2*L3*M3*sin(q1) + L3*M4*sin(q1) + L4*M1*sin(q3) - L3*M4*sin(q2) + 2*L4*M3*sin(q3) + 2*L4*M4*sin(q3))/(M1 + 2*M3 + 2*M4) ;
               (MZ3*cos(q1) + MZ3*cos(q2) + MZ4*cos(q3) + MZ4*cos(q4) + MZ1*cos(q5) + MY1*sin(q5) - L3*M1*cos(q1) - 2*L3*M3*cos(q1) - L3*M4*cos(q1) - L4*M1*cos(q3) + L3*M4*cos(q2) - 2*L4*M3*cos(q3) - 2*L4*M4*cos(q3))/(M1 + 2*M3 + 2*M4) ;
              ] ;

    com_vel = [(dq1*(L3*M1*cos(q1) - MZ3*cos(q1) + 2*L3*M3*cos(q1) + L3*M4*cos(q1)))/(M1 + 2*M3 + 2*M4) + (dq3*(L4*M1*cos(q3) - MZ4*cos(q3) + 2*L4*M3*cos(q3) + 2*L4*M4*cos(q3)))/(M1 + 2*M3 + 2*M4) - (dq5*(MZ1*cos(q5) + MY1*sin(q5)))/(M1 + 2*M3 + 2*M4) - (dq2*(MZ3*cos(q2) + L3*M4*cos(q2)))/(M1 + 2*M3 + 2*M4) - (MZ4*dq4*cos(q4))/(M1 + 2*M3 + 2*M4) ;
               (dq5*(MY1*cos(q5) - MZ1*sin(q5)))/(M1 + 2*M3 + 2*M4) + (dq1*(L3*M1*sin(q1) - MZ3*sin(q1) + 2*L3*M3*sin(q1) + L3*M4*sin(q1)))/(M1 + 2*M3 + 2*M4) + (dq3*(L4*M1*sin(q3) - MZ4*sin(q3) + 2*L4*M3*sin(q3) + 2*L4*M4*sin(q3)))/(M1 + 2*M3 + 2*M4) - (dq2*(MZ3*sin(q2) + L3*M4*sin(q2)))/(M1 + 2*M3 + 2*M4) - (MZ4*dq4*sin(q4))/(M1 + 2*M3 + 2*M4) ;
              ] ;

    com_accel = [(dq2^2*(MZ3*sin(q2) + L3*M4*sin(q2)))/(M1 + 2*M3 + 2*M4) - (dq3^2*(L4*M1*sin(q3) - MZ4*sin(q3) + 2*L4*M3*sin(q3) + 2*L4*M4*sin(q3)))/(M1 + 2*M3 + 2*M4) - (dq1^2*(L3*M1*sin(q1) - MZ3*sin(q1) + 2*L3*M3*sin(q1) + L3*M4*sin(q1)))/(M1 + 2*M3 + 2*M4) + (ddq1*(L3*M1*cos(q1) - MZ3*cos(q1) + 2*L3*M3*cos(q1) + L3*M4*cos(q1)))/(M1 + 2*M3 + 2*M4) + (ddq3*(L4*M1*cos(q3) - MZ4*cos(q3) + 2*L4*M3*cos(q3) + 2*L4*M4*cos(q3)))/(M1 + 2*M3 + 2*M4) - (ddq5*(MZ1*cos(q5) + MY1*sin(q5)))/(M1 + 2*M3 + 2*M4) - (ddq2*(MZ3*cos(q2) + L3*M4*cos(q2)))/(M1 + 2*M3 + 2*M4) - (dq5^2*(MY1*cos(q5) - MZ1*sin(q5)))/(M1 + 2*M3 + 2*M4) - (MZ4*ddq4*cos(q4))/(M1 + 2*M3 + 2*M4) + (MZ4*dq4^2*sin(q4))/(M1 + 2*M3 + 2*M4) ;
                 (ddq5*(MY1*cos(q5) - MZ1*sin(q5)))/(M1 + 2*M3 + 2*M4) - (dq2^2*(MZ3*cos(q2) + L3*M4*cos(q2)))/(M1 + 2*M3 + 2*M4) + (ddq1*(L3*M1*sin(q1) - MZ3*sin(q1) + 2*L3*M3*sin(q1) + L3*M4*sin(q1)))/(M1 + 2*M3 + 2*M4) + (ddq3*(L4*M1*sin(q3) - MZ4*sin(q3) + 2*L4*M3*sin(q3) + 2*L4*M4*sin(q3)))/(M1 + 2*M3 + 2*M4) + (dq1^2*(L3*M1*cos(q1) - MZ3*cos(q1) + 2*L3*M3*cos(q1) + L3*M4*cos(q1)))/(M1 + 2*M3 + 2*M4) + (dq3^2*(L4*M1*cos(q3) - MZ4*cos(q3) + 2*L4*M3*cos(q3) + 2*L4*M4*cos(q3)))/(M1 + 2*M3 + 2*M4) - (ddq2*(MZ3*sin(q2) + L3*M4*sin(q2)))/(M1 + 2*M3 + 2*M4) - (dq5^2*(MZ1*cos(q5) + MY1*sin(q5)))/(M1 + 2*M3 + 2*M4) - (MZ4*ddq4*sin(q4))/(M1 + 2*M3 + 2*M4) - (MZ4*dq4^2*cos(q4))/(M1 + 2*M3 + 2*M4) ;
              ] ;
end