function [QL,QR,PL,PR] = LyapRiccatiGenerator
% This function uses optimization to generate Q and P matrices for Riccati (QR and PR) and
% Lyapunov (QL and PL) equations, such that each method will yield the same
% nominal bound for our CLF V. There are two problem types: in the first
% one, we don't fix anything a priori, and we simultaneously search for the
% Lyapunov coefficients (kp and kd to generate A, and QL) and the Riccati
% coefficients (QR). In the second problem type, we fix a particular
% Lyapunov solution and search for a QR which will yield a Riccati solution
% with the same nominal bound for the CLF V
%
% WARNING: There are some hacks here; several places where you have to
% enter the same data in a couple places

close all
clear all
clc

% Problem type:
%  1-> Optimize to find A, QL,QR such that the Riccati and Lyapunov solutions for P result in the correct ratios of constants
%  2-> Set A and QL, and then optimize to find QR

problem_type = 2;

switch problem_type
    
    case 1
        
        nonzero_Q_corners = false; %HAVE TO SET THIS IN THE CONSTRAINT FUNCTION AS WELL
        
        kp_0 = .5;
        kd_0 = 2;
        qL_0 = 1;
        qR_0 = 1;
        if nonzero_Q_corners
            qcL_0 = 0;
            qcR_0 = 0;
        end
        
        x0(1) = kp_0; x0(2) = kd_0; x0(3) = qL_0; x0(4) = qR_0;
        if nonzero_Q_corners
            x0(5) = qcL_0; x0(6) = qcR_0;
        end
        
        options=optimset('Algorithm','active-set','Diagnostics', 'on', 'Display', 'iter');
        [x,fval,exitflag] = fmincon(@myobj,x0,[],[],[],[],[],[],@mycon1,options)
        
        %Check out the results
        kp = x(1)
        kd = x(2)
        qL = x(3)
        qR = x(4)
        if nonzero_Q_corners
            qcL = x(5)
            qcR = x(6)
        end
        
        A = [zeros(4) eye(4);
            -kp*eye(4)    -kd*eye(4)] ;
        
        QL = qL*eye(8) ;
        if nonzero_Q_corners
            QL(1,8) = qcL; QL(8,1) = qcL;
        end
        PL = lyap(A', QL) ;
        c1L = min(eig(PL));
        c2L = max(eig(PL));
        c12L = c2L/c1L;
        c3L  = min(eig(QL))/max(eig(PL));
        
        F_plain = [zeros(4), eye(4);
            zeros(4), zeros(4)];
        G_plain = [zeros(4);
            eye(4)];
        QR= qR*eye(8) ;
        if nonzero_Q_corners
            QR(1,8) = qcR; QR(8,1) = qcR;
        end
        PR = care(F_plain,G_plain,QR);
        c1R = min(eig(PR));
        c2R = max(eig(PR));
        c12R = c2R/c1R;
        c3R  = min(eig(QR))/max(eig(PR));
        
    case 2
                nonzero_Q_corners = true; %HAVE TO SET THIS IN THE CONSTRAINT FUNCTION AS WELL
        
        %Set these for a particular Lyapunov result you want to match
        %%%%Bad code; you have to copy this down to mycon2 because it won't
        %%%%pass it and its not global
        kp = 1;
        kd = 1.8;
        qL = 1;
        qcL = 0;
        
        % Initial values for optimization
        qR_up_0 = 1; %First four diagonal elements of QR
        qR_down_0 = .5; %Last four diagonal elements of QR
        if nonzero_Q_corners
            qcR_0 = 0; %Corner elements of QR
            %qc2R_0 = 0; %Corner elements of QR
            %qc3R_0 = 0; %Corner elements of QR
        end
        
        
        %Set up for optimization
        x0(1) = qR_up_0;
        x0(2) = qR_down_0;
        if nonzero_Q_corners
            x0(3) = qcR_0; 
            %x0(4) = qc2R_0;
            %x0(5) = qc3R_0;
        end
        
        options=optimset('Algorithm','active-set','Diagnostics', 'on', 'Display', 'iter');
        [x,fval,exitflag] = fmincon(@myobj,x0,[],[],[],[],[],[],@mycon2,options)
        
        %Check out the results
        qR_up = x(1)
        qR_down = x(2)
        if nonzero_Q_corners
            qcR = x(3)
            %qc2R = x(4) 
            %qc3R = x(5)
        end
        
        A = [zeros(4) eye(4);
            -kp*eye(4)    -kd*eye(4)] ;
        
        QL = qL*eye(8) ;
        QL(1,8) = qcL; QL(8,1) = qcL;
        PL = lyap(A', QL) ;
        c1L = min(eig(PL));
        c2L = max(eig(PL));
        c12L = c2L/c1L;
        c3L  = min(eig(QL))/max(eig(PL));
        
        F_plain = [zeros(4), eye(4);
            zeros(4), zeros(4)];
        G_plain = [zeros(4);
            eye(4)];
        QR= [qR_up*eye(4) zeros(4) ; zeros(4) qR_down*eye(4)];
        if nonzero_Q_corners
            QR(1,8) = qcR; QR(8,1) = qcR;
            %QR(1,7) = qc2R; QR(7,1) = qc2R;
            %QR(2,8) = qc3R; QR(8,2) = qc3R;
        end
        PR = care(F_plain,G_plain,QR);
        c1R = min(eig(PR));
        c2R = max(eig(PR));
        c12R = c2R/c1R;
        c3R  = min(eig(QR))/max(eig(PR));
end

end

function f = myobj(x)
f = 1;
end

function [c, ceq] = mycon1(x)

nonzero_Q_corners = false;

kp = x(1); kd = x(2); qL = x(3); qR = x(4);
if nonzero_Q_corners
    qcL= x(5); qcR = x(6);
end

A = [zeros(4) eye(4);
    -kp*eye(4)    -kd*eye(4)] ;

QL = qL*eye(8) ;
if nonzero_Q_corners
    QL(1,8) = qcL; QL(8,1) = qcL;
end
PL = lyap(A', QL) ;
c1L = min(eig(PL));
c2L = max(eig(PL));
c12L = c2L/c1L;
c3L  = min(eig(QL))/max(eig(PL));

F_plain = [zeros(4), eye(4);
    zeros(4), zeros(4)];
G_plain = [zeros(4);
    eye(4)];
QR= qR*eye(8) ;
if nonzero_Q_corners
    QR(1,8) = qcR; QR(8,1) = qcR;
end
PR = care(F_plain,G_plain,QR);
c1R = min(eig(PR));
c2R = max(eig(PR));
c12R = c2R/c1R;
c3R  = min(eig(QR))/max(eig(PR));

ceq(1) = c12L-c12R; %ratio of max to min eigenvalues of each P should be the same
ceq(2) = c3L-c3R; %same gamma for each one

%Ensure positive definite QL and QR and Hurwitz A
min_reQL = min(real(eig(QL)));
c(1) = -min_reQL + .0001;
min_reQR = min(real(eig(QR)));
c(2) = -min_reQR + .0001;
max_reA = max(real(eig(A)));
c(3) = max_reA + .0001;

end

function [c, ceq] = mycon2(x)

        %Set these for a particular Lyapunov result you want to match
        kp = 1;
        kd = 1.8;
        qL = 1;
        qcL = 0;

nonzero_Q_corners = true;



        qR_up = x(1);
        qR_down = x(2);
        if nonzero_Q_corners
            qcR = x(3);
            %qc2R = x(4);
            %qc3R = x(5);
        end
        
        A = [zeros(4) eye(4);
            -kp*eye(4)    -kd*eye(4)] ;
        
        QL = qL*eye(8) ;
        QL(1,8) = qcL; QL(8,1) = qcL;
        PL = lyap(A', QL) ;
        c1L = min(eig(PL));
        c2L = max(eig(PL));
        c12L = c2L/c1L;
        c3L  = min(eig(QL))/max(eig(PL));
        
        F_plain = [zeros(4), eye(4);
            zeros(4), zeros(4)];
        G_plain = [zeros(4);
            eye(4)];
        QR= [qR_up*eye(4) zeros(4) ; zeros(4) qR_down*eye(4)];
        if nonzero_Q_corners
            QR(1,8) = qcR; QR(8,1) = qcR;
           % QR(1,7) = qc2R; QR(7,1) = qc2R;
           % QR(2,8) = qc3R; QR(8,2) = qc3R;
        end
PR = care(F_plain,G_plain,QR);
c1R = min(eig(PR));
c2R = max(eig(PR));
c12R = c2R/c1R;
c3R  = min(eig(QR))/max(eig(PR));

ceq(1) = c12L-c12R; %ratio of max to min eigenvalues of each P should be the same
ceq(2) = c3L-c3R; %same gamma for each one

%Ensure positive definite QL and QR and Hurwitz A
min_reQL = min(real(eig(QL)));
c(1) = -min_reQL + .0001;
min_reQR = min(real(eig(QR)));
c(2) = -min_reQR + .0001;
max_reA = max(real(eig(A)));
c(3) = max_reA + .0001;

end