function [] = make_plots(t,x,t2,u,y,dy,f,V,Vdot,v,step_t,doplots,doprints,print_type)
%MAKE_PLOTS    Make plots of walking.
%
%   See also WALK.

%Eric Westervelt
%8/21/00

close all %KSG [01MAY12]

if nargin < 14
    print_type = 'ps';
end

if nargin < 13
    doprints = 0;
end

if nargin < 12
    doplots = ones(9,1); doplots(end)=0;
end

if length(doplots) < 6
    error('Need to specify whether to plot EACH plot.')
end

figno = 2;
plot_num = 0;

offset = [50 50 0 0];
screen_position = [30 100 300 400];
paper_position = [1.25 1.5 6 8];

plot_num = plot_num + 1;
if doplots(plot_num)
    fig_hl=figure(figno); figno = figno + 1;
    set(fig_hl,'Position', screen_position);
    screen_position = screen_position + offset;
    set(fig_hl,'PaperPosition',paper_position)
    
    subplot(2,1,1)
    plot(t2,v)
    legend('v_{31}','v_{32}','v_{41}','v_{42}',-1)
    title('Added control effort v')
    xlabel('time (sec)')
    grid
    axis([min(t2) max(t2) min(min(v))*1.1 max(max(v))*1.1])
    subplot(2,1,2)
    u_star = u-v;
    plot(t2,u_star)
    legend('u^{*}_{31}','u^{*}_{32}','u^{*}_{41}','u^{*}_{42}',-1)
    title('u^{*} feedforward control')
    xlabel('time (sec)')
    grid
    axis([min(t2) max(t2) min(min(u_star))*1.1 max(max(u_star))*1.1])
    if doprints
        name = 'v_and_u_star';
        if strcmp(print_type,'ps')
            print('-dps',[name,'.ps'])
        else
            print('-djpeg',[name,'.jpg']);
        end
    end
end

plot_num = plot_num + 1;
if doplots(plot_num)
    fig_hl=figure(figno); figno = figno + 1;
    set(fig_hl,'Position', screen_position);
    screen_position = screen_position + offset;
    set(fig_hl,'PaperPosition',paper_position)
    
%     subplot(2,1,1)
%     plot(t2,V)
%     legend('V_{\epsilon}')
%     title('Control Lyapunov Function V_{\epsilon}')
%     xlabel('time (sec)')
%     grid
%     axis([min(t2) max(t2) min(min(V))*1.1 max(max(V))*1.1])
%     subplot(2,1,2)
%     plot(t2,Vdot)
%     legend('dV_{\epsilon}')
%     title('Time Derivative of Control Lyapunov Function V_{\epsilon}')
%     xlabel('time (sec)')
%     grid
%     axis([min(t2) max(t2) min(min(Vdot))*1.1 max(max(Vdot))*1.1])
    
    subplot(2,1,1)
    V_to_plot = V/V(1); %Either plot V or plot V/V(1)
    plot(t2,V_to_plot)
    legend('dV_{\epsilon}')
    title('Control Lyapunov Function V_{\epsilon} normalized by initial value')
    xlabel('time (sec)')
    grid
    axis([min(t2) max(t2) min(min(V_to_plot))*1.1 max(max(V_to_plot))*1.1])
    subplot(2,1,2)
    Vdot_to_plot = Vdot/abs(Vdot(1));
    plot(t2,Vdot_to_plot)
    legend('dV_{\epsilon}')
    title('Time Derivative of Control Lyapunov Function V_{\epsilon} normalized by initial value')
    xlabel('time (sec)')
    grid
    axis([min(t2) max(t2) min(min(Vdot_to_plot))*1.1 max(max(Vdot_to_plot))*1.1])
    if doprints
        name = 'CLF';
        if strcmp(print_type,'ps')
            print('-dps',[name,'.ps'])
        else
            print('-djpeg',[name,'.jpg']);
        end
    end
end

% plot_num = plot_num + 1;
% if doplots(plot_num)
%     figure
%     plot(0:.01:10,sin(0:.01:10))
% end

plot_num = plot_num + 1;
if doplots(plot_num)
    fig_hl=figure(figno); figno = figno + 1;
    set(fig_hl,'Position', screen_position);
    screen_position = screen_position + offset;
    set(fig_hl,'PaperPosition',paper_position)
    
    subplot(3,1,1)
    plot(t,x(:,1),'-',t,x(:,2),'--')
    legend('q_{31}','q_{32}',-1)
    grid
    xlabel('time (sec)')
    axis([min(t) max(t) min(min(x(:,1:2)))*.99 max(max(x(:,1:2)))*1.01])
    subplot(3,1,2)
    plot(t,x(:,3),'-',t,x(:,4),'--',t,x(:,5),'-.')
    legend('q_{41}','q_{42}',-1)
    xlabel('time (sec)')
    grid
    axis([min(t) max(t) min(min(x(:,3:4)))*.99 max(max(x(:,3:4)))*1.01])
    subplot(3,1,3)
    plot(t,x(:,5))
    legend('q_1',-1)
    xlabel('time (sec)')
    grid
    %   whos
    %   [min(t) max(t) min(x(:,5))*1.1 max(x(:,5))*.9]
    %   axis([min(t) max(t) min(x(:,5))*1.1 max(x(:,5))*.9])
    axis([min(t) max(t) min(x(:,5)) max(x(:,5))])
    
    if doprints
        name = 'positions';
        if strcmp(print_type,'ps')
            print('-dps',[name,'.ps']);
        else
            print('-djpeg',[name,'.jpg']);
        end
    end
end

plot_num = plot_num + 1;
if doplots(plot_num)
    fig_hl=figure(figno); figno = figno + 1;
    set(fig_hl,'Position', screen_position);
    screen_position = screen_position + offset;
    set(fig_hl,'PaperPosition',paper_position)
    
    subplot(3,1,1)
    plot(t,x(:,6),'-',t,x(:,7),'--')
    legend('q_{31} (dot)','q_{32} (dot)',-1)
    grid
    xlabel('time (sec)')
    axis([min(t) max(t) min(min(x(:,6:7)))*1.1 max(max(x(:,6:7)))*1.1])
    subplot(3,1,2)
    plot(t,x(:,8),'-',t,x(:,9),'--')
    legend('q_{41} (dot)','q_{42} (dot)',-1)
    grid
    xlabel('time (sec)')
    axis([min(t) max(t) min(min(x(:,8:9)))*1.1 max(max(x(:,8:9)))*1.1])
    subplot(3,1,3)
    plot(t,x(:,10))
    legend('q_1 (dot)',-1)
    grid
    xlabel('time (sec)')
    axis([min(t) max(t) min(x(:,10))*1.1 max(x(:,10))*1.1])
    if doprints
        name = 'velocity';
        if strcmp(print_type,'ps')
            print('-dps',[name,'.ps'])
        else
            print('-djpeg',[name,'.jpg']);
        end
    end
end

plot_num = plot_num + 1;
if doplots(plot_num)
    fig_hl=figure(figno); figno = figno + 1;
    set(fig_hl,'Position', screen_position);
    screen_position = screen_position + offset;
    set(fig_hl,'PaperPosition',paper_position)
    
    subplot(2,1,1)
    plot(t2,u(:,1),'-',t2,u(:,2),'--')
    legend('\tau_{31}','\tau_{32}',-1)
    title('Hip controls')
    xlabel('time (sec)')
    grid
    axis([min(t2) max(t2) min(min(u(:,1:2)))*1.1 max(max(u(:,1:2)))*1.1])
    subplot(2,1,2)
    plot(t2,u(:,3),'-',t2,u(:,4),'--')
    legend('\tau_{41}','\tau_{42}',-1)
    title('Knee controls')
    xlabel('time (sec)')
    grid
    axis([min(t2) max(t2) min(min(u(:,3:4)))*1.1 max(max(u(:,3:4)))*1.1])
    if doprints
        name = 'control';
        if strcmp(print_type,'ps')
            print('-dps',[name,'.ps'])
        else
            print('-djpeg',[name,'.jpg']);
        end
    end
end

plot_num = plot_num + 1;
if doplots(plot_num)
    fig_hl=figure(figno); figno = figno + 1;
    set(fig_hl,'Position', screen_position);
    screen_position = screen_position + offset;
    set(fig_hl,'PaperPosition',paper_position)
    
    nn = 4; mm = 2; %KSG [: Originally this had mm=1. Setting it to mm=2 causes it to plot the output derivatives
    
    % positions
    subplot(nn,mm,1)
    plot(t2,y(:,1))
    title('Outputs')
    ylabel('y_1 (q1-q1d)')
    xlabel('time (sec)')
    grid
    if 0
        axis([min(t2) max(t2) -1e-4 1e-4]);
    else
        axis([min(t2) max(t2) min(min(y(:,1)))*1.1 max(max(y(:,1)))*1.1]);
    end
    subplot(nn,mm,mm+1)
    plot(t2,y(:,2))
    ylabel('y_2 (d1+d2)')
    xlabel('time (sec)')
    grid
    if 0
        axis([min(t2) max(t2) -1e-4 1e-4]);
    else
        axis([min(t2) max(t2) min(min(y(:,2)))*1.1 max(max(y(:,2)))*1.1]);
    end
    subplot(nn,mm,2*mm+1)
    plot(t2,y(:,3))
    ylabel('stance leg knee')
    xlabel('time (sec)')
    grid
    if 0
        axis([min(t2) max(t2) -1e-4 1e-4]);
    else
        axis([min(t2) max(t2) min(min(y(:,3))) max(max(y(:,3)))*1.1]);
    end
    subplot(nn,mm,3*mm+1)
    plot(t2,y(:,4))
    ylabel('swing leg knee')
    xlabel('time (sec)')
    grid
    if 0
        axis([min(t2) max(t2) -1e-4 1e-4]);
    else
        axis([min(t2) max(t2) min(min(y(:,4))) max(max(y(:,4)))])
    end
    
    if mm == 2
        % velocities
        subplot(nn,mm,2)
        plot(t2,dy(:,1))
        title('Output time derivatives')
        ylabel('dy_1 (q1-q1d)')
        xlabel('time (sec)')
        grid
        if 0
            axis([min(t2) max(t2) -1e-4 1e-4]);
        else
            axis([min(t2) max(t2) min(min(dy(:,1)))*1.1 max(max(dy(:,1)))*1.1]);
        end
        subplot(nn,mm,4)
        plot(t2,dy(:,2))
        ylabel('dy_2 (d1+d2)')
        xlabel('time (sec)')
        grid
        if 0
            axis([min(t2) max(t2) -1e-4 1e-4]);
        else
            axis([min(t2) max(t2) min(min(dy(:,2)))*1.1 max(max(dy(:,2)))*1.1]);
        end
        subplot(nn,mm,6)
        plot(t2,dy(:,3))
        ylabel('stance leg knee')
        xlabel('time (sec)')
        grid
        if 0
            axis([min(t2) max(t2) -1e-4 1e-4]);
        else
            axis([min(t2) max(t2) min(min(dy(:,3))) max(max(dy(:,3)))*1.1]);
        end
        subplot(nn,mm,8)
        plot(t2,dy(:,4))
        ylabel('swing leg knee')
        xlabel('time (sec)')
        grid
        if 0
            axis([min(t2) max(t2) -1e-4 1e-4]);
        else
            axis([min(t2) max(t2) min(min(dy(:,4))) max(max(dy(:,4)))])
        end
    end
    if doprints
        name = 'outputs';
        if strcmp(print_type,'ps')
            print('-dps',[name,'.ps'])
        else
            print('-djpeg',[name,'.jpg']);
        end
    end
end

plot_num = plot_num + 1;
if doplots(plot_num)
    [g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=modelParameters;
    fig_hl=figure(figno); figno = figno + 1;
    set(fig_hl,'Position', screen_position);
    screen_position = screen_position + offset;
    set(fig_hl,'PaperPosition',paper_position)
    
    subplot(2,1,1)
    plot(t,hip_pos(x))
    ylabel('Hip horizontal position')
    xlabel('time (sec)')
    subplot(2,1,2)
    plot(t,-L4*cos(x(:,3))-L3*cos(x(:,1))+L3*cos(x(:,2))+L4*cos(x(:,4)))
    grid
    xlabel('time (sec)')
    ylabel('Foot height (m)')
    axis([min(t) max(t) 0 ...
        max(-L4*cos(x(:,3))-L3*cos(x(:,1))+L3*cos(x(:,2))+L4*cos(x(:,4)))*1.1])
    if doprints
        name = 'foot_hip_traj';
        if strcmp(print_type,'ps')
            print('-dps',[name,'.ps'])
        else
            print('-djpeg',[name,'.jpg']);
        end
    end
end


plot_num = plot_num + 1;
if doplots(plot_num)
    [g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4]=modelParameters;
    fig_hl=figure(figno); figno = figno + 1;
    set(fig_hl,'Position', screen_position);
    screen_position = screen_position + offset;
    set(fig_hl,'PaperPosition',paper_position)
    
    subplot(2,1,1)
    plot(t,-L3.*cos(x(:,1))-L4*cos(x(:,3)))
    grid
    xlabel('time (sec)')
    title('Hip height (m)')
    axis([min(t) ...
        max(t) ...
        min(-L3.*cos(x(:,1))-L4*cos(x(:,3)))...
        max(-L3.*cos(x(:,1))-L4*cos(x(:,3)))]);
    
    subplot(2,1,2)
    plot(t,pi+x(:,3)-x(:,1),'-',t,pi+x(:,4)-x(:,2),'--')
    legend('stance leg','swing leg')
    grid
    xlabel('time (sec)')
    title('Relative knee angles (rad)')
    axis([min(t) ...
        max(t) ...
        min(min([pi+x(:,3)-x(:,1) pi+x(:,4)-x(:,2)]))...
        max(max([pi+x(:,3)-x(:,1) pi+x(:,4)-x(:,2)]))]);
    
    if doprints
        name = 'hip_height';
        if strcmp(print_type,'ps')
            print('-dps',[name,'.ps'])
        else
            print('-djpeg',[name,'.jpg']);
        end
    end
end


plot_num = plot_num + 1
if doplots(plot_num)
    fig_hl=figure(figno); figno = figno + 1;
    set(fig_hl,'Position', screen_position);
    screen_position = screen_position + offset;
    set(fig_hl,'PaperPosition',paper_position)
    
    subplot(3,1,1)
    plot(t2,f(:,1))
    ylabel('F_{tan} (N)')
    axis([min(t2) max(t2) min(f(:,1))*1.05 max(f(:,1))*1.1])
    grid
    subplot(3,1,2)
    plot(t2,f(:,2))
    ylabel('F_{norm} (N)')
    axis([min(t2) max(t2) min(f(:,2))*.9 max(f(:,2))*1.1])
    grid
    subplot(3,1,3)
    plot(t2,f(:,1)./f(:,2))
    ylabel('F_{tan}/F_{norm}');
    xlabel('time (sec)')
    axis([min(t2) max(t2) min(f(:,1)./f(:,2))*1.1 max(f(:,1)./f(:,2))*1.1])
    grid
    if doprints
        name = 'stance_leg_forces';
        if strcmp(print_type,'ps')
            print('-dps',[name,'.ps'])
        else
            print('-djpeg',[name,'.jpg']);
        end
    end
end
tilefigs