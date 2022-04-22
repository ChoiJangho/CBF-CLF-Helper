function animate_ball_and_beam_sastry(ts, xs, ref_ps, video, play_speed, title_str, t_final, dt_frame)
if nargin < 4
    video = false;
end
if nargin < 5
    play_speed = 1.0;
end
if nargin < 6
    title_str = " ";
end
if nargin < 7
    t_final = ts(end);
end
if nargin < 8
    dt_frame = 0.04; % 25 fps
end
    L = 2 * ceil(max(xs(1, :)))+1;
	open_figure('size', [1600, 900], 'font_size', 18);
    dt = ts(2) - ts(1);
    if dt < dt_frame
        down_sample_ratio = floor(dt_frame / dt);
    end
    frame_rate = dt * down_sample_ratio / play_speed;

    if video
        extraArgs.videoFilename = ...
            [datestr(now,'YYYYMMDD_hhmmss') '.mp4'];
        vout = VideoWriter(extraArgs.videoFilename,'MPEG-4');
        vout.Quality = 100;
        vout.FrameRate = 1/frame_rate;
        vout.open;
    end
    title_str = strcat(title_str, " (play speed: x", num2str(play_speed, '%.2f'), ")");
    
    for i =1:down_sample_ratio:length(ts) 
        if ts(i) > t_final 
            break
        end
        clf;
        theta = xs(3, i);
        r = xs(1, i);

        alpha = theta;
        line([-0.5*L * cos(alpha), 0.5*L * cos(alpha)], ...
            [-0.5*L * sin(alpha), 0.5*L * sin(alpha)],'Color', 'k', 'LineWidth', 2);
        hold on;
        scatter(0, 0, 50, 'k');
        center_ball_x = r * cos(alpha);
        center_ball_y = r * sin(alpha);
        ball_radius = 0.5;
        center_ball_x = center_ball_x - ball_radius * sin(alpha);
        center_ball_y = center_ball_y + ball_radius * cos(alpha);
        draw_circle([center_ball_x, center_ball_y], ball_radius, 'actual');
            
        center_desired_x = (ref_ps(i)) * cos(alpha) - ball_radius * sin(alpha);
        center_desired_y = (ref_ps(i)) * sin(alpha) + ball_radius * cos(alpha);
        draw_circle([center_desired_x, center_desired_y], ball_radius, 'desired');        
        
        axis([-0.5*L, 0.5*L, -0.25*L, 0.25*L]) ;
        axis equal;
        title(title_str);
        grid on ;
        drawnow ;
        pause(0.001) ;
        if video
            current_frame = getframe(gcf); %gca does just the plot
            writeVideo(vout,current_frame);
        end        
            
    end

     if video
          vout.close
     end
     
end

function h = draw_circle(center,r, type)

hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
if strcmp(type, 'desired')
    h = plot(xunit, yunit, 'r:', 'LineWidth', 2);    
elseif strcmp(type, 'actual')
    fill(xunit,yunit,[1,1,1],'FaceColor',[0.5, 0.5, 0.5],'EdgeColor','none')
elseif strcmp(type, 'unsafe')
    fill(xunit,yunit,[1,1,1],'FaceColor',[1.0, 0.2, 0.2],'EdgeColor','none')    
else
    error("Unknown type.");    
end
hold off
end