function anim_flat_ground(ts, xs, dt, record_video)
if nargin < 3, dt = 1/20; end

if nargin < 4
    record_video = false;
end

if record_video
    video_name = ...
    [datestr(now,'YYYYMMDD_hhmmss') '.mp4'];
vout = VideoWriter(video_name,'MPEG-4');
vout.Quality = 100;
% vout.FrameRate = 1/dt;
vout.FrameRate = 0.25 * 1/dt;
vout.open;
end

palette = get_palette_colors();


n = length(ts);
if n == 1
    te = ts;
    xe = xs;
else
    [te,xe]=even_sample(ts',xs',1/dt);
    te = te';
    xe = xe';
    n = length(te);
end

fig_hl=open_figure('size', [720, 720]);
palette = get_palette_colors();

% Shifting the horizonal positions of the torso to set the initial position to 0.
xe(1, :) = xe(1, :) - xe(1, 1);
for i = 1:n
clf
set(gcf,'color','w');
%anim_axis=[-.8 .8 -1.5 1.4]; % for falling animation
anim_axis=[-.8 .8 -0.1 1.4];
axis off
axis(anim_axis)
axis equal
grid

draw_rabbit(fig_hl, xe(:, i));

% Draw ground
buffer=5;
ground= drawShadedRectangle([-buffer xe(1, i)+buffer],[0 -0.2], palette.orange, palette.white);

new_axis=anim_axis+[xe(1, i) xe(1, i) 0 0];
axis(new_axis);

drawnow;
if record_video
  current_frame = getframe(gcf); %gca does just the plot      
  writeVideo(vout,current_frame);
  pause(0.05);
end
end

end