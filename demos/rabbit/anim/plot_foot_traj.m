function plot_foot_traj(t,x,box_height)
%PLOT_FOOT_TRAJ    Plot foot trajectory with constraint box.

%Eric Westervelt
%3/23/01

fig_hl = figure(1);
grid

h = swing_foot_height(x);
Ts = t(length(t));
Il = find(t > .1*Ts);
Iu = find(t < .96*Ts);
if Il(1) > Iu(length(Iu))
  error('check PLOT_FOOT_TRAJ.M here');
end

plot(t,h);
bx = t([Il(1) Iu(length(Iu)) Iu(length(Iu)) Il(1)]);
by = [0; 0; box_height; box_height];
box = patch(bx,by,'r');
set(box,'EdgeColor','r');