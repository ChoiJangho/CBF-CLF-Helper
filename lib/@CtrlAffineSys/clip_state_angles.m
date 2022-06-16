function x_clipped = clip_state_angles(obj, x)
x_clipped = x;    
x_clipped(obj.dims_angle) = clip_angle(x(obj.dims_angle));
end