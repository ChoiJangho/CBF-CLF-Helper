function [value, isterminal, direction] = ball_out_of_range(obj, t, x)
    if x(1) > 0.20 || x(1) < -0.20 || x(3) > pi/2 - 0.01 || x(3) < -pi/2 + 0.02
        value = -1;
    else
        value = 1;
    end
    
    isterminal = 1;
    direction = -1;
end