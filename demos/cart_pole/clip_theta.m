function x_clipped = clip_theta(x)
    x_clipped = x;
    theta = x(3);
    r = mod(theta, 2*pi);
    if r > pi
        x_clipped(3) = r - 2*pi;
    else
        x_clipped(3) = r;
    end
end