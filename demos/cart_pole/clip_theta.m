function theta_clipped = clip_angle(theta)
%% clip theta to the range of (-pi, pi]
    r = mod(theta, 2*pi);
    if r > pi
        theta_clipped = r - 2*pi;
    else
        theta_clipped = r;
    end
end