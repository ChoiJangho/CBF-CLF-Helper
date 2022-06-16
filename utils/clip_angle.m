function thetas_clipped = clip_angle(thetas)
%% clip theta to the range of (-pi, pi]
    rs = mod(thetas, 2*pi);
    thetas_clipped = rs;
    thetas_clipped(rs > pi) = rs(rs > pi) - 2 * pi;
end