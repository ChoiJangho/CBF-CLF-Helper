function plot_quanser_cart_pole_result(t, xs, us, Bs, Vs, slacks)
if size(xs, 1) ~= 4
    xs = xs';
end

figure
subplot(5,1,1)
plot(t, xs(1, :))
ylabel('x [m]')
grid on

subplot(5,1,2)
plot(t, xs(2, :))
ylabel('theta [rad]')
grid on

subplot(5,1,3)
plot(t, xs(3, :))
ylabel('dx [m/sec]')
grid on

subplot(5,1,4)
plot(t, xs(4, :))
ylabel('dtheta [rad/sec]')
grid on

subplot(5, 1, 5)
plot(t, us)
xlabel('t [sec]')
ylabel('u [V]')
grid on

figure
n_sub = ~isempty(Bs) + ~isempty(Vs) + ~isempty(slacks);
i_sub = 1;
if ~isempty(Bs)
    subplot(n_sub,1,i_sub)
    plot(t, Bs)
    xlabel('t')
    ylabel('B')
    i_sub = i_sub+1;
end

if ~isempty(Vs)
    subplot(n_sub,1,i_sub)
    plot(t, Vs)
    xlabel('t')
    ylabel('V')
    i_sub = i_sub + 1;
end

if ~isempty(slacks)
    subplot(n_sub,1,i_sub)
    plot(t, slacks)
    xlabel('t')
    ylabel('slack')
end

end