function fig = animate_cart_pole(obj, xs, us, ts, dt, title_text)
    fig = figure;
    for r = 1:length(ts)-1
        obj.draw_cart_pole(xs(1, r), xs(2, r), us(r),  ...
              ['t=' num2str(r*dt) ' sec'], title_text);
        pause(dt);
    end
end           