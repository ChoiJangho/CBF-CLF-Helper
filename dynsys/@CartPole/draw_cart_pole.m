function draw_cart_pole(obj, x, theta, force, text1, text2)
%% Draw a single snapshot of cart-pole given the cart position and the pole angle
% x: position of the cart.
% theta: angle of the rod, upright is 0

    l = 1;
    xmin = -10; 
    xmax = 10;    
    height = 0.1;
    width  = 0.3;
    maxU = obj.u_max;
    if isempty(maxU)
        maxU = 10;
    end


    % Compute positions 
    cart = [ x + width,  height
             x + width, -height
             x - width, -height
             x - width,  height
             x + width,  height ];
    pendulum = [x, 0; x-2*l*sin(theta), cos(theta)*2*l];


    clf; hold on
    plot(0,2*l,'k+','MarkerSize',20,'linewidth',2)
    plot([xmin, xmax], [-height-0.03, -height-0.03],'k','linewidth',2)

    % Plot force
    plot([0 force/maxU*xmax],[-0.3, -0.3],'g','linewidth',10)

    % Plot the cart-pole
    fill(cart(:,1), cart(:,2),'k','edgecolor','k');
    plot(pendulum(:,1), pendulum(:,2),'r','linewidth',4)

    % Plot the joint and the tip
    plot(x,0,'y.','markersize',24)
    plot(pendulum(2,1),pendulum(2,2),'y.','markersize',24)

    % Text
    text(0,-0.3,'applied force')
    % text(0,-0.5,'immediate reward')
    if exist('text1','var')
      text(0,-0.9, text1)
    end
    if exist('text2','var')
      text(0,-1.1, text2)
    end

    set(gca,'DataAspectRatio',[1 1 1],'XLim',[xmin xmax],'YLim',[-2 2]);
    axis off;
    drawnow;
end