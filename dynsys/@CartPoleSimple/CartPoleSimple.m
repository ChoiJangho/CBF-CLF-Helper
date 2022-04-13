classdef CartPoleSimple < CtrlAffineSysFL
    properties
        A_sym % Symbolix expression for the Jacobian matrix at the origin
        A_origin % Jacobian Matrix at the origin
        B_origin % Jacobian Matrix at the origin
    end
    
    methods
        function obj = CartPoleSimple(params, output_to_regulate, feedback_gain)
        % output_to_regulate: 'theta', 's'
            params.output_to_regulate = output_to_regulate;
            params.K_siso = feedback_gain;
            params.rel_deg_y = 2;
            obj@CtrlAffineSysFL(params, 'symbolic', 'siso');
        end
        
        function [x, f, g] = defineSystem(obj, params)
            l = params.l;  % [m]      length of pendulum
            m = params.m;  % [kg]     mass of pendulum
            M = params.M;  % [kg]     mass of cart
            gravity = params.g; % [m/s^2]  acceleration of gravity
            if isfield(params, 'J');
                J = params.J;
            else
                J = (1/3) * m * l^2;
            end

            syms s ds theta dtheta real
            % theta: angle of the rod, upright is 0
            % s: position of the cart.
            % u: input force on the cart.
            x = [theta; s; dtheta; ds;];

            matrix_inertia = [m*l^2+J, -m*l*cos(theta);
                               -m*l*cos(theta), m+M];
            ddq_drift = matrix_inertia\[m*gravity*l*sin(theta);-m*l*dtheta^2*sin(theta)];

            f = [dtheta;
                ds;
                ddq_drift];    

            g = [0;
                0;
                matrix_inertia\[0;1]];
        end
        
        function [y, z] = defineOutputWithZeroCoords(obj, params, x)
            if strcmp(params.output_to_regulate, 'theta')
                y = x(1);
                z = [x(2); x(4)*cos(x(1))-x(3)];
            elseif strcmp(params.output_to_regulate, 's')
                y = x(2);
                z = [x(1); x(4)*cos(x(1))-x(3)];
            end
        end    
        
        function fig = draw_rollout(obj, xs, us, ts, dt, title_text)
            fig = figure;
            for r = 1:length(ts)-1
                obj.draw_cart_pole(xs(2, r), pi+xs(1, r), us(r),  ...
                      ['t=' num2str(r*dt) ' sec'], title_text);
                pause(dt);
            end
        end
        
        function draw_cart_pole(obj, x, theta, force, text1, text2)
            % theta: angle from below
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
            pendulum = [x, 0; x+2*l*sin(theta), -cos(theta)*2*l];


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
    end
end
