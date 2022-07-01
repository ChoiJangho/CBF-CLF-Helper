%% Author Jason Choi (jason.choi@berkeley.edu)
%% Reference, Tedrake, Underactuated Robotics
classdef CartPoleFl < CtrlAffineSysFL & CartPole
    methods
        function obj = CartPoleFl(params, output_to_regulate, feedback_gain)
        % output_to_regulate: 'theta', 's'
        %   'theta': regulate angle of the pendulum (default)
        %   's': regulate position of the cart.
            if nargin < 2
                output_to_regulate = 'theta';
            end
            if nargin < 3            
                feedback_gain = [9, 6];
            end
            params.output_to_regulate = output_to_regulate;
            params.K_siso = feedback_gain;
            params.rel_deg_y = 2;
            obj@CartPole(params);
            obj@CtrlAffineSysFL(params, 'symbolic', 'siso');
        end
        
        function [y, z] = defineOutputWithZeroCoords(obj, params, x)
            if strcmp(params.output_to_regulate, 's')
                y = x(1);
                z = [x(2); x(3)*cos(x(2))-x(4)];
            elseif strcmp(params.output_to_regulate, 'theta')
                y = x(2);
                z = [x(1); x(3)*cos(x(2))-x(4)];
            end
        end    
    end
end
