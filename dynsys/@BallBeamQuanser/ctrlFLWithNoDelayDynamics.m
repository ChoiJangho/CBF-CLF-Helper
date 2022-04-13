function [u, extraout] = ctrlFLWithNoDelayDynamics(obj, x, varargin)

fl_controller = @(x, varargin) obj.dynsys_no_delay.ctrlFeedbackLinearize( ...
    x, @(x, varargin) obj.dynsys_no_delay.ctrlSisoLinearFeedback(x, varargin), ...
    varargin{:});

[theta_d, extraout] = fl_controller(x, varargin{:});

k_servo = 12;    
u_raw = k_servo * (theta_d - x(3));
u = obj.clipInput(u_raw);
extraout.theta_d = theta_d;
extraout.u_raw = u_raw;
end

