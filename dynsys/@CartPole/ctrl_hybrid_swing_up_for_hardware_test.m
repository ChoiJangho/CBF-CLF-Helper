function [u, extraout] = ctrl_hybrid_swing_up_for_hardware_test(obj, t, x, varargin)
% Args:
%   x: state
%   epsilon_switch: angle threshold that switches to the CLF-QP.
% Reference:
%   Quanser, Lienar Inverted Pendulum, Student Workbook, Sec.3.1.2.
    kwargs = parse_function_args(varargin{:});
    if isfield(kwargs, 'epsilon_switch')
        epsilon_switch = kwargs.epsilon_switch;        
    else
        epsilon_switch = pi/18;
    end
    if isfield(kwargs, 'k_energy')
        k_energy = kwargs.k_energy;        
    else
        k_energy = [];
    end
    
    theta = x(2);
    if t < 0.2
       u = (obj.m + obj.M) * 5;
       extraout.E = obj.pole_energy(x);
       extraout.swing_up = 1;
       extraout.Vs = obj.clf(x);       
    elseif abs(clip_angle(theta)) < epsilon_switch
       [u, extraout] = obj.ctrlClfSontag(x);
       % [u, extraout] = obj.ctrlClfQp(x);
       % append dummy values
       extraout.E = obj.pole_energy(x);
       extraout.swing_up = 0;       
    else
        [u, extraout] = obj.ctrl_pump_energy(t, x, 'k_energy', k_energy);
        extraout.Vs = obj.clf(x);
%         extraout.feas = 0;
%         extraout.comp_time = 0;
%        extraout.slack = 0;
        extraout.swing_up = 1;
    end
end