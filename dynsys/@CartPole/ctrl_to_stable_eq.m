function [u, extraout] = ctrl_to_stable_eq(obj, t, x, varargin)
% controller for cart&pole reaching to the state where the pendulum is downright and the cart is
% at the center (basically, the stable equilibrium).
% Args:
%   x: state
%   k_energy: proportional feedback gain for pivot acceleration.
% Reference:
%   Quanser, Lienar Inverted Pendulum, Student Workbook, Sec.3.1.2.
    kwargs = parse_function_args(varargin{:});
    if isfield(kwargs, 'epsilon_switch')
        epsilon_switch = kwargs.epsilon_switch;        
    else
        epsilon_switch = pi/24;
    end
    if isfield(kwargs, 'k_energy')
        k_energy = kwargs.k_energy;
    else
        k_energy = [];
    end
    
    theta = x(2);
    if abs(clip_angle(theta - pi)) < epsilon_switch
       [u, extraout] = obj.ctrl_clf_sontag_to_stable_eq(t, x);
       % append dummy values
       extraout.E = obj.pole_energy(x);
    else
        [u, extraout] = obj.ctrl_pump_energy(t, x, ...
            'k_energy', k_energy, 'energy_desired', obj.potential_energy_downright());
    end
end

