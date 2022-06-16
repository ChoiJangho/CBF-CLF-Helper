function [u, extraout] = ctrl_pump_energy(obj, t, x, varargin)
% controller that pumps energy until it matches with the desired energy
% level of the pole.
% Args:
%  k_energy: feedback_gain for the energy pump. If not, given, we do the
%    bang-bang control based on the force limit.
%  energy_desired: desired energy level, default is set to the energy when
%    the pole is at upright and the system is at rest.
    kwargs = parse_function_args(varargin{:});
    if isfield(kwargs, 'k_energy')
        k_energy = kwargs.k_energy;        
    else
        k_energy = [];
    end
    if isfield(kwargs, 'energy_desired')
        energy_desired = kwargs.energy_desired;
    else
        energy_desired = obj.potential_energy_upright();
    end    
    theta = x(2);
    dx = x(3);
    dtheta = x(4);
    E = obj.pole_energy(x);
    delta_E = E - energy_desired;
%     if delta_E > 0
%         u = 0;
%         extraout.E = E;
%         return
%     end
    
    if ~isempty(k_energy)
        u = - k_energy * dtheta * cos(theta) * delta_E;
        u = min(u, obj.u_max);
        u = max(u, obj.u_min);        
    else
        % Bang-bang control
        u_sign = sign(- dtheta * cos(theta) * delta_E);
        if u_sign > 0
            u = obj.u_max;
        elseif u_sign < 0
            u = obj.u_min;
        else
            u = 0;
        end
    end
    extraout.E = E;
end