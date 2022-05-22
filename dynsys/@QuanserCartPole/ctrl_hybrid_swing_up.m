function [u, extraout] = ctrl_hybrid_swing_up(obj, t, x, k_swing_up, epsilon_switch, varargin)
% Args:
%   x: state
%   k_swing_up: proportional feedback gain for pivot acceleration.
% Reference:
%   Quanser, Lienar Inverted Pendulum, Student Workbook, Sec.3.1.2.
    theta = x(2);
    if abs(clip_angle(theta)) < epsilon_switch
       [u, extraout] = obj.ctrlClfQp(x);
       % append dummy values
       extraout.E = obj.get_total_energy2(x);
       extraout.swing_up = 0;       
    else
        [u, extraout] = ctrl_swing_up(obj, t, x, k_swing_up);
        extraout.slack = 0;
        extraout.Vs = 0;
        extraout.feas = 0;
        extraout.comp_time = 0;
        extraout.swing_up = 1;
    end
end

function [u, extraout] = ctrl_swing_up(obj, t, x, k_swing_up)
    theta = x(2);    
    dx = x(3);
    dtheta = x(4);
    E = obj.get_total_energy2(x)
    if E < 0
        if t < 0.05
            u = obj.u_max;
        else    
            u = obj.u_max * (dtheta * cos(theta) <= 0) + obj.u_min * (dtheta * cos(theta) > 0);
        end
    else
        u = 0;
    end
    extraout.E = E;
%     a_raw = - k_swing_up * (E - obj.Er) * sign(dtheta * cos(theta));
%     a = min(a_raw, obj.a_max);
%     a = max(a, -obj.a_max);
%     constant_a = obj.Mc * obj.Rm * obj.r_mp / (obj.eta_g * obj.Kg * obj.eta_m * obj.Kt);
%     constant_dx = obj.Kg * obj.Km;
%     u = constant_a * a + constant_dx * dx;
%     extraout.a = a;
%     extraout.a_raw = a_raw;
%        
end