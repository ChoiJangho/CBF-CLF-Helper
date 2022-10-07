%% Main reference: Student Workbook - Linear Pendulum Gantry Experiment, Quanser
function [x, f, g] = defineSystem(obj, params)
        syms s ds theta dtheta u_actual real
        % theta: angle of the rod, upright is 0, in the
        % counter-clockwise direction.
        % s: position of the cart. (positive in the direction to right.)
        % u: motor voltage
        x = [s; theta; ds; dtheta; u_actual];
        q = [s; theta];
        dq = [ds; dtheta];
        q_act = s;
        V = obj.get_potential_energy(params, x);
        T = obj.get_kinetic_energy(params, x);
        [D, C, G, B] = get_lagrangian_dynamics(T, V, q, dq, q_act);
        % B is respect to F_c (force to cart, we have to convert it to
        % motor torque ( eq (2.11) in the manual)
        B_motor = B * params.constant_V2F;
        % The first drag term is very large.
        B_drag = [(params.constant_motor_drag + params.b_cart) * ds; params.b_pole * dtheta];
        
        % Equivalent of eq (2.9) and (2.10) in quanser manual
        f = [dq; D\(-C * dq - G - B_drag) + D\B_motor * u_actual; 
            -(1/params.tau_delay) * u_actual];
        g = [zeros(4, 1); 1/params.tau_delay];
end       