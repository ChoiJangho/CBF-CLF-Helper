function clf = defineClf(obj, params, symbolic_state)
    x = symbolic_state;
    [~, f_, g_] = defineSystem(obj, params);
    A_sym = jacobian(f_, x);
    obj.A_sym = matlabFunction(A_sym, 'vars', {x});
    obj.A_origin = double(subs(A_sym, x, zeros(4, 1)));
    obj.B_origin = double(subs(g_, x, zeros(4, 1)));
    obj.A_stable_eq = double(subs(A_sym, x, [0; pi; 0; 0]));
    obj.B_stable_eq = double(subs(g_, x, [0; pi; 0; 0]));
    
    l = obj.params.l;  % [m]      length of pendulum
    m = obj.params.m;  % [kg]     mass of pendulum
    M = obj.params.M;  % [kg]     mass of cart
    g = obj.params.gravity; % [m/s^2]  acceleration of gravity
    A = obj.A_origin;
    B = obj.B_origin;
    Q = diag([100.0,100,10.0,0.01]);
    R = obj.udim;
    [P, K] = icare(A, B, Q, R);
    obj.P_origin = P;
    obj.K_origin = K;
    obj.clf_rate = -max(real(eig(A - B * K)));
    clf = x' * P * x;
    [obj.P_stable_eq, obj.K_stable_eq] = icare(obj.A_stable_eq, obj.B_stable_eq, Q, R);
    obj.clf_rate_stable_eq = -max(real(eig(obj.A_stable_eq - obj.B_stable_eq * obj.K_stable_eq)));
end