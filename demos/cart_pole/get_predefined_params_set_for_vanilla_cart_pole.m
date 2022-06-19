function params = get_predefined_params_set_for_vanilla_cart_pole(type, quanser_weight_type)
if nargin < 2
    quanser_weight_type = 'NO_LOAD';
end
if strcmp(type, 'ONES')
    params.l = 1;
    params.m = 1;
    params.M = 1;
    params.gravity = 1;
elseif strcmp(type, 'QUANSER')
    [ Mp, Lp, lp, Jp, Bp ] = QuanserCartPole.config_sp('LONG_24IN');
    [ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, Mc, r_mp, Beq] = ...
            QuanserCartPole.config_ip02(quanser_weight_type);                            
    params.l = lp;
    params.J = Jp;
    params.m = Mp;
    params.M = Mc;
    params.gravity = 9.81;
    params.L = Lp;            
    params.Jeq = Mc + eta_g * Kg^2 * Jm / r_mp^2;            
    params.b_pole = Bp;
    params.b_cart = Beq;
elseif strcmp(type, 'QUANSER_NO_DRAG')
    [ Mp, Lp, lp, Jp, Bp ] = QuanserCartPole.config_sp('LONG_24IN');
    [ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, Mc, r_mp, Beq] = ...
            QuanserCartPole.config_ip02(quanser_weight_type);                            
    params.l = lp;
    params.J = Jp;
    params.m = Mp;
    params.M = Mc;
    params.gravity = 9.81;
    params.L = Lp;            
    params.Jeq = Mc + eta_g * Kg^2 * Jm / r_mp^2;            
    params.b_pole = 0;
    params.b_cart = 0;
elseif strcmp(type, 'QUANSER_SIMPLE')
    [ Mp, Lp, lp, Jp, Bp ] = QuanserCartPole.config_sp('LONG_24IN');
    [ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, Mc, r_mp, Beq] = ...
            QuanserCartPole.config_ip02(quanser_weight_type);      
    params.l = lp;
    params.m = Mp;
    params.M = Mc;
    params.gravity = 9.81;       
else
    error("Undefined parameter set type.");
end