function [ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, Mc, r_mp, Beq] = ...
        config_ip02(IP02_LOAD_TYPE)
    % from Inch to Meter
    K_IN2M = 0.0254;
    % from rad/s to RPM
    K_RDPS2RPM = 60 / ( 2 * pi );
    % from oz-force to N
    K_OZ2N = 0.2780139;

    % Motor Armature Resistance (Ohm)
    Rm = 2.6;
    % Motor Torque Constant (N.m/A)
    Kt = 1.088 * K_OZ2N * K_IN2M; % = .00767
    % Motor ElectroMechanical Efficiency [ = Tm * w / ( Vm * Im ) ]
    eta_m = 1;
    % Motor Back-EMF Constant (V.s/rad)
    Km = 0.804e-3 * K_RDPS2RPM; % = .00767
    % Rotor Inertia (kg.m^2)
    Jm = 5.523e-5 * K_OZ2N * K_IN2M; % = 3.9e-7
    % IP02 Cart Mass, with 3 cable connectors (kg)
    Mc2 = 0.57;
    % Cart Weight Mass (kg)
    Mw = 0.37;
    % Planetary Gearbox (a.k.a. Internal) Gear Ratio
    Kg = 3.71;
    % Planetary Gearbox Efficiency
    eta_g = 1;
    % Motor Pinion Radius (m)
    r_mp = 0.5 / 2 * K_IN2M;  %  = 6.35e-3
    if strcmp( IP02_LOAD_TYPE, 'NO_LOAD')
        Mc = Mc2;
        Beq = 4.3;
    elseif strcmp ( IP02_LOAD_TYPE, 'WEIGHT')
        Mc = Mc2 + Mw;
        Beq = 5.4;
    elseif strcmp ( IP02_LOAD_TYPE, '2WEIGHTS')
        Mc = Mc2 + 2*Mw;
        Beq = 6.5;
    else 
        error( 'Error: Set the IP02 load configuration.' )
    end
end