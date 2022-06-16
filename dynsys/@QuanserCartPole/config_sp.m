%% These methods are provided by Quanser.
function [ Mp, Lp, lp, Jp, Bp ] = config_sp(PEND_TYPE)
    % from Inch to Meter
    K_IN2M = 0.0254;
    % Set these variables (used in Simulink Diagrams)
    if strcmp( PEND_TYPE, 'LONG_24IN')
        % Pendulum Mass (with T-fitting)
        Mp = 0.230;
        % Pendulum Full Length (with T-fitting, from axis of rotation to tip)
        Lp = ( 25 + 1 / 4 ) * K_IN2M;  % = 0.6413;
        % Distance from Pivot to Centre Of Gravity
        lp = 13 * K_IN2M;  % = 0.3302
        % Pendulum Moment of Inertia (kg.m^2) - approximation
        Jp = Mp * Lp^2 / 12;  % = 7.8838 e-3
        % Equivalent Viscous Damping Coefficient (N.m.s/rad)
        Bp = 0.0024;
    elseif strcmp( PEND_TYPE, 'MEDIUM_12IN')
        % Pendulum Mass (with T-fitting)
        Mp = 0.127;
        % Pendulum Full Length (with T-fitting, from axis of rotation to tip)
        Lp = ( 13 + 1 / 4 ) * K_IN2M;  % = 0.3365
        % Distance from Pivot to Centre Of Gravity
        lp = 7 * K_IN2M;  % = 0.1778
        % Pendulum Moment of Inertia (kg.m^2) - approximation
        Jp = Mp * Lp^2 / 12;  % = 1.1987 e-3
        % Equivalent Viscous Damping Coefficient (N.m.s/rad)
        Bp = 0.0024;
    else 
        error( 'Error: Set the type of pendulum.' )
    end
end