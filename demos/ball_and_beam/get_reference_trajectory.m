function xi_d = get_reference_trajectory(t, amplitude, period, type)
    if nargin < 4
        type = 'sine';
    end
    
    omega = 2 * pi / period;    

% p_ref: reference position of the ball
% v_ref: reference velocity of the ball
% a_ref: reference acceleration of the ball
% j_ref: reference jerk of the ball
        
if strcmp(type, 'sine')
    %% Sine wave.
    p_ref = amplitude * sin(omega * t);
    v_ref = amplitude * omega * cos(omega * t);
    a_ref = - amplitude * omega^2 * sin(omega * t);
    j_ref = - amplitude * omega^3 * cos(omega * t);
elseif strcmp(type, 'square')    
    %% Square wave.
    p_ref = amplitude * sign(sin(omega * t));
    v_ref = 0;
    a_ref = 0;
    j_ref = 0;
else
    error('Unknown error.');
end

xi_d = [p_ref; v_ref; a_ref; j_ref];
end