function perturbed_state = perturb(obj, state, perturbation)
%% Perturb the initial state. Useful in training script
% Perturb the initial state randomly and solve the nonlinear-op so that
% following 3 conditions are met
%   Stance foot is on the ground.
%   Stance foot's velocity is zero.
%   Swing foot is over the ground.

% Prevent meaningless infinity loop (zero-perturbation fault)
if perturbation == 0
    perturbed_state = state;
    return;
end

while(true)
    % Initial perturbance (raw)
    sign_vector = 2*((rand(obj.xdim, 1) < 0.5) - 0.5);
    perturbation_coeff = rand(obj.xdim, 1).* sign_vector*perturbation;
    perturbed_state = state.* (1+perturbation_coeff);
    options = optimoptions('fsolve','display','off', ...
        'Algorithm', 'levenberg-marquardt');
    % Nonlinear optimize so that p_RightToe(3) = 0, v_RightToe(3)=0
    constrVars = fsolve(@(z0) fzeroFunc(z0,perturbed_state), perturbed_state([2, 7, 8, 9]),options);
    perturbed_state([2, 7, 8, 9]) = constrVars;
    pFootSwing = p_LeftToe(perturbed_state(1:7));
    % If swing foot is under ground reconduct the perturbance.
    if pFootSwing(3) > 0
        break
    end
end
end
function constr= fzeroFunc(z0, x0)
    yInit = z0(1);
    qSwingKneeInit = z0(2);
    dxInit = z0(3);
    dyInit = z0(4);

    qInit = x0(1:7);
    qInit(2) = yInit;
    qInit(7) = qSwingKneeInit;
    pFootStance = p_RightToe(qInit);

    dqInit = x0(8:14);
    dqInit(1) = dxInit;
    dqInit(2) = dyInit;

    vFootStance = J_RightToe(qInit)*dqInit;
    vFootStance = vFootStance([1,3]);

    % forward kinematics constraint for pfoot, vfoot 
    % => constraining all the belows are 0
    constr = [pFootStance(3);
              vFootStance];    
end