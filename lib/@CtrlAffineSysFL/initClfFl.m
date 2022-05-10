function initClfFl(obj, params)
%% Function that initializes the CLF under the feedback linearization structure.
    if ~strcmp(obj.output_option, 'phase')
        error("NotImplemented");
    end
    %% Set up CLF.
    % In controlSysFL, CLF is automatically set, so obj.n_clf can be
    % hard-coded to 1
    obj.n_clf = 1;
    %% There are three ways to specify the CLF.
    % Option 1. Directly specify params.P_FL, the gram matrix of the Lyapunov
    % function.
    % Option 2. Specify params.Q_FL, params.Kp_FL, params.Ld_FL, the
    % feedback gains, and sovle the lyapunov equation for the closed loop
    % system.
    % Option 3. Specify params.Q_FL, params.R_FL, the weight matrices in the LQR.
    if isfield(params, 'P_FL') || isfield(params, 'Gram_clf_FL') ...
            || (isfield(params, 'clf') && isfield(params.clf, 'P'))
        clf_type = 1;
    elseif isfield(params, 'Kp_FL') || isfield(params, 'Kd_FL')
        clf_type = 2;
    else
        clf_type = 3;
    end
    
    if clf_type == 1
        if isfield(params, 'P_FL')
            if size(params.P_FL, 1) ~= 2*obj.ydim || size(params.P_FL, 2) ~= 2 * obj.ydim
                error("Wrong params.P_FL size.")
            end
            obj.Gram_clf_FL = params.P_FL;
            obj.params = rmfield(obj.params, 'P_FL');
        elseif isfield(params, 'Gram_clf_FL')
            if size(params.Gram_clf_FL, 1) ~= 2*obj.ydim ...
                    || size(params.Gram_clf_FL, 2) ~= 2 * obj.ydim
                error("Wrong params.Gram_clf_FL size.")
            end
            obj.Gram_clf_FL = params.Gram_clf_FL;
            obj.params = rmfield(obj.params, 'Gram_clf_FL');
        else
            if size(params.clf.P, 1) ~= 2*obj.ydim || size(params.clf.P, 2) ~= 2 * obj.ydim
                error("Wrong params.clf.P size.")
            end
            obj.Gram_clf_FL = params.clf.P;
            obj.params.clf = rmfield(obj.params.clf, 'P');
        end
    elseif clf_type == 2
        if ~isfield(params, 'Kp_FL') && isfield(params, 'Kd_FL')
            error("Define variable in the params called 'Kp_FL', p gain for the CLF");
        end
        if ~isfield(params, 'Kd_FL') && isfield(params, 'Kp_FL')
            error("Define variable in the params called 'Kd_FL', d gain for the CLF");
        end
        if ~isfield(params, 'Q_FL')
            disp("Setting Q matrix for CLF to default identity matrix.");
            Q = eye(2*obj.ydim);
        else
            if length(params.Q_FL) == 1
                Q = params.Q_FL * eye(2*obj.ydim);
            elseif size(params.Q_FL, 1) ~= 2*obj.ydim
                error("Wrong Q_FL size.")
            else
                Q = params.Q_FL;
            end
        end
        fprintf("Setting the lyapunov function based on the feedback gains specified. \n");        
        Kp = params.Kp_FL;
        Kd = params.Kd_FL;
        A = [zeros(obj.ydim), eye(obj.ydim);
            -Kp*eye(obj.ydim), -Kd*eye(obj.ydim)];
        obj.Gram_clf_FL = lyap(A', Q); % A'P+PA=Q, find P
    elseif clf_type == 3
        if ~isfield(params, 'Q_FL')
            disp("Setting Q matrix for CLF to default identity matrix.");
            Q = eye(2*obj.ydim);
        else
            if length(params.Q_FL) == 1
                Q = params.Q_FL * eye(2*obj.ydim);
            elseif size(params.Q_FL, 1) ~= 2*obj.ydim || size(params.Q_FL, 2) ~= 2*obj.ydim
                error("Wrong Q_FL size.")
            else
                Q = params.Q_FL;
            end
        end
        if ~isfield(params, 'R_FL')
            error("Define params.R_FL (weight for the input), to set up based on the LQR.");
        else
            if length(params.R_FL) == 1
                R = params.R_FL * eye(obj.udim);
            elseif size(params.R_FL, 1) ~= obj.udim || size(params.R_FL, 2) ~= obj.udim
                error("Wrong R_FL size.")
            else
                R = params.R_FL;
            end
        end
        A = [zeros(obj.ydim), eye(obj.ydim); zeros(obj.ydim), zeros(obj.ydim)];
        B = [zeros(obj.ydim); eye(obj.ydim)];
        [~, obj.Gram_clf_FL] = lqr(A, B, Q, R);
    end
    
    if isfield(params, 'clf') && isfield(params.clf, 'rate')
        obj.clf_rate = params.clf.rate;
        obj.params.clf = rmfield(obj.params.clf, 'rate');
    elseif isfield(params, 'clf_rate')
        obj.clf_rate = params.clf_rate;
        obj.params = rmfield(obj.params, 'clf_rate');
    else
        obj.clf_rate = min(eig(Q))/max(eig(obj.Gram_clf_FL))/obj.eps_FL;
        fprintf("Setting clf rate automatically to %.3f \n", obj.clf_rate);
    end
    
end