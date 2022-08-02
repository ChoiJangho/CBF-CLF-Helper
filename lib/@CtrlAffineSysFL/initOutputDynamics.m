function initOutputDynamics(obj, symbolic_s, symbolic_f, symbolic_g, symbolic_y, symbolic_phase, symbolic_y_max_exceed, symbolic_y_min_exceed, params)
% Construct essential property of IO-FL, prepare for CLF
%   ex) F, G, F_epsilon, P, Q, A, gamma
% From 
%   params => epsilon_FL, Q_F, Kd_FL, Kp_FL
% To
%   object => ydim, udim, y, rel_deg_y, lf_y, lg_y, l2f_y, lglf_y
%          => y_exceed_max, lf_y_exceed_max, lg_y_exceed_max,
%          l2f_y_exceed_max, lglf_y_exeed_max as a function
%          => F_FL, G_FL, Gram_CLF_FL(P), F_FL_eps, clf_rate
    if isempty(symbolic_s) || isempty(symbolic_f) || isempty(symbolic_g)
        error('s, f, g is empty. Create a class function defineSystem and define your dynamics with symbolic expression.');
    end
    
    if ~isa(symbolic_f, 'sym')
        f_ = sym(symbolic_f);
    else
        f_ = symbolic_f;
    end
    if ~isa(symbolic_g, 'sym')
        g_ = sym(symbolic_g);
    else
        g_ = symbolic_g;
    end
    
    s = symbolic_s;
    obj.ydim = size(symbolic_y, 1);
    if obj.ydim ~= obj.udim
        error("dimension of output should be same as input dimension to apply feedback linearization.")
    end

    %% Set up and save function handles of output and its lie derivatives.
    dy = simplify(jacobian(symbolic_y, symbolic_s));
    lf_y_ = dy * f_;
    lg_y_ = simplify(dy * g_);
    if ~isAlways(norm(lg_y_) == 0) % ||Lgy||=0, relative degree is 1
        obj.rel_deg_y = 1;
        disp("WARNING: output relative degree 1. Currently, the CLF for feedback linearization only supports 2.");
        obj.y_sym = matlabFunction(symbolic_y, 'vars', {s});
        obj.lf_y_sym = matlabFunction(lf_y_, 'vars', {s});
        obj.lg_y_sym = matlabFunction(lg_y_, 'vars', {s});
    else
        dlf_y_ = simplify(jacobian(lf_y_, symbolic_s));
        l2f_y_ = dlf_y_ * f_;
        lglf_y_ = dlf_y_ * g_;
        if isAlways(norm(lglf_y_) == 0)
            error("output relative degree is higher than 2, this is currently not supported by the library.");
        end
        obj.rel_deg_y = 2;
        obj.y_sym = matlabFunction(symbolic_y, 'vars', {s});
        obj.lf_y_sym = matlabFunction(lf_y_, 'vars', {s});
        obj.l2f_y_sym = matlabFunction(l2f_y_, 'vars', {s});
        obj.lglf_y_sym = matlabFunction(lglf_y_, 'vars', {s});
        
        %% Phase related
        if ~isempty(symbolic_phase)
            obj.phase_sym = matlabFunction(symbolic_phase, 'vars', {s});
            % y to use when phase exceed max bound.
            dy_max_exceed = simplify(jacobian(symbolic_y_max_exceed, symbolic_s));
            lf_y_max_exceed_ = dy_max_exceed * f_;
            dlf_y_max_exceed_ = jacobian(lf_y_max_exceed_, symbolic_s);
            l2f_y_max_exceed_ = dlf_y_max_exceed_ * f_;
            lglf_y_max_exceed_ = dlf_y_max_exceed_ * g_;
            obj.y_max_exceed_sym = matlabFunction(symbolic_y_max_exceed, 'vars', {s});
            obj.lf_y_max_exceed_sym = matlabFunction(lf_y_max_exceed_, 'vars', {s});
            obj.l2f_y_max_exceed_sym = matlabFunction(l2f_y_max_exceed_, 'vars', {s});
            obj.lglf_y_max_exceed_sym = matlabFunction(lglf_y_max_exceed_, 'vars', {s});
            obj.phase_sym = matlabFunction(symbolic_phase, 'vars', {s});
            % y to use when phase exceed min bound.
            dy_min_exceed = simplify(jacobian(symbolic_y_min_exceed, symbolic_s));
            lf_y_min_exceed_ = dy_min_exceed * f_;
            dlf_y_min_exceed_ = jacobian(lf_y_min_exceed_, symbolic_s);
            l2f_y_min_exceed_ = dlf_y_min_exceed_ * f_;
            lglf_y_min_exceed_ = dlf_y_min_exceed_ * g_;
            obj.y_min_exceed_sym = matlabFunction(symbolic_y_min_exceed, 'vars', {s});
            obj.lf_y_min_exceed_sym = matlabFunction(lf_y_min_exceed_, 'vars', {s});
            obj.l2f_y_min_exceed_sym = matlabFunction(l2f_y_min_exceed_, 'vars', {s});
            obj.lglf_y_min_exceed_sym = matlabFunction(lglf_y_min_exceed_, 'vars', {s});
        end
            
%         %% Set up desired linear output dynamics.
%         if ~isfield(params, 'epsilon_FL')
%             error("Define variable in the params called 'epsilon_FL', which is the rate of RES-CLF");
%         end
%         eps = params.epsilon_FL;
%         obj.F_FL = [zeros(obj.ydim), eye(obj.ydim);
%             zeros(obj.ydim), zeros(obj.ydim)];
%         obj.F_FL_eps = [zeros(obj.ydim), 1/eps * eye(obj.ydim);
%             zeros(obj.ydim), zeros(obj.ydim)];
%         obj.G_FL = [zeros(obj.ydim);
%                     eye(obj.ydim)];
% 
%         %% Set up CLF.
%         if ~isfield(params, 'Kp_FL')
%             error("Define variable in the params called 'Kp_FL', p gain for RES-CLF");
%         end
%         if ~isfield(params, 'Kd_FL')
%             error("Define variable in the params called 'Kd_FL', d gain for RES-CLF");
%         end
%         if ~isfield(params, 'Q_FL')
%             disp("Setting Q matrix for CLF to default identity matrix.");
%             Q = eye(2*obj.ydim);
%         else
%             if size(params.Q_FL, 1) ~= 2*obj.ydim
%                 error("Wrong Q_FL size.")
%             end
%             Q = params.Q_FL;
%         end
%         Kp = params.Kp_FL;
%         Kd = params.Kd_FL;
%         A = [zeros(obj.ydim), eye(obj.ydim);
%             -Kp*eye(obj.ydim), -Kd*eye(obj.ydim)];
%         obj.Gram_clf_FL = lyap(A', Q); % A'P+PA=Q, find P 
% 
%         if isfield(params, 'clf')
%             if ~isfield(params.clf, 'rate')
%                 obj.params.clf.rate = min(eig(Q))/max(eig(obj.Gram_clf_FL))/eps;
%                 fprintf("Setting clf rate automatically to %.3f \n", obj.params.clf.rate);
%             end
%         else
%             obj.params.clf.rate = min(eig(Q))/max(eig(obj.Gram_clf_FL))/eps;
%             fprintf("Setting clf rate automatically to %.3f \n", obj.params.clf.rate);
%         end

%         obj.clf_FL = @(y, dy) control_lyapunov_function(y, dy, obj, eps, P);
%         obj.lF_clf_FL = @(y, dy) lF_clf(y, dy, obj, eps, P);
%         obj.lG_clf_FL = @(y, dy) lG_clf(y, dy, obj, eps, P);
    end   
end

% function V = control_lyapunov_function(y, dy, obj, eps, P)
%     y = obj.y(s);
%     dy = obj.lf_y(s);
%     eta_eps = [(1/eps)*y; dy];
%     V = transpose(eta_eps)*P*eta_eps;
% end
% 
% function lF_clf_ = lF_clf(s, obj, eps, P)
%     y = obj.y(s);
%     dy = obj.lf_y(s);
%     eta_eps = [(1/eps)*y; dy];
%     lF_clf_ = transpose(eta_eps)*(transpose(obj.F_FL_eps)*P+P*obj.F_FL_eps)*eta_eps;
% end
% 
% function lG_clf_ = lG_clf(s, obj, eps, P)
%     y = obj.y(s);
%     dy = obj.lf_y(s);
%     eta_eps = [(1/eps)*y; dy];
%     lG_clf_ = (2 * (obj.G_FL'*P) * eta_eps)';
% end
