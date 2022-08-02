classdef CtrlAffineSysFL < CtrlAffineSys
    %% Control-Affine Dynamic System with Feeback Linearization utilities
    properties
        % Autonomous matrix of the Linearized output dynamics
        F_FL
        F_FL_eps % auxilary
        % Actuation matrix of the Linearized output dynamics
        G_FL
        % Dimension of output
        ydim
        % Relative degree of y
        rel_deg_y
        % Output as a function handle (assuming relative degree 2)
        y_sym
        phase_sym
        % 1st order Lie derivative of the output
        lf_y_sym
        lg_y_sym
        % 2nd order Lie derivatives of the output as function handles
        lglf_y_sym
        l2f_y_sym
        % output related functions needed handle phase bounds
        y_max_exceed_sym
        lf_y_max_exceed_sym
        lg_y_max_exceed_sym
        lglf_y_max_exceed_sym
        l2f_y_max_exceed_sym
        y_min_exceed_sym
        lf_y_min_exceed_sym
        lg_y_min_exceed_sym
        lglf_y_min_exceed_sym
        l2f_y_min_exceed_sym        
        
        % binary indicator on whether it uses phase-based output or not.
        use_phase        
        
        % rate of the RES-CLF.
        eps_FL
        % CLF under feedback linearization and its derivatives as function handles.
        Gram_clf_FL % Gram matrix of the CLF for feedback linearization.
        
    end
    
    methods
        function obj = CtrlAffineSysFL(params, setup_option)
            if nargin < 1
                error("Warning: params argument is missing.")
            end
            disp(setup_option)
            if nargin < 2
                setup_option = 'symbolic';
            end
            if strcmp(setup_option, 'built_in')
                setup_option = 'built-in';
            elseif strcmp(setup_option, 'builtin')
                setup_option = 'built-in';
            end
            
            obj@CtrlAffineSys(params, setup_option);
            obj.init_sys_FL(params);
        end
        
        function [y, phase, y_max_exceed, y_min_exceed] = defineOutput(obj, params, symbolic_state)
            y = [];
            phase = [];
            y_max_exceed = [];
            y_min_exceed = [];
        end
        
        function V = clf_FL(obj, y, dy)
            eta_eps = [(1/obj.eps_FL)*y; dy];
            V = transpose(eta_eps)*obj.Gram_clf_FL*eta_eps;
        end
        
        function lF_clf_ = lF_clf_FL(obj, y, dy)
            eta_eps = [(1/obj.eps_FL)*y; dy];
            P = obj.Gram_clf_FL;
            lF_clf_ = transpose(eta_eps)*(transpose(obj.F_FL_eps)*P+P*obj.F_FL_eps)*eta_eps;
        end
        
        function lG_clf_ = lG_clf_FL(obj, y, dy)
            eta_eps = [(1/obj.eps_FL)*y; dy];
            P = obj.Gram_clf_FL;
            lG_clf_ = (2 * (obj.G_FL'*P) * eta_eps)';
        end
        
        function [y, dy, L2fy, LgLfy, phase] = eval_y(obj, s)
            % Todo: interpret this and make it more robust!
            if ~obj.use_phase
                y = obj.y(s);
                dy = obj.lf_y(s);
                L2fy = obj.l2f_y(s);
                LgLfy = obj.lglf_y(s);
                phase = [];
                return
            end
            phase = obj.phase(s);
            if phase > obj.params.phase_max
                y = obj.y_max_exceed(s);
                dy = obj.lf_y_max_exceed(s);
                L2fy = obj.l2f_y_max_exceed(s);
                LgLfy = obj.lglf_y_max_exceed(s);
            elseif phase < obj.params.phase_min
                y = obj.y_min_exceed(s);
                dy = obj.lf_y_min_exceed(s);
                L2fy = obj.l2f_y_min_exceed(s);
                LgLfy = obj.lglf_y_min_exceed(s);
            else
                y = obj.y(s);
                dy = obj.lf_y(s);
                L2fy = obj.l2f_y(s);
                LgLfy = obj.lglf_y(s);
            end
        end
        
        %% Sym2Value Function
        % Help convenient use of function handlers regardless of built-in
        % and symbolic
        % TODO: As relative degree increase, this way of coding would be
        % inefficient. Other way to do this?
        function y_ = y(obj, x)
            % y_ = y(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the y = h(x) instead of defineOutput.
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.y(x) should be overriden by user.");
            end
            y_ = obj.y_sym(x);
        end
        
        function phase_ = phase(obj, x)
            % phase_ = phase(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the phase
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.phase(x) should be overriden by user.");
            end
            phase_ = obj.phase_sym(x);
        end
        
        function lf_y_ = lf_y(obj, x)
            % lf_y_ = lf_y(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the
            % L_f(y)
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lf_y(x) should be overriden by user.");
            end
            lf_y_ = obj.lf_y_sym(x);
        end
        
        function lg_y_ = lg_y(obj, x)
            % lg_y_ = lg_y(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the L_g(y).
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lg_y(x) should be overriden by user.");
            end
            lg_y_ = obj.lg_y_sym(x);
        end
        
        function lglf_y_ = lglf_y(obj, x)
            % lglf_y_ = lglf_y(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the L_g{L_f(y)}.
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lglf_y(x) should be overriden by user.");
            end
            lglf_y_ = obj.lglf_y_sym(x);
        end
        
        function l2f_y_ = l2f_y(obj, x)
            % l2f_y_ = l2f_y(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the
            % L2_f(y)
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.l2f_y(x) should be overriden by user.");
            end
            l2f_y_ = obj.l2f_y_sym(x);
        end
        
        function y_max_exceed_ = y_max_exceed(obj, x)
            % y_max_exceed_ = y_max_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the y_max_exceed(x)
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.y_max_exceed(x) should be overriden by user.");
            end
            y_max_exceed_ = obj.y_max_exceed_sym(x);
        end
        
        function lf_y_max_exceed_ = lf_y_max_exceed(obj, x)
            % lf_y_max_exceed_ = lf_y_max_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the
            % L_f(y_max_exceed)
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lf_y_max_exceed(x) should be overriden by user.");
            end
            lf_y_max_exceed_ = obj.lf_y_max_exceed_sym(x);
        end
        
        function lg_y_max_exceed_ = lg_y_max_exceed(obj, x)
            % lg_y_max_exceed_ = lg_y_max_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the
            % L_g(y_max_exceed)
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lg_y_max_exceed(x) should be overriden by user.");
            end
            lg_y_max_exceed_ = obj.lg_y_max_exceed_sym(x);
        end
        
        function lglf_y_max_exceed_ = lglf_y_max_exceed(obj, x)
            % lglf_y_max_exceed_ = lglf_y_max_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the CBF L_g{B(x)}.
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lglf_y_max_exceed(x) should be overriden by user.");
            end
            lglf_y_max_exceed_ = obj.lglf_y_max_exceed_sym(x);
        end
        
        function l2f_y_max_exceed_ = l2f_y_max_exceed(obj, x)
            % l2f_y_max_exceed_ = l2f_y_max_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the CBF L_g{B(x)}.
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.l2f_y_max_exceed(x) should be overriden by user.");
            end
            l2f_y_max_exceed_ = obj.l2f_y_max_exceed_sym(x);
        end
        
        function y_min_exceed_ = y_min_exceed(obj, x)
            % y_min_exceed_ = y_min_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the CBF L_g{B(x)}.
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.y_min_exceed(x) should be overriden by user.");
            end
            y_min_exceed_ = obj.y_min_exceed_sym(x);
        end
        
        function lf_y_min_exceed_ = lf_y_min_exceed(obj, x)
            % lf_y_min_exceed_ = lf_y_min_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the CBF L_g{B(x)}.
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lf_y_min_exceed(x) should be overriden by user.");
            end
            lf_y_min_exceed_ = obj.lf_y_min_exceed_sym(x);
        end
        
        function lg_y_min_exceed_ = lg_y_min_exceed(obj, x)
            % lg_y_min_exceed_ = lg_y_min_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the CBF L_g{B(x)}.
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lg_y_min_exceed(x) should be overriden by user.");
            end
            lg_y_min_exceed_ = obj.lg_y_min_exceed_sym(x);
        end
        
        function lglf_y_min_exceed_ = lglf_y_min_exceed(obj, x)
            % lglf_y_min_exceed_ = lglf_y_min_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the CBF L_g{B(x)}.
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lglf_y_min_exceed(x) should be overriden by user.");
            end
            lglf_y_min_exceed_ = obj.lglf_y_min_exceed_sym(x);
        end
        
        function l2f_y_min_exceed_ = l2f_y_min_exceed(obj, x)
            % l2f_y_min_exceed_ = l2f_y_min_exceed(obj, x)
            % For 'built-in' setup, override this function with the
            % user-defined implementation of the lie derivative of the CBF L_g{B(x)}.
            if strcmp(obj.setup_option, 'built-in')
                error("For; 'built-in' setup_option, obj.l2f_y_min_exceed(x) should be overriden by user.");
            end
            l2f_y_min_exceed_ = obj.l2f_y_min_exceed_sym(x);
        end   
    end
end
