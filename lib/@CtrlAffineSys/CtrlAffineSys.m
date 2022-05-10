%% Author: Jason Choi (jason.choi@berkeley.edu)
classdef CtrlAffineSys < handle    
    %% Control-Affine Dynamic System Class.
    % More comments may be present in source code

    properties
        setup_option % Set-up option: 'symbolic', 'built-in'
        is_sys_initialized = false
        
        %% necessary parameters

        clf_rate % rate used in the clf constraint.
        cbf_rate % rate used in the cbf constraint.

        u_max % max bound of control input.
        u_min % min bound of control input.
                
        xdim % State dimension
        udim % Control input dimension
        n_clf % Number of clf in use
        n_cbf % Number of cbf in use
        
        %% weights used in the built-in controllers

        weight_input % weight on control input
        weight_slack % weight on slack variable
        
        % Model parameters as a structure object, specific to the system.
        % This might contain other parameters used for setting up the dynsys,
        % for instance, ``clf_rate`` and ``cbf_rate``. However, referring to this 
        % variable's fields directly instead of the equivalent class variables
        % (when they exist) is not recommended.
        % (Always preferred to refer directly to the class properties.)
        params
               
        %% Functions generated from symbolic expressions.
        % (Used when setup_option is 'symbolic'.)

        f_sym % :math:`f` function generated from symbolic expression
        g_sym % :math:`g` function generated from symbolic expression
        cbf_sym % CBF function generated from symbolic expression
        lf_cbf_sym % :math:`L_f B(x)` function generated from symbolic expression
        lg_cbf_sym % :math:`L_g B(x)` function generated from symbolic expression
        clf_sym % CLF function generated from symbolic expression
        lf_clf_sym % :math:`L_f V(x)` function generated from symbolic expression
        lg_clf_sym % :math:`L_g V(x)` function generated from symbolic expression
    end
    
    methods
        function obj = CtrlAffineSys(params, setup_option, clone_to_built_in)
        % | Define a dynamical system through the ``CtrlAffineSys`` constructor
        % | ``dynsys = CtrlAffineSys(params)``
        % 
        % Examples:
        %  default usage:
        %    ``dynsys = CtrlAffineSys(params, 'symbolic')``
        %  if user-defined dynamics, cbf, clf are used:
        %    ``dynsys = CtrlAffineSys(params, 'built-in')``
        %  if you want to create built-in class from the symbolic option:
        %    ``dynsys = CtrlAffineSys(params, 'symbolic-built-in')``
        %
        % Args:
        %  params: dictionary of necessary parameters to define the dynsys
        %    you can pass the model parameters you want to use to define
        %    the dynamics, together with the below fields that is necessary
        %    to support the functionality of the library.
        %
        %    Necessary fields:
        %      | ``clf_rate`` / ``clf.rate``: rate for the clf constraint.
        %      | ``cbf_rate`` / ``cbf.rate``: rate for the cbf constraint.
        %    For the 'built-in' option:
        %      | ``xdim``: state dimension
        %      | ``udim``: control input dimension
        %      | ``u_min``: control min bound (scalar, or the vector of size u_min)
        %      | ``u_max``: control max bound        
        %  setup_option: ``'symbolic'`` or ``'builtin'``
            if nargin < 1
                error("Warning: params argument is missing.")
            end
            if nargin < 2
                setup_option = 'symbolic';
            end
            if nargin < 3
                clone_to_built_in = false;
            end
            if strcmp(setup_option, 'built-in') && clone_to_built_in
                error("clone_to_built_in is used is only for 'symbolic' or 'symbolic_set_up'.");
            end
            if clone_to_built_in
                setup_option = 'symbolic-built-in';
            end
            
            if strcmp(setup_option, 'built_in')
                setup_option = 'built-in';
            elseif strcmp(setup_option, 'builtin')
                setup_option = 'built-in';
            end
            obj.setup_option = setup_option;
            obj.init_sys(params);
        end
        
        function [x, f, g] = defineSystem(obj, params)
        % | For 'symbolic' setup, use this to define your dynamics
        % | ``[x, f, g] = defineSystem(obj, params)``
        %
        % Returns:
        %   [x, f, g]: 
        %
        %     x: symbolic state vector.
        %
        %     f: drift term, expressed symbolically wrt x.
        %
        %     g: control vector fields, expressed symbolically wrt x.
        %
            x = []; f = []; g = [];         
        end
        
        function clf = defineClf(obj, params, symbolic_state)
        % For 'symbolic' setup, use this to define the CLF.
        % ``clf = defineClf(obj, params, symbolic_state)``
        %
        % Args:
        %   symbolic_state: same symbolic state created in defineSystem
        %
        % CLF expressed symbolically wrt symbolic_state.
            clf = [];
        end
        
        function cbf = defineCbf(obj, params, symbolic_state)
        % cbf = defineCbf(obj, params, symbolic_state)
        % For 'symbolic' setup, use this to define the CBF.
        % symbolic state: same symbolic state created in defineSystem
        % cbf: CBF expressed symbolically wrt symbolic_state.
            cbf = [];
        end
                
        function dx = dynamics(obj, t, x, u)
        % dx = obj.dynamics(t, x, u)
        % Control-Affine Dynamics: xdot = f(x) + g(x) u
        % Inputs: t: time, x: state, u: control input
        %   x can be multiple element (size: (xdim, n_element))
        %   u can be multiple element (size: (udim, n_element))       
        % Output: dx: \dot(x) (size: (xdim, n_element))
            dx = obj.f(x) + obj.g(x) * u;
        end
        
        function f_ = f(obj, x)
        % f_ = obj.f(x)
        % For 'built-in' setup, override this function with the
        % user-defined implementation of f(x).
        % Autonomous vector fields (or drift).
        % x can be multiple element (size: (xdim, n_element))
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, f(x) should be overriden by user.");                
            end
            f_ = obj.f_sym(x);            
        end
        
        function g_ = g(obj, x)
        % g_ = obj.g(x)
        % For 'built-in' setup, override this function with the
        % user-defined implementation of g(x).
        % Control vector fields (or actuation effect).
        % x can be multiple element (size: (xdim, n_element))
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.g(x) should be overriden by user.");                
            end
            g_ = obj.g_sym(x);
        end
        
        function Vs = clf(obj, x)
        % Vs = obj.clf(x)
        % For 'built-in' setup, override this function with the
        % user-defined implementation of the Control Lyapunov Function V(x).
        % x can be multiple elements (size: (xdim, n_element))
        % Vs:  (size: (obj.n_clf, n_element))
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.clf(x) should be overriden by user.");                
            end
            n_states = size(x, 2);
            Vs = zeros(obj.n_clf, n_states);
            for i = 1:obj.n_clf
                clf_i = obj.clf_sym{i}(x);
                Vs(i, :) = clf_i;
            end
        end
        
        function LfVs = lf_clf(obj, x)
        % LfVs = obj.lf_clf(x)
        % For 'built-in' setup, override this function with the
        % user-defined implementation of the lie derivative of the CLF L_f{V(x)}.
        % x can be multiple elements (size: (xdim, n_element))
        % LfVs:  (size: (obj.n_clf, n_element))
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lf_clf(x) should be overriden by user.");
            end
            n_states = size(x, 2);
            LfVs = zeros(obj.n_clf, n_states);
            for i = 1:obj.n_clf               
                lf_clf_i = obj.lf_clf_sym{i}(x);
                LfVs(i, :) = lf_clf_i;
            end
        end
        
        function LgVs = lg_clf(obj, x)
        % LgVs = obj.lg_clf(x)
        % For 'built-in' setup, override this function with the
        % user-defined implementation of the lie derivative of the CLF L_g{V(x)}.
        % x can be multiple elements (size: (xdim, n_element))
        % LgVs:  (size: (obj.n_clf, obj.udim, n_element))        
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lg_clf(x) should be overriden by user.");
            end
            n_states = size(x, 2);
            LgVs = zeros(obj.n_clf, obj.udim, n_states);
            for i = 1:obj.n_clf
                lg_clf_i = reshape(obj.lg_clf_sym{i}(x), [], obj.udim)';
                LgVs(i, :, :) = lg_clf_i;
            end
        end
        
        function Vdots = dclf(obj, x, u)
        % Vdots = obj.dclf(x, u)
        % Model based estimate of the Lie derivatives of CLF
        % x can be multiple elements (size: (xdim, n_element))
        % u can be multiple elements (size: (udim, n_element))
        % Output size: (obj.n_clf, n_element)
            u = reshape(u, [1, size(u)]);
            u = permute(u, [2, 1, 3]);
            LfVs = obj.lf_clf(x);
            LgVs = obj.lg_clf(x);
            Vdots = LfVs + reshape(pagemtimes(LgVs, u), obj.n_clf, []);            
        end
            
        function Bs = cbf(obj, x)
        % Bs = obj.cbf(x)
        % For 'built-in' setup, override this function with the
        % user-defined implementation of the Control Barrier Function B(x).
        % x can be multiple elements (size: (xdim, n_element))
        % Bs:  (size: (obj.n_cbf, n_element))
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.cbf(x) should be overriden by user.");                
            end
            n_states = size(x, 2);
            Bs = zeros(obj.n_cbf, n_states);
            for i = 1:obj.n_cbf
                cbf_i = obj.cbf_sym{i}(x);
                Bs(i, :) = cbf_i;
            end
        end
        
        function LfBs = lf_cbf(obj, x)
        % LfBs = obj.lf_cbf(x)
        % For 'built-in' setup, override this function with the
        % user-defined implementation of the lie derivative of the CBF L_f{B(x)}.
        % x can be multiple elements (size: (xdim, n_element))
        % LfBs: (size: (obj.n_cbf, n_element))
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lf_cbf(x) should be overriden by user.");
            end
            n_states = size(x, 2);
            LfBs = zeros(obj.n_cbf, n_states);
            for i = 1:obj.n_cbf               
                lf_cbf_i = obj.lf_cbf_sym{i}(x);
                LfBs(i, :) = lf_cbf_i;
            end
        end
        
        function LgBs = lg_cbf(obj, x)
        % LgBs = obj.lg_cbf(x)
        % For 'built-in' setup, override this function with the
        % user-defined implementation of the lie derivative of the CBF L_g{B(x)}.
        % x can be multiple elements (size: (xdim, n_element))
        % LgBs: (size: (obj.n_cbf, obj.udim, n_element))
            if strcmp(obj.setup_option, 'built-in')
                error("For 'built-in' setup_option, obj.lg_cbf(x) should be overriden by user.");
            end
            n_states = size(x, 2);
            LgBs = zeros(obj.n_cbf, obj.udim, n_states);
            for i = 1:obj.n_cbf
                lg_cbf_i = reshape(obj.lg_cbf_sym{i}(x), [], obj.udim)';
                LgBs(i, :, :) = lg_cbf_i;
            end
        end
        
        function Bdots = dcbf(obj, x, u)
        % Bdots = obj.dcbf(x, u)
        % Model based estimate of the Lie derivatives of CBF
        % x can be multiple elements (size: (xdim, n_element))
        % u can be multiple elements (size: (udim, n_element))
        % Output size: (obj.n_cbf, n_element)
            u = reshape(u, [1, size(u)]);
            u = permute(u, [2, 1, 3]);
            LfBs = obj.lf_cbf(x);
            LgBs = obj.lg_cbf(x);
            Bdots = LfBs + reshape(pagemtimes(LgBs, u), obj.n_cbf, []);            
        end
        
        function train_data = reconstruct_train_data(obj, xs, us, varargin)
            % This reconstruction method adapts following philosophy
            % dV = V(t+1) - V(t) / dt
            % dV_hat = (LfV(x_t) + LfV(x_{t+1}))/2 + (LgV(x_t) +
            %           LgV(x_{t+1})/2 * u
            % dV_error = dV - dV_hat
            % There might be an alternative to evaluate
            % LfV((x_t+x_{t+1})/2) rather than the above method.
            settings = parse_function_args(varargin{:});
            
            % Sanity check
            [x_dim, num_data] = size(xs);
            [u_dim, num_data_u] = size(us);
            if x_dim ~= obj.xdim || u_dim ~= obj.udim
                error("Your data format is wrong. Make sure that your input abides by right format");
            end
            if isfield(settings, 'ts')
                ts = settings.ts;
            elseif isfield(settings, 'dt')
                ts = 0:settings.dt:(num_data-1)*settings.dt;
            else
                error("You must provide time information to reconstruct training data");
            end
            
            reconstruct_clf = isfield(settings, 'Vs');
            reconstruct_cbf = isfield(settings, 'Bs');
            
            if reconstruct_clf
                Vs = settings.Vs;
                dV_hats = zeros(1, num_data - 1);
                for i = 1:num_data-1
                    x = xs(:, i); u = us(:, i);
                    x_next = xs(:, i+1);
                    dV_hat = obj.lf_clf(x) + obj.lg_clf(x) * u;
                    dV_hat_next = obj.lf_clf(x_next) + obj.lg_clf(x_next) * u;
                    dV_hats(i) = (dV_hat + dV_hat_next)/2;
                end
                dVs = (Vs(2:end) - Vs(1:end-1))./(ts(2:end) - ts(1:end-1));
                dVs_error = dVs - dV_hats;   
                
                train_data.dVs_error = dVs_error;
            end
            
            if reconstruct_cbf
                Bs = settings.Bs;
                dB_hats = zeros(1, num_data-1);
                for i = 1:num_data-1
                    x = xs(:, i); u = us(:, i);
                    x_next = xs(:, i+1);
                    dB_hat = obj.lf_cbf(x) + obj.lg_cbf(x) * u;
                    dB_hat_next = obj.lf_cbf(x_next) + obj.lg_cbf(x_next) * u;
                    dB_hats(i) = (dB_hat + dB_hat_next)/2;
                end
                dBs = (Bs(2:end) - Bs(1:end-1))./(ts(2:end) - ts(1:end-1));
                dBs_error = dBs - dB_hats;   
                
                train_data.dBs_error = dBs_error;
            end
        end
    end
end

