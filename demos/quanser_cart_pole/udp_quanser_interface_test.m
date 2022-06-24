clear all

use_cbf_filter = true;
params = get_predefined_params_set_for_vanilla_cart_pole('QUANSER', 'WEIGHT');

% Set up input saturation limit.
params.u_max = (params.m + params. M) * 10;
% Set up CLF-related parameters
params.clf.rate = 0.5;
params.weight_slack = 1e10;
% Create the dynamic system to simulate.
model_sys = CartPole(params);

params.k_cbf = 10;
params.x_lim = 0.35;
params.cbf.rate = 10;
params.u_max = 6;
params.u_min = -6;
dynsys = QuanserCartPole(params);
controller_for_force = @(x, varargin) model_sys.ctrl_hybrid_swing_up( ...
  [], x, 'k_energy', 10, varargin{:});
if ~use_cbf_filter
    %% Low-level controller maps desired force to input voltage.
    controller = @(x, varargin) dynsys.ctrl_voltage_for_desired_force( ...
        [], x, controller_for_force, varargin{:});
else
    controller_unfiltered = @(x, varargin) dynsys.ctrl_voltage_for_desired_force( ...
        [], x, controller_for_force, varargin{:});
    controller = @(x, varargin) dynsys.ctrlCbfQp(x, ...
        'u_ref', controller_unfiltered, varargin{:});
end

% UDP setting.
PORT = 8080;  % port of this server
udp_handle = udpport("datagram","IPV4","LocalPort", PORT);
while true
    if (udp_handle.NumDatagramsAvailable > 0)  % if udp was received
        
        % read udp
        datagram = read(udp_handle, udp_handle.NumDatagramsAvailable, "double");
        
        % data is the current state.
        x = datagram.Data;
        disp(x)
        senderAdress = datagram.SenderAddress;
        senderPort = datagram.SenderPort;
        [u, ~] = controller(x');
        write(udp_handle, u, "double", senderAdress, 8081);
    end
    pause(0.002);    
end
% send back the same data to the client
