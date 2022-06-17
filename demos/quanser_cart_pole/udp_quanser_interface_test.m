clear all

params.IP02_LOAD_TYPE = 'WEIGHT';
params.PEND_TYPE = 'LONG_24IN';
params.u_max = 10; % max voltage
params.u_min = -10; % min voltage
params.x_lim = 0.3;
params.weight_slack = 1e3;
params.k_cbf = 10;

params.clf.rate = 1;
params.cbf.rate = 10;
cartPole = QuanserCartPoleClone(params);
controller = @cartPole.ctrlCbfClfQp;

% UDP setting.
PORT = 8080;  % port of this server
udp_handle = udpport("datagram","IPV4","LocalPort", PORT);
while true
    if (udp_handle.NumDatagramsAvailable > 0)  % if udp was received
        
        % read udp
        datagram = read(udp_handle, udp_handle.NumDatagramsAvailable, "double");
        
        % data is the current state.
        x = datagram.Data;
        senderAdress = datagram.SenderAddress;
        senderPort = datagram.SenderPort;
        [u, ~] = controller(x');
        disp(u)
        write(udp_handle, u, "double", senderAdress, senderPort);
    end
    pause(0.001);    
end
% send back the same data to the client
