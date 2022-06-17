clear all
PORT = 8080;  % port of this server
udp_handle = udpport("datagram","IPV4","LocalPort", PORT)
while true
    if (udp_handle.NumDatagramsAvailable > 0)  % if udp was received
        
        % read udp
        datagram = read(udp_handle, udp_handle.NumDatagramsAvailable, "uint8");
        
        % get datagram components
        data = datagram.Data
        senderPort = datagram.SenderPort;
        
    end
    pause(0.01);    
end
% send back the same data to the client
