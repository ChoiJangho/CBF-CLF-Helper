clear all
PORT = 7070;  % port of this server
udp_handle = udpport("datagram","IPV4","LocalPort", PORT)
while true
    data = [0;deg2rad(12);0;0];
    write(udp_handle, data, "double", "0.0.0.0", 8080);
    pause(0.01);
%     if (u.NumDatagramsAvailable > 0)  % if udp was received
%         
%         % read udp
%         datagram = read(u, u.NumDatagramsAvailable, "uint8")
%         
%         % get datagram components
%         data = datagram.Data
%         senderAdress = datagram.SenderAddress
%         senderPort = datagram.SenderPort
%         
%         break
%     end
end
% send back the same data to the client
