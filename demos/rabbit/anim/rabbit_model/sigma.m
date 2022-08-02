function x0 = sigma(dtheta1,theta1)
%SIGMA    Maps horizontal velocity of hips just before impact to
%    state of the system just before impact.
%
%    X0 = SIGMA(DTHETA1,THETA1)

%Eric Westervelt
%01-May-2001 05:42:39

[a,b,m] = controlParameters;

Ah_inv = zeros(5);
Ah_inv(1,3)=-0.5;
Ah_inv(1,5)=1;
Ah_inv(2,2)=1;
Ah_inv(2,3)=-0.5;
Ah_inv(2,5)=1;
Ah_inv(3,3)=0.5;
Ah_inv(3,5)=1;
Ah_inv(4,2)=1;
Ah_inv(4,3)=-0.5;
Ah_inv(4,4)=1;
Ah_inv(4,5)=1;
Ah_inv(5,1)=1;

aN = a([7;14;21;28]);
aN_minus_1 = a([6;13;20;27]);

if nargin == 2
  q = Ah_inv*[aN; theta1];
else
  q = Ah_inv*[aN; b+m];
end

dq = Ah_inv*[-6*(aN_minus_1-aN)/m; 1]*dtheta1;

x0 = [q; dq];