function out = limb_position(x,pH_horiz)
% LIMB_POSITION   Head height.
%   out = LIMB_POSITION(X,PH_HORIZ)
%   The members are pH, pT, pHead, pFem1, pFem2, pG1, pG2, pTib1,
%   pTib2, pFoot1, and pFoot2.

%Eric Westervelt
%24-Mar-2001 10:46:12

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4]= modelParameters;

[n,m] = size(x);
if n == 10, k = m; else, k = n; end

if n ~= 1 & m ~= 1
  out.pH1 = pH_horiz*ones(k,1);
  out.pH2 = -L3.*cos(x(:,1))-L4.*cos(x(:,3));
  out.pT1 = out.pH1 + 1/M1.*(-sin(x(:,5)).*MZ1+cos(x(:,5)).*MY1);
  out.pT2 = out.pH2 + 1/M1.*(cos(x(:,5)).*MZ1+sin(x(:,5)).*MY1);
  out.pHead1 = out.pH1 + -L1.*sin(x(:,5));
  out.pHead2 = out.pH2 + L1.*cos(x(:,5));
  out.pFem11 = out.pH1 + -MZ3/M3.*sin(x(:,1));
  out.pFem12 = out.pH2 + MZ3/M3.*cos(x(:,1));
  out.pFem21 = out.pH1 + -MZ3/M3.*sin(x(:,2));
  out.pFem22 = out.pH2 + MZ3/M3.*cos(x(:,2));
  out.pG11 = out.pH1 + -L3.*sin(x(:,1));
  out.pG12 = out.pH2 + L3.*cos(x(:,1));
  out.pG21 = out.pH1 + -L3.*sin(x(:,2));
  out.pG22 = out.pH2 + L3.*cos(x(:,2));
  out.pTib11 = out.pH1 + -MZ4/M4.*sin(x(:,3))-L3.*sin(x(:,1));
  out.pTib12 = out.pH2 + MZ4/M4.*cos(x(:,3))+L3.*cos(x(:,1));
  out.pTib21 = out.pH1 + -L3.*sin(x(:,2))-MZ4/M4.*sin(x(:,4));
  out.pTib22 = out.pH2 + L3.*cos(x(:,2))+MZ4/M4.*cos(x(:,4));
  out.pFoot11 = out.pH1 + -L3.*sin(x(:,1))-L4.*sin(x(:,3));
  out.pFoot12 = out.pH2 + L3.*cos(x(:,1))+L4.*cos(x(:,3));
  out.pFoot21 = out.pH1 + -L3.*sin(x(:,2))-L4.*sin(x(:,4));
  out.pFoot22 = out.pH2 + L3.*cos(x(:,2))+L4.*cos(x(:,4));
else
  out.pH1 = pH_horiz;
  out.pH2 = -L3*cos(x(1))-L4*cos(x(3));
  out.pT1 = out.pH1 + 1/M1*(-sin(x(5))*MZ1+cos(x(5))*MY1);
  out.pT2 = out.pH2 + 1/M1*(cos(x(5))*MZ1+sin(x(5))*MY1);
  out.pHead1 = out.pH1 + -L1*sin(x(5));
  out.pHead2 = out.pH2 + L1*cos(x(5));
  out.pFem11 = out.pH1 + -MZ3/M3*sin(x(1));
  out.pFem12 = out.pH2 + MZ3/M3*cos(x(1));
  out.pFem21 = out.pH1 + -MZ3/M3*sin(x(2));
  out.pFem22 = out.pH2 + MZ3/M3*cos(x(2));
  out.pG11 = out.pH1 + -L3*sin(x(1));
  out.pG12 = out.pH2 + L3*cos(x(1));
  out.pG21 = out.pH1 + -L3*sin(x(2));
  out.pG22 = out.pH2 + L3*cos(x(2));
  out.pTib11 = out.pH1 + -MZ4/M4*sin(x(3))-L3*sin(x(1));
  out.pTib12 = out.pH2 + MZ4/M4*cos(x(3))+L3*cos(x(1));
  out.pTib21 = out.pH1 + -L3*sin(x(2))-MZ4/M4*sin(x(4));
  out.pTib22 = out.pH2 + L3*cos(x(2))+MZ4/M4*cos(x(4));
  out.pFoot11 = out.pH1 + -L3*sin(x(1))-L4*sin(x(3));
  out.pFoot12 = out.pH2 + L3*cos(x(1))+L4*cos(x(3));
  out.pFoot21 = out.pH1 + -L3*sin(x(2))-L4*sin(x(4));
  out.pFoot22 = out.pH2 + L3*cos(x(2))+L4*cos(x(4));
end
