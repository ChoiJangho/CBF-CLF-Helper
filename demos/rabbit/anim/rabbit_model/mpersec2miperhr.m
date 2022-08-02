function ve=mpersec2miperhr(vsi)
%MPERSEC2MIPERHR Convert units from m/sec to mi/hr.
%   MPERSEC2MIPERHR(VE) is the velocity VE in the units of m/sec.
%   
%      ve = 1/100*2.54*12*5280/3600*vsi
%
%   Example
%       vsi = mpersec2miperhr(1)
%          returns 0.44704

%Eric Westervelt
%7/17/00

ve = 100/2.54/12/5280*3600*vsi;