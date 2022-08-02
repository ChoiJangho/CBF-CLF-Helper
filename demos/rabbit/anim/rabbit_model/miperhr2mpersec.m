function vsi=miperhr2mpersec(ve)
%MIPERHR2MPERSEC Convert units from mi/hr to m/sec.
%   MIPERHR2MPERSEC(VE) is the velocity VE in the units of mi/hr.
%   
%      vsi = 100/2.54/12/5280*3600*ve
%
%   Example
%       vsi = miperhr2mpersec(1)
%          returns 0.44704

%Eric Westervelt
%7/17/00

vsi = 5280*12*2.54/100/3600*ve;