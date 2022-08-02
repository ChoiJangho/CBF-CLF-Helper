function [g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = modelParameters
%MODELPARAMETERS   Model parameters for kneed biped.
%
%   [G,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = MODELPARAMETERS

%Eric Westervelt
%10/2/00
global animation_scale
scale = animation_scale;
% gravity
g=9.81;

% lengths
L1=0.625;
L3=0.4;
L4=0.4;

% mass
M1=12*scale;
M3=6.8*scale;
M4=3.2*scale;

% com
MY1=.2;
MZ1=4.;
MZ3=1.11;
MZ4=.41;

% link inertia
XX1=2.22*scale;
XX3=1.08*scale;
XX4=.93*scale;