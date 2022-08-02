function [p1,p5] = compute_p1_p5(m,which_param1,which_param2)
%COMPUTE_P1_P5   Computes the (n-1)th parameters of the Bezier
%   coefficients.  WHICH_PARAM is for the controller to which we are
%   switching.  Q is the configuration of the robot at the end of the
%   step.  M is norminalization scaling parameter for the transition
%   controller (as opposed to the controller we are switching from or
%   to).  DTHETA is the value of dtheta at the end of the transition
%   controller's step.  (See notes 4/2/01)
%
%    [P1,P5] = COMPUTE_P1_P5(M,WHICH_PARAM1,WHICH_PARAM2)

%Eric Westervelt
%4/1/01

global WHICH_PARAM PARAM_TMP

param_tmp_tmp = PARAM_TMP;
PARAM_TMP = [];

WHICH_PARAM = which_param1;
[a1,b1,m1] = controlParameters;
WHICH_PARAM = which_param2;
[a2,b2,m2] = controlParameters;

PARAM_TMP = param_tmp_tmp;
WHICH_PARAM = [];

p0_ctrl1 = a1([5;12;19;26]);
p1_ctrl1 = a1([6;13;20;27]);
p1 = m/m1*(-p0_ctrl1+p1_ctrl1)+p0_ctrl1;

p5_ctrl2 = a2([10;17;24;31]);
p6_ctrl2 = a2([11;18;25;32]);
p5 = m/m2*(p5_ctrl2-p6_ctrl2)+p6_ctrl2;