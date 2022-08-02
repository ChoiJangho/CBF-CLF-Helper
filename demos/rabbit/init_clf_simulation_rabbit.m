%% Initialize Global Simulation Configuration of Rabbit

%% Control Mode: You can use various control options
params.mode.motor_dynamics = 1; % going to be supported
    
%% Physical Property
params.physics.mu = 0.8; % friction constraint;
    
%% Reference gait parameters. Keep this value fixed.
params.theta_end = 3.384545423940834;
params.theta_init = 2.903323878938605;
params.a_bez = [2.42225522164138,2.50944978892455,2.66152682946601,2.78780950883388,2.88091497548752,2.94002391033518;0.667715484387163,0.638300798629787,0.490574595122909,0.478548440963253,0.473893838339675,0.576633115386526;2.94201463482336,2.99945422582175,2.85290119992043,2.52843337205465,2.42762552077273,2.44116379159792;0.575072495911993,0.635317622337650,1.13813693839107,1.22909640103678,1.00446628056820,0.717766128548961];
    
%% Basic settings for the CtrlAffineSysFL
params.xdim = 14;
params.udim = 4;
params.rel_deg_y = 2;
params.use_phase = false;
    
% scale changes the mass and inertial values of the robot.
% Note that the reference gait is designed for the scale of 1.
params.scale = 1;
% You can introduce an additional mass to the torso.
params.torso_add = 0;
    
%% Input options
params.u_max = 150;
params.u_min  = -150;

% Be careful when tuning this! Low epsilon induces high performance,
% but it can be too aggresive to disregard some physical limitations
% and unknown dynamics
params.epsilon_FL = 0.07; % Keep it fixed! Fragile!
params.Kp_FL = 1;
params.Kd_FL = 1.8;

%% Quadratic Programming Parameters
% objective function = A * |mu|^2 + B * |delta|^2
% tune A and B to control the weight of each variable
params.weight.input = 1; % A (can be scalar or matrix)
params.weight.slack = 100; % B (should be scalar)

%% Defines the initial state of the reference gait.
q0_ini = [-0.161463255651884;0.739782734886561;0.152681626367748;2.44507104015569;0.658106151373543;2.95111227794945;0.599243461678844];
dq0_ini = [0.781190436522269;0.243063912942872;0.247793330530586;1.10287289727466;-0.548512132037030;0.203061891957879;1.50818729243256];
