%% INITIALIZETORQUEBALANCING
%
% This is the initialization script for torque balancing simulations of 
% floating base robots using Matlab.
%
% Forward dynamics integration is available for the robot balancing on one 
% foot or two feet (6 or 12 contact constraints, respectively). The controller 
% ensures stability properties of the system around any set point in case of 
% k=1 (see [Nava et al, IROS 2016]. 
% Contact forces are evaluated though QP solver to ensure unilateral constraints, 
% and to constrain them inside the friction cones.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
clear  variables
close  all
clc

%% Paths definition and initialize the forward dynamics integration
% add the required paths. This procedure will make the paths consistent for
% any starting folder.
full_path = which('initializeTorqueBalancingErr.m');
project_path = fileparts(full_path);
robot_root   = [project_path, '/tools/robotFunctions'];
plots_root   = [project_path, '/tools/visualization'];
src_root     = [project_path, '/src'];
config_root  = [project_path, '/config'];
init_root    = [project_path,'/init'];

% add the paths
addpath(robot_root);
addpath(plots_root);
addpath(src_root);
addpath(config_root);
addpath(init_root);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% robot name 
CONFIG.robot_name = 'icubGazeboSim';
%% Configure the simulation
CONFIG.demo_movements                        = 0;                          %either 0 or 1
CONFIG.feet_on_ground                        = [1,1];                      %either 0 or 1; [left foot,right foot]
CONFIG.use_QPsolver                          = 0;                          %either 0 or 1

%% Visualization setup
% robot simulator
CONFIG.visualize_robot_simulator             = 1;                          %either 0 or 1
% forward dynamics integration results
CONFIG.visualize_integration_results         = 1;                          %either 0 or 1
CONFIG.visualize_joints_dynamics             = 1;                          %either 0 or 1

%% Integration time [s]
CONFIG.tStart                                = 0;
CONFIG.tEnd                                  = 10;
CONFIG.sim_step                              = 0.01;

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% ONLY FOR DEVELOPERS
% tolerances for pseudoinverse and QP
CONFIG.pinv_tol           = 1e-8;
CONFIG.pinv_damp          = 5e-6;
CONFIG.reg_HessianQP      = 1e-3;

%% Forward dynamics integration setup
% CONFIG.integrateWithFixedStep will use a Euler forward integrator instead
% of ODE15s to integrate the forward dynamics. It may be useful for debug.
CONFIG.integrateWithFixedStep = 1;                                         %either 0 or 1

% The fixed step integration needs a desingularization of system mass matrix
% in order to converge to a solution
if CONFIG.integrateWithFixedStep == 1
    
    CONFIG.massCorr = 0.05;
else
    CONFIG.massCorr = 0;
end

% Integration options. If the intrgration is slow, try to modify these
% options.
if CONFIG.demo_movements == 0
    CONFIG.options                   = odeset('RelTol',1e-3,'AbsTol',1e-3);
else
    CONFIG.options                   = odeset('RelTol',1e-6,'AbsTol',1e-6);
end

%% Visualization setup
% this script modifies the default MATLAB options for figures and graphics.
% This will result in a better visualization of the plots.
plot_set

% this is the figure counter. It is used to automatically adapt the figure
% number in case new figures are added
CONFIG.figureCont = 1;

%% trajectory parameter
time_struct.ti    =CONFIG.tStart;
time_struct.step  =CONFIG.sim_step;
time_struct.tf    =CONFIG.tEnd;
start_param = [5 ,2 ,-0.120249695321353,-0.0680999719842103,0.369603821651986];
CONFIG.references = AdHocTraj(time_struct,start_param);
CONFIG.traj_param = [-4.5811,-1.7824,5.1574,3.9221,5.5949,-0.0308,-0.1151,-0.0045,0.0070,-0.0277,0.4661,0.4640,0.4149,0.4518,0.3840]'; 

%% Initialize the robot model
wbm_modelInitialize('icubGazeboSim');
CONFIG.ndof = 25;

%% Initial joints position [deg]
leftArmInit  = [ -20  30  0  45  0]';
rightArmInit = [ -20  30  0  45  0]';
torsoInit    = [ -10   0  0]';

if sum(CONFIG.feet_on_ground) == 2
    
    % initial conditions for balancing on two feet
    leftLegInit  = [  90   0   0  -90  -5.5  0]';
    rightLegInit = [  90   0   0  -90  -5.5  0]';
    
elseif CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 0
    
    % initial conditions for the robot standing on the left foot
    leftLegInit  = [  25.5   15   0  -18.5  -5.5  0]';
    rightLegInit = [  25.5    5   0  -40    -5.5  0]';
    
elseif CONFIG.feet_on_ground(1) == 0 && CONFIG.feet_on_ground(2) == 1
    
    % initial conditions for the robot standing on the right foot
    leftLegInit  = [  25.5    5   0  -40    -5.5  0]';
    rightLegInit = [  25.5   15   0  -18.5  -5.5  0]';
end

% feet size
CONFIG.footSize  = [-0.07 0.07;       % xMin, xMax
                    -0.03 0.03];      % yMin, yMax
% joints configuration [rad]
CONFIG.qjInit    = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);


%% INITIALIZATION
% initialize the forward dynamics
initForwardDynamics(CONFIG);
