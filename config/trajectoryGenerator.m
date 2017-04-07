function  desired_x_dx_ddx_CoM = trajectoryGenerator(t,CONFIG)
%TRAJECTORYGENERATOR generates a desired CoM trajectory. The default trajectory
%                    is a sinusoid in the Y direction.
%
% desired_x_dx_ddx_CoM = TRAJECTORYGENERATOR(xCoMInit,t,CONFIG) takes as an 
% input the initial CoM position, xCoMInit, the current time t and the structure
% CONFIG which contains the user-defined parameters.
% The output desired_x_dx_ddx_CoM is a matrix [3x3] which contains the CoM
% reference acceleration, velocity and position.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% Config parameters

xCoMDes   = CONFIG.references.p(t,CONFIG.traj_param);
dxCoMDes  = CONFIG.references.pd(t,CONFIG.traj_param);
ddxCoMDes =CONFIG.references.pdd(t,CONFIG.traj_param);
desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes];
% 
% xCoMDes   =CONFIG.xCoMRef;
% dxCoMDes  =zeros(size(xCoMDes));
% ddxCoMDes =zeros(size(xCoMDes));
% desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes];
   
end



