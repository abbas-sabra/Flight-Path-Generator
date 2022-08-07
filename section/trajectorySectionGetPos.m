function [pos] = trajectorySectionGetPos(traj_section, varargin)
% trajectorySectionGetPos returns the position from a trajectory
%   [Detailed description of the function]
%   The function returns a positon vector from a given trajectory.
%
% Inputs:
%   traj_section 	trajectory section struct, see trajectorySectionInit         
%
%   t               dimensionless time paramter 
%
% Outputs:
%   pos             position [x; y; z] in local geodetic coordinate system
%                   (3x1 vector), in m
%
% Syntax:
%   [pos] = trajectorySectionGetPos(traj) 
%   [pos] = trajectorySectionGetPos(traj,t) 
%
% Literature:
%   [1] Author (Year): Title, Publisher.
%   [2] Author (Year): Title, Publisher.
%
% See also: trajectorySectionInit, trajectorySectionGetVel, trajectorySectionGetAcc
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

t = 0;

if isempty(varargin)
    t(:) = traj_section.t;
else
    t(:) = varargin{1};
end    

% Calculate x-positon with t
ax = traj_section.pos_x.a;
bx = traj_section.pos_x.b;
cx = traj_section.pos_x.c;
dx = traj_section.pos_x.d;

px = ax + bx.*(t)+ cx.*(t)^2 + dx.*(t)^3; 


% Calculate y-positon with t
ay = traj_section.pos_y.a;
by = traj_section.pos_y.b;
cy = traj_section.pos_y.c;
dy = traj_section.pos_y.d;

py = ay + by.*(t)+ cy.*(t)^2 + dy.*(t)^3; 

% Calculate z-positon with t

az = traj_section.pos_z.a;
bz = traj_section.pos_z.b;
cz = traj_section.pos_z.c;
dz = traj_section.pos_z.d;

pz = az + bz.*(t)+ cz.*(t)^2 + dz.*(t)^3;


pos = [px; py; pz];

end

