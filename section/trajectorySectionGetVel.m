function [vel] = trajectorySectionGetVel(traj_section, varargin)
% trajectorySectionGetVel returns the velocity from a trajectory
%   [Detailed description of the function]
%   The function returns a velocity vector from a given trajactory.
%   
% Inputs:
%   traj_section 	trajectory section struct, see trajectorySectionInit       
%
%   t               dimensionless time paramter 
%                   (scalar), [0-1]
%
% Outputs:
%   vel         velocity [vx; vy; vz] in local geodetic coordinate system
%               (3x1 vector), in m/s
%
% Syntax:
%   vel = trajectorySectionGetVel(traj_section) 
%   vel = trajectorySectionGetVel(traj_section,t) 
%
% Literature:
%   [1] Author (Year): Title, Publisher.
%   [2] Author (Year): Title, Publisher.
%
% See also: trajectorySectionGetPos, trajectorySectionGetAcc
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

t = 0;

if isempty(varargin)
    t(:) = traj_section.t;
else
    t(:) = varargin{1};
end   

% Calculate scalar-velocity with t
a = traj_section.vel.a;
b = traj_section.vel.b;
c = traj_section.vel.c;
d = traj_section.vel.d;

scalar_velocity = a + b.*(t) + c.*(t)^2 + d.*(t)^3; 

% Calculate first Derivative of path
[first_deriv, ~] = trajectorySectionGetDerivatives(traj_section, t);
  
% Calculate unit-length derivative vector
norm_firstDerivative = first_deriv / max(eps(0), norm(first_deriv));

% Calculate velocity in local geodetic coordinate system
vel = scalar_velocity * norm_firstDerivative;

end

