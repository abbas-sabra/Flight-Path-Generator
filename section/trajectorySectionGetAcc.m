function [acc] = trajectorySectionGetAcc(traj_section, varargin)
% trajectorySectionGetAcc returns the acceleration from a trajectory
% section.
%   [Detailed description of the function]
%   The function returns a acceleration vector from a given trjactory 
%   section.
%
% Inputs:
%   traj_section   	trajectory section struct, see trajectorySectionInit        
%
%   t               dimensionless time paramter 
%                   (scalar), [0-1]
%
% Outputs:
%   acc             acceleration [ax; ay; az] in local geodetic system
%                   (3x1 vector), in m/s^2
%
% Syntax: 
%   [acc] = trajectoryGetAcc(traj)
%   [acc] = trajectoryGetAcc(traj,t) 
%
% Literature:
%   [1] Author (Year): Title, Publisher.
%   [2] Author (Year): Title, Publisher.
%
% See also: trajectoryGetPos, trajectoryGetVel,
%   trajectoryCreateFromWaypoints
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

t = 0;

if isempty(varargin)
    t(:) = traj_section.t;
else
    t(:) = varargin{1};
end   

% Calculate second Derivative of path
[first_deriv, sec_deriv] = trajectorySectionGetDerivatives(traj_section, t);

% Calculate the curvature
kappa = norm(cross(first_deriv,sec_deriv))./(norm(first_deriv))^3;
A = cross(first_deriv,sec_deriv);
N = (cross(A,first_deriv))./((norm(cross(first_deriv,sec_deriv))).*(norm(first_deriv)));

% Calculate velocity in local geodetic coordinate system
[vel] = trajectorySectionGetVel(traj_section, t);
G0 =[0; 0; -9.81];
acc = kappa.*(norm(vel).^2)*N+G0;
% 
% 
% 
% 
% % Calculate x-acceleration
% accx = kappa.*(vel(1,:).^2);
% 
% % Calculate y-acceleration
% accy = kappa.*(vel(2,:).^2);
% 
% % Calculate z-acceleration
% accz = kappa.*(vel(3,:).^2); 
% 
% % Calculate the acceleration [ax; ay; az] in local geodetic system
% % (3x1 vector), in m/s^2
% acc = [accx; accy; accz]+G0;

end



