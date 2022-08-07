function [n] = trajectorySectionGetLoadFactor(traj_section, varargin)
% trajectorySectionGetLoadFactor returns the velocity from a trajectory
%   [Detailed description of the function]
%   The function returns a velocity vector from a given trjactory.
%   
% Inputs:
%   traj_section   	trajectory section struct, see trajectoryCreate         
%
%   t               dimensionless time paramter 
%                   (scalar), [0-1]
%
% Outputs:
%   n               load-factor in local geodetic coordinate system
%                   scalar, in [1].
%
% Syntax:
%   n = trajectorySectionGetVel(acc) 
%  
%
% Literature:
%   [1] Author (Year): Title, Publisher.
%   [2] Author (Year): Title, Publisher.
%
% See also: trajectorySectionGetAcc
%
% Copyright 2021 TU Braunschweig
% ************************************************************************


t = 0;

if isempty(varargin)
    t(:) = traj_section.t;
else
    t(:) = varargin{1};
end   

[acc] = trajectorySectionGetAcc(traj_section, t);


% G0 : gravitational acceleration is approximately 9.81 m/s2
G0 =[0; 0; -9.81];

% Calculate the load-factor
n = norm(acc)./norm(G0);


end

