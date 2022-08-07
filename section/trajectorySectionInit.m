function traj_section = trajectorySectionInit()
% trajectorySectionInit defines a trajectory section struct.
%   The function initializes a struct with the coefficents of a cubic
%   spline for all three dimensions.
% 
% Inputs:
%   none
%
% Outputs:
%   traj_section    trajectory section struct
%                   where the position (sub structs: pos_x, pos_y, pos_z) 
%                   is a cubic spline in the form
%                       f(t) = a + b*t + c*t^2 + d*t^3
%                   and the velocity (sub struct: vel) is a cubic spline
%                   in the form
%                       f(t) =  a + b*t + c*t^2 + d*t^3
%                   and t is the dimensionless time paramter.
%
% Syntax: 
%   traj_section = trajectorySectionInit()
%
% See also: trajectorySectionSet, trajectorySectionGetPos
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

cubic_spline = struct( ...
    'a', 0, ...
    'b', 0, ...
    'c', 0, ...
    'd', 0 ...
    );


traj_section = struct( ...
    'pos_x', cubic_spline, ...
    'pos_y', cubic_spline, ...
    'pos_z', cubic_spline, ...
    'vel',   cubic_spline, ...
    't', 0, ...
    'arc_length', 0,...
    'distance', 0 ...
    );

end

