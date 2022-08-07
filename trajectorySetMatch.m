function traj = trajectorySetMatch(traj,pos,probably_others)
% trajectorySetMatch sets the active point on the trajectory.
%   The current trajectory section as well as the dimensionless time
%   parameter of that section are set.
%
% Inputs:
%   traj         trajectory struct, see trajectoryInit
%   pos          vehicle position [x; y; z] in local geodetic
%                coordinate system
%                (3x1 vector), in m
%   probably_others     are other parameters required?
%
% Outputs:
%   traj         trajectory struct, see trajectoryInit
%
% Syntax:
%   traj = trajectorySetMatch(traj,pos,probably_others)
%
% See also: trajectoryGetMatch, trajectoryInit
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

[ active_section, t ] = trajectoryGetMatch( traj, pos, probably_others );

traj.active_section = active_section;
traj.current_time = t;

end

