function traj = trajectoryInit( num_sections_max )
% trajectoryInit defines a trajectory struct.
%   The trajectory struct contains a buffer of several trajectory sections
%   which define the trajectory with splines.
%   
% Inputs:
%   num_sections_max    maximum number of splines of the trajectory
%
% Outputs:
%   traj        trajectory coefficent struct
%               with              
%               traj.num_sections_max   maximum number of sections (buffer)
%               traj.num_sections_set   number of set sections inside
%                                       buffer
%               traj.sections           (N-1 x 1) array of trajectory
%                                       section structs, see
%                                       trajectorySectionInit
%               traj.active_section     the active section (from matching)
%               traj.current_time       the current dimensionless time on
%                                       on the section (from matching)
%
% Syntax: 
%   traj = trajectoryInit(waypoints) 
%
% See also: trajectoryGetPos, trajectoryGetVel, trajectoryGetAcc
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

traj = struct( ...
    'num_sections_max', num_sections_max, ...
    'num_sections_set', 0, ...
    'sections', repmat( trajectorySectionInit(), num_sections_max, 1 ), ...
    'active_section', 0, ...
    'current_time', 0, ...
    'arc_length', 0, ...
    'distance', 0 ...
    );

end

