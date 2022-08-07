function traj = trajectorySetSection( traj, traj_section, varargin )
% trajectorySectionSet writes a trajectory section struct into a trajectory
% struct.
% 
% Inputs:
%   traj            trajectory struct, see trajectoryInit
%   traj_section    trajectory section struct, see trajectorySectionInit
%   section_idx 	index of the section to be set
% 
% Outputs:
%   traj_section    trajectory section struct, see trajectorySectionInit
%
% Syntax: 
%   traj = trajectorySetSection( traj, traj_section )
%   traj = trajectorySetSection( traj, traj_section, section_idx )
%
% See also: trajectoryInit, trajectorySectionSet
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

section_idx = 0;

if isempty(varargin)
    % increment
    if traj.num_sections_set < traj.num_sections_max
        traj.num_sections_set = traj.num_sections_set + 1;
    else
        warning('Buffer size reached. Trajectory section was not set.');
    end
    section_idx(:) = traj.num_sections_set;
else
    section_idx(:) = varargin{1};
    if section_idx > traj.num_sections_set || section_idx < 1
        warning('Invalid section index. Trajectory section was not set.');
        return;
    end
end

traj.sections(section_idx) = traj_section;

end

