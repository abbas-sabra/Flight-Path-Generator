function traj_section = trajectoryGetSection( traj, varargin )
% trajectoryGetSection returns a trajectory section struct.
% 
% Inputs:
%   traj            trajectory struct, see trajectoryInit
%   section_idx     the index of the section to be returned
%
% Outputs:
%   traj_section    trajectory section struct, see trajectorySectionInit
%
% Syntax: 
%   traj = trajectoryGetSection( traj )
%   traj = trajectoryGetSection( traj, section_idx )
%
% See also: trajectoryInit, trajectorySectionSet
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

section_idx = 0 ;

if isempty(varargin)
    section_idx(:) = traj.active_section;
else
    section_idx(:) = varargin{1};
end

if section_idx > traj.num_sections_set || section_idx < 1
    warning('Invalid section index. First section will be returned.');
    section_idx(:) = 1;    
end

traj_section = traj.sections(section_idx);

end

