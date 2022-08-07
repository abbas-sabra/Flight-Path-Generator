function [sections,points] = trajectory_conflict_detection(waypoints,targetVel)
% trajectory_conflict_detection detect sections and points on the
% trajectory at which the load factor exceeds the maximum Value
%   
% Inputs:
% waypoints           waypoints in local geodetic coordinate system
%                       (3 x N vector), in m 
% targetVel           target velocity at given waypoint
%                       (1 x N vector), in m/s
% Outputs:
% points               points on the trajectory at which the load factor
%                      exceeds the maximum Value
% sections             trajectory sections at which the load factor
%                      exceeds the maximum Value
%
% Syntax: 
%   [sections,points] = trajectory_conflict_detection(waypoints,targetVel) 
% 
%
% See also: trajectoryCreateFromWaypoints, trajectorySection_CubicSpline
%           trajectorySectionGetLoadFactor, trajectorySectionGetDerivatives
%           trajectorySectionGetPos, trajectorySectionGetVel,
%           trajectorySectionGetAcc
%
% Copyright 2021 TU Braunschweig
% ************************************************************************


traj = trajectoryCreateFromWaypoints(waypoints,targetVel);



% evaluate positions
resolution = 50;

n = zeros(1,traj.num_sections_set*resolution);

waypoints = zeros(3,traj.num_sections_set+1);

traj_section = trajectoryGetSection(traj,1);

waypoints(:,1) = trajectorySectionGetPos(traj_section,0);

%compute the load factor

for k=1:traj.num_sections_set
    t=linspace(0,1,resolution);
    traj_section = trajectoryGetSection(traj,k);
    for j = 1:resolution
        idx = (k-1)*resolution+j;
        n(:,idx) = trajectorySectionGetLoadFactor(traj_section, t(j));
    end
    waypoints(:,k+1) = trajectorySectionGetPos(traj_section,1);
end

% Find points and sections on the trajectory at which the load factor exceeds the
% maximum Value

y = real(n)>3;
[~,points] = find(y==true);
v = points./(resolution);
sections = floor(v);
 
end
