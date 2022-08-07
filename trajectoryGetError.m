function [Error,w] = trajectoryGetError(traj, t, Aircraft_Pos, u)
% this function computes the tracking error. this error is the muinimum 
% Euclidean  distance between the Aircraft position (x, y, z) and
% the trajectory
%
% Inputs:
%  traj             trajectory struct, see trajectoryInit
%
%  t                set of roots which satisfy the condition
%
%  Aircraft_Pos     aircraft position [x; y; z] in local geodetic
%                   coordinate system
%                   (3x1 vector), in m
%
%  u                sections with roots which satisfy the condition
%
%  t                imensionless time parameter of the current section
%                	(scalar), [0-1]
%
%   Aircraft_Pos    current uav coordinates (x,y,z),in m

% Outputs:
%   error           the minimum Euclidean  distance between the uav
%                   coordinates (x, y, z) and the trajectory, in m
%
%   w               the element of the set "t" which gives the muinimum 
%                   Euclidean  distance between the Aircraft position
%                   (x, y, z) and the trajectory
%
% Syntax:
%   [Error,w] = trajectoryGetError(active_section, tm, pos)
%   
%
% Literature:
%   [1]- Florian Holzapfel (2004):Nichtlineare adaptive Regelung eines 
% unbemannten Fluggerätes, Lehrstuhl für Flugmechanik und Flugregelung,
% Technische Universität München.
%   [2]- Rodrigo Gutiérrez, Elena López-Guillén, OrcID,Luis M. Bergasa, 
% Rafael Barea, Óscar Pérez ,Carlos Gómez-Huélamo, Felipe Arango,
% Javier del Egido and Joaquín López-Fernández (2020):
% A Waypoint Tracking Controller for Autonomous Road Vehicles Using
% ROS Framework, Electronics Department, University of Alcalá,
% Systems Engineering and Automation Department, University of Vigo.
%
% See also: trajectoryGetMatch, trajectoryCreateFromWaypoints
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

% Aircraft Position
x = Aircraft_Pos(1);
y = Aircraft_Pos(2);
z = Aircraft_Pos(3);

pf = [x;y;z];

% number of elements in set t 
m = size(t,1);

Errors = zeros(1,m);
pos = zeros(3,m);

for j=1:m
    
% compute the position of probable match points     
traj_section = trajectoryGetSection(traj, u(j));
pos(:,j) = trajectorySectionGetPos(traj_section, t(j));

% compute the tracking errors
Errors(j) = norm(pos(:,j)-pf);

end

% the minimum Euclidean  distance between the Aircraft and the trajectory
[Error]= min(Errors);

% get the  minium distance
[w] = find(Errors==min(Errors), 1, 'first');

end