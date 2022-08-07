function [active_section, Error, t] = trajectoryGetMatch(traj, Aircraft_Pos, active_section,...
    backward_step_size, forward_step_size)
% trajectoryGetMatch computes the active point on the trajectory.
% % As the uav flights along the spline segments, the parameter t, that
% grows from 0 to 1 inside each segment, is a good way to detect segment 
% changes. In this way, the current segment i and its polynomials Sxi(t)
% Syi(t), Szi(t) coefficients are always known. To calculate the reference
% point (xd, yd, zd), the value of t = tm that minimizes the Euclidean 
% distance between the uav coordinates (x, y, z) and the trajectory must
% be calculated.
% 
% Inputs:
%   traj            trajectory struct, see trajectoryInit
%
%   Aircraft_Pos    aircraft position [x; y; z] in local geodetic
%                   coordinate system
%                   (3x1 vector), in m
%
%   active_section 	current section of the trajectory
%                  	(scalar), [0 - length(traj.x.a)]
%
% 
% Outputs:
%   active_section 	current section of the trajectory
%                  	(scalar), [0 - length(traj.x.a)]
%
%   Error           the muinimum Euclidean  distance between the aircraft
%                   position (x, y, z) and the trajectory
%
%   t               dimensionless time parameter of the current section
%                	(scalar), [0-1]
% 
% Syntax:
%   [active_section, Error, t] = trajectoryGetMatch(traj, pos, probably_others)
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
% Copyright TU Braunschweig 2021
% *************************************************************************


% to calculate the value of t = tm that minimizes the Euclidean 
% distance between the uav coordinates (x, y, z) and the trajectory.
% This can be calculated by the following mathematical method :
% pos = (x,y,z), p(t) = (px(t), py(t), pz(t)), p'(t) = (px'(t), py'(t), pz'(t)).
% (p(t)-pos).p'(t)=0. " scalar product" of these two vectors must be equal
% to 0.
% we obtain:
% A.*t.^5 + B.*t.^4 + C.*t.^3 + D.*t.^2 + E.*t + F = 0;
% with :

A  = zeros(1,traj.num_sections_set);
B  = zeros(1,traj.num_sections_set);
C  = zeros(1,traj.num_sections_set);
D  = zeros(1,traj.num_sections_set);
E  = zeros(1,traj.num_sections_set);
F  = zeros(1,traj.num_sections_set);
P  = zeros(traj.num_sections_set,6);
ti = -ones(5,traj.num_sections_set);

% Aircraft Positions
 x = Aircraft_Pos(1);
 y = Aircraft_Pos(2);
 z = Aircraft_Pos(3);

first_checked_section = max(1,active_section - backward_step_size); 
last_checked_section = min(traj.num_sections_set,active_section + forward_step_size);


for k = first_checked_section:last_checked_section

% The solution tm of the polynomial with tm = [0,1] must be chosen,
% and when this solution overtakes 1, k must be incremented
% to change to a new spline segment. 

traj_section = trajectoryGetSection(traj,k);

% Polynomial coefficents 
A(k) = 3.*traj_section.pos_x.d.^2 + 3.*traj_section.pos_y.d.^2 + 3.*traj_section.pos_z.d.^2;
B(k) = 5.*traj_section.pos_x.c.*traj_section.pos_x.d + 5.* traj_section.pos_y.c.* traj_section.pos_y.d +  5.* traj_section.pos_z.c.* traj_section.pos_z.d   ;
C(k) = 2.*traj_section.pos_x.c.^2 + 4.*traj_section.pos_x.b.* traj_section.pos_x.d + 2.*traj_section.pos_y.c.^2 + 4.*traj_section.pos_y.b.*traj_section.pos_y.d + 2.*traj_section.pos_z.c.^2 + 4.*traj_section.pos_z.b.*traj_section.pos_z.d;
D(k) = 3.*traj_section.pos_x.c.*traj_section.pos_x.b  - 3.*x.*traj_section.pos_x.d + 3.*traj_section.pos_x.a.*traj_section.pos_x.d + 3.*traj_section.pos_y.b.*traj_section.pos_y.c - 3.*y.*traj_section.pos_y.d + 3.*traj_section.pos_y.a.*traj_section.pos_y.d + 3.*traj_section.pos_z.c.*traj_section.pos_z.b - 3.*z.*traj_section.pos_z.d + 3.*traj_section.pos_z.a.*traj_section.pos_z.d;
E(k) = traj_section.pos_x.b.^2 - 2.*x.*traj_section.pos_x.c + 2.*traj_section.pos_x.a.*traj_section.pos_x.c + traj_section.pos_y.b.^2 - 2.*y.*traj_section.pos_y.c + 2.*traj_section.pos_y.a.*traj_section.pos_y.c + traj_section.pos_z.b.^2 - 2.*z.*traj_section.pos_z.c + 2.*traj_section.pos_z.a.*traj_section.pos_z.c;
F(k) = traj_section.pos_x.a.*traj_section.pos_x.b - x.*traj_section.pos_x.b + traj_section.pos_y.a.*traj_section.pos_y.b - y.*traj_section.pos_y.b + traj_section.pos_z.a.*traj_section.pos_z.b - z.*traj_section.pos_z.b;

% Polynomial of 5-th degree
P(k,:)  = [A(k) B(k) C(k) D(k) E(k) F(k)];

% calculate the roots
ti(:,k) = roots(P(k,:));

end

% get only real postive roots tm = [0,1]
 idx = real(ti)>=0 & real(ti)<=1 & imag(ti)==0;
 
% get the value of the roots at which the condition is satisfied.  
 T = ti(idx);
 
% find the sections that have met the condition
 [~,sections] = find(idx==true); 

% call the functions trajectoryGetError, more details in the function
% description
 [Error,w] = trajectoryGetError(traj, T, Aircraft_Pos, sections);
 
% calculate the active section
 active_section = sections(w);
 
% get the value of the root at which the Euclidean distance between
% the Aircraft and the trajectory is minimal 
 t = T(w);
  
  
end