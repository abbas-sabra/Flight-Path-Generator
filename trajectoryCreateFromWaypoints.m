function [traj] = trajectoryCreateFromWaypoints(waypoints,targetVel,varargin)
% trajectoryCreateFromWaypoints creates a trajectory from waypoints.
%   The function computes the coefficents of a cubic spline for all three
%   dimensions. The velocity tangential to the path is computed as cubic
%   spline.
%   
% Inputs:
%   waypoints           waypoints in local geodetic coordinate system
%                       (3 x N vector), in m 
%   targetVel           target velocity at given waypoint
%                       (1 x N vector), in m/s
%   num_sections_max    maximum number of splines of the trajectory
%                       (scalar) = number of waypoints -1
%
% Outputs:
%   traj                trajectory struct, see trajectoryInit
%
% Syntax: 
%   [traj] = trajectoryInit(waypoints,targetVelocity) 
%   [traj] = trajectoryInit(waypoints,targetVelocity,num_sections_max) 
% 
% Literature:
%   [1] Author (Year): Title, Publisher.
%   [2] Author (Year): Title, Publisher.
%
% See also: trajectoryInit, trajectoryGetSection, trajectorySetSection
%
% Copyright 2021 TU Braunschweig
% ************************************************************************



if isempty(varargin)
    num_sections_max = size(waypoints,2)-1;
else
    num_sections_max = varargin{1};
end   

traj = trajectoryInit( num_sections_max );

% t = KnotVector (1 X N Vector),
n = size(waypoints,2);
% adjust traj struct
traj.num_sections_set = n-1;
% definition and computation of the KnotVector t (1 X N Vector),
t = zeros(1,n);
t(1)=0;
for k=2:n
    t(k)=sqrt((waypoints(1,k)-waypoints(1,k-1))^2+(waypoints(2,k)-waypoints(2,k-1))^2+(waypoints(3,k)-waypoints(3,k-1))^2) + t(k-1);
end

% compute the coefficinets of the Spline Vector,
% See trajectorySection_CubicSpline

% compute the coefficients of the Spline Vector component in  X-direction
[ax,bx,cx,dx] = trajectorySection_CubicSpline(num_sections_max,t,waypoints(1,:));
% compute the coefficients of the Spline Vector component in  Y-direction
[ay,by,cy,dy] = trajectorySection_CubicSpline(num_sections_max,t,waypoints(2,:));
% compute the coefficients of the Spline Vector component in  Z-direction
[az,bz,cz,dz] = trajectorySection_CubicSpline(num_sections_max,t,waypoints(3,:)); 

% compute the coefficients of the tangential Velocity
[a,b,c,d] = trajectorySection_CubicSpline(num_sections_max,t,targetVel);

% normalize the Spline segments
[ax,bx,cx,dx] = splineNormalize(ax,bx,cx,dx,t);
[ay,by,cy,dy] = splineNormalize(ay,by,cy,dy,t);
[az,bz,cz,dz] = splineNormalize(az,bz,cz,dz,t);
[a,b,c,d] = splineNormalize(a,b,c,d,t);


% Inserting the coefficients into the respective trajectory Spline section
% in traj struct

for k = 1: n-1
     
 traj.sections(k).pos_x.a = ax(k);
 traj.sections(k).pos_x.b = bx(k);
 traj.sections(k).pos_x.c = cx(k);
 traj.sections(k).pos_x.d = dx(k);
 traj.sections(k).pos_y.a = ay(k);
 traj.sections(k).pos_y.b = by(k);
 traj.sections(k).pos_y.c = cy(k);
 traj.sections(k).pos_y.d = dy(k);
 traj.sections(k).pos_z.a = az(k);
 traj.sections(k).pos_z.b = bz(k);
 traj.sections(k).pos_z.c = cz(k);
 traj.sections(k).pos_z.d = dz(k);
 traj.sections(k).vel.a = a(k);
 traj.sections(k).vel.b = b(k);
 traj.sections(k).vel.c = c(k);
 traj.sections(k).vel.d = d(k);
 traj.sections(k).t = 0; 
end

% compute arc length and waypoint distance
resolution = 10;
for k=1:traj.num_sections_set
    traj_section = trajectoryGetSection(traj,k);
    pos_last = trajectorySectionGetPos(traj_section,0);
    pos_first = pos_last;
    S = 0;
    for t = linspace(0,1,resolution)
        pos = trajectorySectionGetPos(traj_section,t);
        S = S + norm(pos-pos_last,2);
        pos_last = pos;
    end
     traj.sections(k).arc_length = S;
     traj.sections(k).distance = norm(pos_last-pos_first,2);
end


end

function [a,b,c,d] = splineNormalize(a,b,c,d,t)
    % normalize t: t_normalize = t-t(k) / (t(k+1)-t(k))
    b(1:end) = b(1:end) .* ( t(2:end) - t(1:end-1) );
    c(1:end) = c(1:end) .* ( t(2:end) - t(1:end-1) ).^2;
    d(1:end) = d(1:end) .* ( t(2:end) - t(1:end-1) ).^3;
end

