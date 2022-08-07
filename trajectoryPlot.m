function [path] = trajectoryPlot(traj,varargin)
% trajectoryPlot visualizes the flight path from waypoints, flight path
% velocity in m/s, flight path acceleration in m/s^2, load factor.
%   
% Inputs:
%   traj                trajectory struct, see trajectoryInit
%
% Outputs:
%   path                flight path (figure)
%
% Syntax: 
%   [path] = trajectoryPlot(traj) 
% 
% Literature:
%   [1] Author (Year): Title, Publisher.
%   [2] Author (Year): Title, Publisher.
%
% See also: trajectoryCreateFromWaypoints, trajectorySection_CubicSpline
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

% evaluate positions
resolution = 50;

pos = zeros(3,traj.num_sections_set*resolution);

first_deriv = zeros(3,traj.num_sections_set*resolution);

sec_deriv = zeros(3,traj.num_sections_set*resolution);

kappa = zeros(1,traj.num_sections_set*resolution);

vel = zeros(3,traj.num_sections_set*resolution);

v = zeros(1,traj.num_sections_set*resolution);

acc = zeros(3,traj.num_sections_set*resolution);

nacc = zeros(1,traj.num_sections_set*resolution);

n = zeros(1,traj.num_sections_set*resolution);

waypoints = zeros(3,traj.num_sections_set+1);

traj_section = trajectoryGetSection(traj,1);

waypoints(:,1) = trajectorySectionGetPos(traj_section,0);

for k=1:traj.num_sections_set
    t=linspace(0,1,resolution);
    traj_section = trajectoryGetSection(traj,k);
    for j = 1:resolution
        idx = (k-1)*resolution+j;
        pos(:,idx) = trajectorySectionGetPos(traj_section,t(j));
        [first_deriv(:,idx), sec_deriv(:,idx)] = trajectorySectionGetDerivatives...
        (traj_section, t(j));
        kappa(:,idx) = norm(cross(first_deriv(:,idx),sec_deriv(:,idx)))./(norm(first_deriv(:,idx)))^3;
        vel(:,idx) = trajectorySectionGetVel(traj_section, t(j));
        v(:,idx) = norm(vel(:,idx));
        acc(:,idx) = trajectorySectionGetAcc(traj_section, t(j));
        nacc(:,idx) = norm(acc(:,idx));
        n(:,idx) = trajectorySectionGetLoadFactor(traj_section, t(j));
    end
    waypoints(:,k+1) = trajectorySectionGetPos(traj_section,1);
end

% plot
path = figure();
title('flight path from waypoints')
subplot(2,3,1)
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'o')
hold on
plot3(pos(1,:),pos(2,:),pos(3,:),'b-')
xlabel('x in m')
ylabel('y in m')
zlabel('z in m')
legend('waypoints','Cubic-Spline-Approx', 'Location', 'northeast');
grid
axis equal

subplot(2,3,2)
distance = [0,cumsum(vecnorm(pos(:,2:end)-pos(:,1:end-1),2,1))];
plot(distance,v,'r')
grid on
xlabel('distance in m')
ylabel('flight path velocity in m/s')

subplot(2,3,3)
distance = [0,cumsum(vecnorm(pos(:,2:end)-pos(:,1:end-1),2,1))];
plot(distance,n,'g')
grid on
xlabel('distance in m')
ylabel('load factor')


subplot(2,3,4)
distance = [0,cumsum(vecnorm(pos(:,2:end)-pos(:,1:end-1),2,1))];
plot(distance,nacc,'b')
grid on
xlabel('distance in m')
ylabel('flight path acceleration in m/s^2')


subplot(2,3,5)
distance = [0,cumsum(vecnorm(pos(:,2:end)-pos(:,1:end-1),2,1))];
plot(distance,kappa,'c')
grid on
xlabel('distance in m')
ylabel('curvature')

subplot(2,3,6)
color_line3(pos(1,:),pos(2,:),pos(3,:),n);
view(15,15);
title('flight path , color = load factor');
view(15,15);
colorbar
grid on
xlabel('x in m')
ylabel('y in m')
zlabel('z in m')

end

