% Nummber of waypoints
num_waypoints = 40;

for k=0:num_waypoints
load('waypoints')
load('targetVel')

num_waypoints; 
idx = round(linspace(1,size(waypoints,2),num_waypoints)); 
waypoints = waypoints(:,idx); 
targetVel = targetVel(:,idx);

% conflict detection
[sections,points] = trajectory_conflict_detection(waypoints,targetVel);

% Waypoints elimination
waypoints(:,sections(1)-k:sections(end)+1)=[];
targetVel(:,sections(1)-k:sections(end)+1)=[];

% conflict detection after the elimination process
[~,newpoints] = trajectory_conflict_detection(waypoints,targetVel);
if (isempty(newpoints))
    break;
end
end

traj = trajectoryCreateFromWaypoints(waypoints,targetVel);

trajectoryPlot( traj );

%load('Aircraft_Pos')

backward_step_size = 0;
forward_step_size = 6;

active_section = 1;
t = 0;
for i = 1:size(Aircraft_Pos,2)
    pos_ac = Aircraft_Pos(:,i); 
    % matching    
    [active_section, Error, t] = trajectoryGetMatch(traj, pos_ac, active_section,...
        backward_step_size, forward_step_size);
    
    if ~(isempty(t)) % matched positions were found
        
    % compute matched position
    traj_section = trajectoryGetSection(traj,active_section);
    pos_matched = trajectorySectionGetPos(traj_section,t);
    
    % visualize
    figure(3);
    plot3(pos_matched(1),pos_matched(2),pos_matched(3),'bx')
    hold on
    plot3([pos_ac(1),pos_matched(1)],[pos_ac(2),pos_matched(2)],[pos_ac(3),pos_matched(3)],'m-')
    hold on
    plot3(pos_ac(1),pos_ac(2),pos_ac(3),'ro')
    grid on
    xlabel('x in m')
    ylabel('y in m')
    zlabel('z in m')
    legend('Lotfußpunkt','Bahnabstand','Flugzeugposition', 'Location', 'northeast')
    
    else % no matched positions were found
        
        warning('tracking not possible !, no matched postions found')
        % ----> take off ? safe flight mode ? increase the range ?
        
        break;
    end 
      
end