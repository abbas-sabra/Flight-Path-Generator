% ==================================================
% Technische Universität Braunschweig
% Bachelorarbeit : Generation of highly dynamic flight paths and
% trajectories from waypoints for dynamic soaring close to the ground
% ==================================================
% Mit diesem Skript wird eine Sollflugbahn aus gegebenen Wegpunkte mit
% B-Splines approximiert.
% Autor(in): Abbas Sabra-
% Erstellt am: 05.08.2021-
% -------------------------------------------------------------------------


load('waypoints')
load('targetVel')
num_waypoints = 20; 
idx = round(linspace(1,size(waypoints,2),num_waypoints)); 
waypoints = waypoints(:,idx); 
xi = waypoints(1,:);
yi = waypoints(2,:);
zi = waypoints(3,:);

n=size(xi,2)-1;
k = 4 ; periodic=0; normalized=0;
[t,Range]= UniformKnotVector(k,n,periodic,normalized);
SampleSize=65;
m=n+k+1;
x = zeros(1,(n+k)*SampleSize+1);
if (periodic==1)
    x=0:1/SampleSize:Range;
else
    x(((k-1)*SampleSize)+1:((n+1)*SampleSize+1)) = 0:1/SampleSize:Range;
    x(((n+1)*SampleSize+2):end)=Range;
end
if (normalized==1)
    x=x./Range;
end  

% -------------------------------------------------------------------------

N1=FirstOrderBSplineFunctions(t,x,k,SampleSize,periodic);
figure();
subplot(2,2,1)
plot(x,N1(1,:))
for i=1:m-1
   hold on
   plot(x,N1(i,:))
   title('Konstant')
  hold off
end

N2=SecondOrderBSplineFunctions(t,x,k,SampleSize,N1,periodic);
subplot(2,2,2)
plot(x,N2(1,:))
for i=1:m-2
   hold on
   plot(x,N2(i,:))
    title('linear')
   hold off
end

N3=ThirdOrderBSplineFunctions(t,x,k,SampleSize,N1,N2,periodic);
subplot(2,2,3)
plot(x,N3(3,:))
for i=1:m-3
   hold on
   plot(x,N3(i,:))
    title('quadratisch')
   hold off
end

N4=FourthOrderBSplineFunctions(t,x,k,SampleSize,N1,N2,N3,periodic);
 subplot(2,2,4)
plot(x,N3(3,:))
for i=1:m-4
   hold on
   plot(x,N4(i,:))
   title('kubisch')
   hold off
end


if k==1
    Ni=N1;
elseif k==2
   Ni=N2;
elseif k==3
   Ni=N3;
else
    Ni=N4;
end


Sx = xi*Ni(1:n+1,(k-1)*SampleSize+1:(n+1)*SampleSize);
Sy = yi*Ni(1:n+1,(k-1)*SampleSize+1:(n+1)*SampleSize);
Sz = zi*Ni(1:n+1,(k-1)*SampleSize+1:(n+1)*SampleSize);

figure();
plot3(Sx,Sy,Sz,'b')
title('Die Sollflugbahn aus gegebenen Wegpunkte durch B-Spline-Approx.')
hold on
plot3(xi,yi,zi,'o')
plot3(xi,yi,zi,'r')
grid on;
xlabel('x in m');
ylabel('y in m');
zlabel('z in m');
legend({'B-Spline-Approx.','Die Wegpunkte','Polygonzug'}, 'Location', 'northeast');










