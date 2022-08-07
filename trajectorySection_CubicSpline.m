function [a,b,c,d] = trajectorySection_CubicSpline(n,x,f)
% trajectorySection_CubicSpline computes the coefficents of a cubic spline.
%   pk (x) = a[k] +b[k](x-x[k]) +c[k](x-x[])^2 +d[k](x-x[k])^3
%   x[k]< x <x[k+1], k=1,2,3...,n
% Inputs:
%       n                nummber of spline sections (scalar) = nummber of
%                        waypoints - 1
%                       
%       x                knot (1 X N Vector)
%
%       f                support values (1 X N Vector)
%                       
%
% Outputs:
%   a,b,c,d              the coefficents of a cubic spline 
%
% Syntax: 
%   [a,b,c,d] = trajectorySection_CubicSpline(n,x,f)
%    
% 
% Literature:
%   [1]- Gisela Engeln-Mullges, Klaus Niederdrenk, Reinhard Wodicka.
% Numerik-Algorithmen.Verfahren, Beispiele, Anwendungen, Auflage 10.
%   [2]- Kai Simon, Vorverarbeitung und Merkmalsextraktionin der
% Online-Handschrifterkennung, Studienarbeit, Uni Freiburg.
%   [3]- Florian Holzapfel,Nichtlineare adaptive Regelung eines unbemannten
% Fluggerätes,Lehrstuhl für Flugmechanik und Flugregelung Technische
% Universität München.
%
%
%
% Copyright 2021 TU Braunschweig
% ************************************************************************
% Eingangsparameter:
% n Umfang der Referenz, n+1 Punkte, n Segmente mit n>=2
% x Stützstellen x[1].....x[n+1]
% f Stützwerte
% -------------------------------------------------------------------------



a(n+1)=0;
for k=1:n
    h(k)= x(k+1)-x(k);
    a(k)= f(k);
end

e(1)=0;
for k=2:n
    e(k)=3*((f(k+1)-f(k))/h(k)-((f(k)-f(k-1))/h(k-1)));
end

al(1)=1;
al(2)=2*(h(1)+h(2));
g(2)=e(2);
for k=3:n
    al(k)= 2*(h(k-1)+h(k))-(h(k-1)^2)/(al(k-1));
    g(k)= e(k)-(h(k-1)*g(k-1))/(al(k-1));
end
al(n+1)=1;
g(n+1)=0;
c(n+1)=0;
c(1)=0;

for k=n:-1:2
    c(k)=(g(k)-h(k)*c(k+1))/(al(k));
end
b(n+1)=0;
d(n+1)=0;
for k=1:n
    b(k)=(f(k+1)-f(k))/h(k) - (2*c(k)+c(k+1))*h(k)/3;
    d(k)= (c(k+1)-c(k))/(3*h(k));
end

a(end) = [];
b(end) = [];
c(end) = [];
d(end) = [];
 

end