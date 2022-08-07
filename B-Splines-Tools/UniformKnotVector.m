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

function [t, Range] = UniformKnotVector(k,n,periodic,normalized)

% ------------------------------------------------------------------------
% Calculate knot sequence.
% k: Degree of B-Spline
% n: Number of control points (P0, P1, ... , Pn) - 1
% t = [t0, ... , t_n+k+1] : Knot sequence / vector
% Range : Nummber of Knots
% periodic = 1 : closed uniform B-Splines
% periodic = 0 : Open uniform B-Splines 
% normalized = 1 : Normalize the KnotVector
% normalized = 0 : do not normalize the KnotVector
% -------------------------------------------------------------------------
 
t = zeros(1,n+k+1);
if (periodic == 1)
    Range=n+k;
    t=0:Range;
else 
    Range=n-k+2;
    for i=k+1:n+1
        t(i)=i-k;
    end
    t(n+2:n+k+1)=Range;
end
if (normalized==1)
    t=t./Range;
end
end

