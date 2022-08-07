function [first_deriv, sec_deriv] = trajectorySectionGetDerivatives ...
    (traj_section, varargin)
% trajectorySectionGetDerivatives returns the first and second derivatives
% of a trajectory section.
%   [Detailed description of the function]
%   The function returns ....
%
% Inputs:
%   traj_section   	trajectory section struct, see trajectoryCreate         
%
%   t               dimensionless time paramter 
%                   (scalar), [0-1]
%
% Outputs:
%   first_deriv     first derivative [dxp; dyp; dzp] in local geodetic system
%                   (3x1 vector), dimensionless
%
%   sec_deriv       second derivative [ddxp; ddyp; ddzp] in local geodetic system
%                   (3x1 vector), dimensionless
%
% Syntax:
%   [first_deriv, sec_deriv] = trajectorySectionGetDerivatives(traj_section)
%   [first_deriv, sec_deriv] = trajectorySectionGetDerivatives(traj_section,t)
% 
% Literature:
%   [1] Author (Year): Title, Publisher.
%   [2] Author (Year): Title, Publisher.
%
% See also: trajectorySectionInit, trajectorySectionGetAcc, 
%   trajectorySectionGetLoadFactor
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

t = 0;

if isempty(varargin)
    t(:) = traj_section.t;
else
    t(:) = varargin{1};
end

% Calculate first derivative of path

bx = traj_section.pos_x.b;
cx = traj_section.pos_x.c;
dx = traj_section.pos_x.d;
dxp = bx + 2.*cx.*(t) + 3.*dx.*(t)^2;


by = traj_section.pos_y.b;
cy = traj_section.pos_y.c;
dy = traj_section.pos_y.d;
dyp = by + 2.*cy.*(t) + 3.*dy.*(t)^2;


bz = traj_section.pos_z.b;
cz = traj_section.pos_z.c;
dz = traj_section.pos_z.d;
dzp = bz + 2.*cz.*(t) + 3.*dz.*(t)^2;

first_deriv = [dxp; dyp; dzp];

% Calculate second derivative of path

ddxp = 2.*cx + 6.*dx.*(t);

ddyp = 2.*cy + 6.*dy.*(t);

ddzp = 2.*cz + 6.*dz.*(t);
sec_deriv = [ddxp; ddyp; ddzp];

end

