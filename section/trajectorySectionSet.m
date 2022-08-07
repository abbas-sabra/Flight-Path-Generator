function traj_section = trajectorySectionSet( traj_section, ...
    pos_x_spline_coeffs, pos_y_spline_coeffs, pos_z_spline_coeffs, ...
    vel_spline_coeffs )
% trajectorySectionSet writes the coefficents of one cubic spline.
% 
% Inputs:
%   to do
%
% Outputs:
%   traj_section    trajectory section spline coefficents
%                   where the position (sub structs: pos_x, pos_y, pos_z) 
%                   is a cubic spline in the form
%                       f(t) = a*t^3 + b*t^2 + c*t + d
%                   and the velocity (sub struct: vel) is a linear spline
%                   in the form
%                       f(t) = a*t + b
%                   and t is the dimensionless time paramter.
%
% Syntax: 
%   traj_section = trajectorySectionSet( traj_section, ...
%       pos_x_spline_coeffs, pos_y_spline_coeffs, pos_z_spline_coeffs, ...
%       vel_spline_coeffs )
%
% See also: trajectorySectionInit
%
% Copyright 2021 TU Braunschweig
% ************************************************************************

traj_section.pos_x.a = pos_x_spline_coeffs(1);
traj_section.pos_x.b = pos_x_spline_coeffs(2);
traj_section.pos_x.c = pos_x_spline_coeffs(3);
traj_section.pos_x.d = pos_x_spline_coeffs(4);

traj_section.pos_y.a = pos_y_spline_coeffs(1);
traj_section.pos_y.b = pos_y_spline_coeffs(2);
traj_section.pos_y.c = pos_y_spline_coeffs(3);
traj_section.pos_y.d = pos_y_spline_coeffs(4);

traj_section.pos_z.a = pos_z_spline_coeffs(1);
traj_section.pos_z.b = pos_z_spline_coeffs(2);
traj_section.pos_z.c = pos_z_spline_coeffs(3);
traj_section.pos_z.d = pos_z_spline_coeffs(4);

traj_section.vel.a = vel_spline_coeffs(1);
traj_section.vel.b = vel_spline_coeffs(2);
traj_section.vel.c = vel_spline_coeffs(3);
traj_section.vel.d = vel_spline_coeffs(4);

end

