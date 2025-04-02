clc;
close all;
clear all;
addpath('./robotics_library');

origin = [0 0 0 1]';
x_0 = [1 0 0 1]';
y_0 = [0 1 0 1]';
z_0 = [0 0 1 1]';

%% Geometric parameters and EE pose and orientation for inverse kinematic
P = [6 6 1.5 1]';
orientation = 30*pi/180;
l_5 = 2;
l_4 = 2;
l_3 = 3;
l_2 = 3;
l_1 = 1.5;

a = [l_2 l_3 l_4 l_5];
alpha = [pi/2 0 0 0];
d = [l_1 0 0 0];

fig_1 = figure;

quiver3(origin(1), origin(2), origin(3), x_0(1)-origin(1), x_0(2)-origin(2), x_0(3)-origin(3), 'r', 'LineWidth', 3)
hold on
quiver3(origin(1), origin(2), origin(3), y_0(1)-origin(1), y_0(2)-origin(2), y_0(3)-origin(3), 'g', 'LineWidth', 3)
quiver3(origin(1), origin(2), origin(3), z_0(1)-origin(1), z_0(2)-origin(2), z_0(3)-origin(3), 'b', 'LineWidth', 3)

%% Rappresentation for the EE using rotational matrix from the pose and orientation

theta_1 = atan2(P(2), P(1));

H_50 = [...
        cos(theta_1 ) -sin(theta_1 ) 0 P(1);...
        sin(theta_1 ) cos(theta_1 ) 0 P(2); ...
        0 0 1 P(3);...
        0 0 0 1];
H_50 = H_50 * [...
        1 0 0 0;...
        0 cos(pi/2) -sin(pi/2) 0;...
        0 sin(pi/2) cos(pi/2) 0;...
        0 0 0 1];
H_50 = H_50 * [...
        cos(orientation) -sin(orientation) 0 0;...
        sin(orientation) cos(orientation) 0 0; ...
        0 0 1 0;...
        0 0 0 1];

[P_5, xyz_5] = transformReferenceSystem(H_50);

plotReferenceSystem(fig_1, P_5, xyz_5(:,1), xyz_5(:,2), xyz_5(:,3));

%% D-H rappresentation of the first link

[DH_10, P_2] = transformation_using_DH(fig_1, eye(4,4), origin, l_2, pi/2, ...
    l_1,theta_1);

%% D-H rappresentation of link 3 

x_4 = xyz_5(:,1) + l_5*(P-xyz_5(:,1));
y_4 = xyz_5(:,2) + l_5*(P-xyz_5(:,1));
z_4 = xyz_5(:,3) + l_5*(P-xyz_5(:,1));
P_4 = P + l_5*(P-xyz_5(:,1));

plotLink(fig_1, P, P_4);

%% Cosin theorem to find theta_2 and theta_3
H_v = [...
        cos(-theta_1) -sin(-theta_1 ) 0 0;...
        sin(-theta_1 ) cos(-theta_1 ) 0 0; ...
        0 0 1 0;...
        0 0 0 1];
v = H_v* (P_4 - P_2);
norm(v)

theta_3 = acos((v(1)^2 + v(3)^2 - l_3^2 - l_4^2)/(2*l_3*l_4));
theta_2 = atan2(v(3),v(1)) - atan2(l_4*sin(theta_3),l_3+l_4*cos(theta_3));

[DH_20, P_3] = transformation_using_DH(fig_1, DH_10, P_2, l_3, 0, 0, theta_2);

[DH_30, P_4] = transformation_using_DH(fig_1, DH_20, P_3, l_4, 0, 0, theta_3);

norm(P_4 - P_3)
norm(P_3 - P_2)
norm(P_4(1:end -1) - P_2(1:end -1))

plot3(P(1), P(2), P(3) ,'og','LineWidth', 7)

axis equal;

%% D-H rappresentation of direct kinematics

phi = [theta_1+0.2 theta_2 theta_3 -theta_2-theta_3+orientation];
DH_0 = eye(4,4);
P_0 = [0 0 0 1]';
transformation_using_DH(fig_1, DH_0, P_0, a, alpha, d, phi);