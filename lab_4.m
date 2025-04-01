clc;
close all;

origin = [0 0 0 1]';

P = [5 2 5 1]';
l_5 = 2;
l_4 = 1;
l_3 = 2;
l_2 = 2;
l_1 = 1.5;

x_0 = [1 0 0 1]';
y_0 = [0 1 0 1]';
z_0 = [0 0 1 1]';

alpha = -30*pi/180;

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
        cos(alpha) -sin(alpha) 0 0;...
        sin(alpha) cos(alpha) 0 0; ...
        0 0 1 0;...
        0 0 0 1];

x_5 = H_50 * x_0;
y_5 = H_50 * y_0;
z_5 = H_50 * z_0;

DH_10 = [...
        cos(theta_1) -sin(theta_1)*cos(pi/2) sin(theta_1)*sin(pi/2) l_2*cos(theta_1); ...
        sin(theta_1) cos(theta_1)*cos(pi/2) -cos(theta_1)*sin(pi/2) l_2*sin(theta_1); ...
        0 sin(pi/2), cos(pi/2), l_1; ...
        0 0 0 1];

x_1 = DH_10 * x_0;
y_1 = DH_10 * y_0;
z_1 = DH_10 * z_0;
P_2 = DH_10 *origin;
P_1 = z_0*l_1;

x_4 = x_5 + l_5*(P-x_5);
y_4 = y_5 + l_5*(P-x_5);
z_4 = z_5 + l_5*(P-x_5);
P_4 = P + l_5*(P-x_5);

figure(2);
quiver3(origin(1), origin(2), origin(3), x_0(1)-origin(1), x_0(2)-origin(2), x_0(3)-origin(3), 'r', 'LineWidth', 3)
hold on
quiver3(origin(1), origin(2), origin(3), y_0(1)-origin(1), y_0(2)-origin(2), y_0(3)-origin(3), 'g', 'LineWidth', 3)
quiver3(origin(1), origin(2), origin(3), z_0(1)-origin(1), z_0(2)-origin(2), z_0(3)-origin(3), 'b', 'LineWidth', 3)

plot3(P(1), P(2), P(3) ,'og','LineWidth', 5)

quiver3(P_2(1), P_2(2), P_2(3), x_1(1)-P_2(1), x_1(2)-P_2(2), x_1(3)-P_2(3), 'r', 'LineWidth', 3)
quiver3(P_2(1), P_2(2), P_2(3), y_1(1)-P_2(1), y_1(2)-P_2(2), y_1(3)-P_2(3), 'g', 'LineWidth', 3)
quiver3(P_2(1), P_2(2), P_2(3), z_1(1)-P_2(1), z_1(2)-P_2(2), z_1(3)-P_2(3), 'b', 'LineWidth', 3)

plot3([origin(1)  P_1(1)], [origin(2)  P_1(2)], [origin(3)  P_1(3)], 'k-.','LineWidth', 4)
plot3([P_2(1)  P_1(1)], [P_2(2)  P_1(2)], [P_2(3)  P_1(3)], 'k-.','LineWidth', 4)
plot3([P(1)  P_4(1)], [P(2)  P_4(2)], [P(3)  P_4(3)], 'k-.','LineWidth', 4)

quiver3(P(1), P(2), P(3), x_5(1)-P(1), x_5(2)-P(2), x_5(3)-P(3), 'r', 'LineWidth', 3)
quiver3(P(1), P(2), P(3), y_5(1)-P(1), y_5(2)-P(2), y_5(3)-P(3), 'g', 'LineWidth', 3)
quiver3(P(1), P(2), P(3), z_5(1)-P(1), z_5(2)-P(2), z_5(3)-P(3), 'b', 'LineWidth', 3)


quiver3(P_4(1), P_4(2), P_4(3), x_4(1)-P_4(1), x_4(2)-P_4(2), x_4(3)-P_4(3), 'r', 'LineWidth', 3)
quiver3(P_4(1), P_4(2), P_4(3), y_4(1)-P_4(1), y_4(2)-P_4(2), y_4(3)-P_4(3), 'g', 'LineWidth', 3)
quiver3(P_4(1), P_4(2), P_4(3), z_4(1)-P_4(1), z_4(2)-P_4(2), z_4(3)-P_4(3), 'b', 'LineWidth', 3)

axis equal;
