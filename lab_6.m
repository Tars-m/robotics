clc;
close all;
clear all;
addpath('./robotics_library');

l = 10;
h = 6;
r_1 = 0.5;
r_2 = 0.5;
P_1 = [-1 -r_1]';
P_1_c = [-1 0]';
P_2 = [l+1, -r_2]';
P_2_c = [l+1, 0]';

A = [0 0]';
B = [l 0]';
C = [l -h]';
D = [0 -h]';
k = [A B C D];

%% Direct kinematics

figure(1)
h = fill(k(1,:), k(2,:), 'b');
h.FaceAlpha = 0.1;
axis equal;
hold on;

origin = [0 0]';
x_0 = [1 0 ]';
y_0 = [0 1]';

quiver(origin(1), origin(2), x_0(1)-origin(1), x_0(2)-origin(2), 'r', 'LineWidth', 3)
quiver(origin(1), origin(2), y_0(1)-origin(1), y_0(2)-origin(2), 'g', 'LineWidth', 3)

theta = linspace(0, 2*pi, 30);
plot(P_1(1) + r_1 * cos(theta), P_1(2) + r_1 * sin(theta), 'b-', 'LineWidth', 1);
plot(P_2(1) + r_2 * cos(theta), P_2(2) + r_2 * sin(theta), 'b-', 'LineWidth', 1);

k_1 = norm(origin' - P_1_c');
k_2 = norm(B' - P_2_c') + l;

q = [660 -30]*pi/180;
l_1 = r_1 * q(1)
l_2 = r_2 * q(2) + l

text(P_1(1), P_1(2)+r_1*2, num2str(q(1)*180/pi,'%.f°'), 'FontSize', 8, 'Color', 'red');
text(P_2(1), P_2(2)+r_2*2, num2str(q(2)*180/pi,'%.f°'), 'FontSize', 8, 'Color', 'red');
plot([P_1_c(1)  A(1)], [P_1_c(2)  A(2)], 'k-.','LineWidth', 2)
plot([P_2_c(1)  B(1)], [P_2_c(2)  B(2)], 'k-.','LineWidth', 2)

theta_1 = acos((l_1^2 + B(1)^2 - l_2^2)/(2*l_1*B(1)));

P = [l_1*cos(-theta_1) l_1*sin(-theta_1)]';

plot([P(1)  B(1)], [P(2)  B(2)], 'k-.','LineWidth', 2)
plot([P(1)  A(1)], [P(2)  A(2)], 'k-.','LineWidth', 2)
plot(P(1), P(2) ,'og','LineWidth', 5);
xlim([-4 14])


%% Inverse kinematics
figure(2)
h = fill(k(1,:), k(2,:), 'b');
h.FaceAlpha = 0.1;
axis equal;
hold on;

quiver(origin(1), origin(2), x_0(1)-origin(1), x_0(2)-origin(2), 'r', 'LineWidth', 3)
quiver(origin(1), origin(2), y_0(1)-origin(1), y_0(2)-origin(2), 'g', 'LineWidth', 3)

plot([P_1_c(1)  A(1)], [P_1_c(2)  A(2)], 'k-.','LineWidth', 2)
plot([P_2_c(1)  B(1)], [P_2_c(2)  B(2)], 'k-.','LineWidth', 2)
plot(P_1(1) + r_1 * cos(theta), P_1(2) + r_1 * sin(theta), 'b-', 'LineWidth', 1);
plot(P_2(1) + r_2 * cos(theta), P_2(2) + r_2 * sin(theta), 'b-', 'LineWidth', 1);

l_1 = norm(A-P);
l_2 = norm(B-P);

q = [l_1/r_1, (l_2-l)/r_2];
text(P_1(1), P_1(2)+r_1*2, num2str(q(1)*180/pi,'%.f°'), 'FontSize', 8, 'Color', 'red');
text(P_2(1), P_2(2)+r_2*2, num2str(q(2)*180/pi,'%.f°'), 'FontSize', 8, 'Color', 'red');
plot([P(1)  B(1)], [P(2)  B(2)], 'k-.','LineWidth', 2)
plot([P(1)  A(1)], [P(2)  A(2)], 'k-.','LineWidth', 2)
plot(P(1), P(2) ,'og','LineWidth', 5);
xlim([-4 14])
