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
canv = fill(k(1,:), k(2,:), 'b');
canv.FaceAlpha = 0.1;
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

q = [600 -100]*pi/180;
l_1 = r_1 * q(1);
l_2 = r_2 * q(2) + l;

text(P_1(1), P_1(2)+r_1*2, num2str(q(1)*180/pi,'%.f°'), 'FontSize', 8, 'Color', 'red');
text(P_2(1), P_2(2)+r_2*2, num2str(q(2)*180/pi,'%.f°'), 'FontSize', 8, 'Color', 'red');
plot([P_1_c(1)  A(1)], [P_1_c(2)  A(2)], 'k-.','LineWidth', 2)
plot([P_2_c(1)  B(1)], [P_2_c(2)  B(2)], 'k-.','LineWidth', 2)

theta_1 = acos((l_1^2 + B(1)^2 - l_2^2)/(2*l_1*B(1)));

P = [l_1*cos(-theta_1) l_1*sin(-theta_1)]';

plot([P(1)  B(1)], [P(2)  B(2)], 'k-.','LineWidth', 2)
plot([P(1)  A(1)], [P(2)  A(2)], 'k-.','LineWidth', 2)
plot(P(1), P(2) ,'og','LineWidth', 5);
title('Direct kinematics');
xlim([-4 14])


%% Inverse kinematics
P = [5, -1]';
figure(2)
canv = fill(k(1,:), k(2,:), 'b');
canv.FaceAlpha = 0.1;
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
title('Inverse kinematics');
xlim([-4 14])


%% Manipulability 

F = [P(1) P(2); (P(1)-l) P(2)];
G = [r_1^2*q(1) 0; 0 r_2*(l+r_2*q(2))];
J = F\G;
[V, D] = eig(inv(J*J'));

theta = linspace(0, 2*pi, 100);
circle = [cos(theta); sin(theta)]; 

axes_lengths = sqrt(diag(D));  % Semi-axis lengths
scaled_circle = diag(axes_lengths) * circle;

ellipsoid_points = V * scaled_circle + P;

% Plot the ellipsoid
plot(ellipsoid_points(1, :), ellipsoid_points(2, :), 'LineWidth', 1);
axis equal;


%% Manipulability grid

figure(3)
A = [0 0]';
B = [l 0]';
C = [l -h]';
D = [0 -h]';
k = [A B C D];
canv = fill(k(1,:), k(2,:), 'b');
canv.FaceAlpha = 0.1;
hold on;
grid on;
[X, Y] = meshgrid(linspace(0, l, 5), linspace(-h, -1, 5));

for i = 1:length(X(1,:))
    for k = 1:length(X(:,1))
        P = [X(k,i), Y(k,i)]';
        l_1 = norm(A-P);
        l_2 = norm(B-P);
        
        q = [l_1/r_1, (l_2-l)/r_2];
        
        F = [P(1) P(2); (P(1)-l) P(2)];
        G = [r_1^2*q(1) 0; 0 r_2*(l+r_2*q(2))];
        J = F\G;
        [V, D] = eig(inv(J*J'));
        
        theta = linspace(0, 2*pi, 100);
        circle = [cos(theta); sin(theta)]; 
        
        axes_lengths = sqrt(diag(D));  % Semi-axis lengths
        scaled_circle = diag(axes_lengths) * circle;
        
        ellipsoid_points = V * scaled_circle + P;
        
        % Plot the ellipsoid
        plot(ellipsoid_points(1, :), ellipsoid_points(2, :), 'b','LineWidth', 1);
        hold on;
    end
end
xlim([-4 14])
title('Manipulability grid');



%% Elipsoid volumes

figure(4)

[X, Y] = meshgrid(linspace(0, l, 100), linspace(-h, -1, 100));
V = zeros(100,100);

for i = 1:length(X(1,:))
    for k = 1:length(X(:,1))
        P = [X(k,i), Y(k,i)]';
        l_1 = norm(A-P);
        l_2 = norm(B-P);
        
        q = [l_1/r_1, (l_2-l)/r_2];
        
        F = [P(1) P(2); (P(1)-l) P(2)];
        G = [r_1^2*q(1) 0; 0 r_2*(l+r_2*q(2))];
        J = F\G;
        V(k,i) = det(inv(J*J'));
       
    end
end
grid on;
surf( V)
imagesc([0, 10], [-6,-1], V)

xlim([-4 14])
ylim([-8, 2])
title('Capability grid');
colorbar