clc;
clear all;
close all;

origin = [0 0 0 1]';
origin_c =[4 1 2 1]'; % canvas origin


x_0 = [1 0 0 1]';
y_0 = [0 1 0 1]';
z_0 = [0 0 1 1]';

alpha_1 = 30 *pi/180; % rotaion of canvas reference around x
alpha_2 = 20 *pi/180; % rotaion of canvas reference around y
alpha_3 = 10 *pi/180; % rotaion of canvas reference around z

theta = -20 *pi/180; % inclination of spatola

H_10 = [...
        1 0 0 origin_c(1);...
        0 cos(alpha_1) -sin(alpha_1) origin_c(2);...
        0 sin(alpha_1) cos(alpha_1) origin_c(3);...
        0 0 0 origin_c(4)];
H_10 = H_10*[...
        cos(alpha_2) 0 sin(alpha_2) 0;...
        0 1 0 0;...
        -sin(alpha_2) 0 cos(alpha_2) 0;...
        0 0 0 1];
H_10 = H_10*[...
        cos(alpha_2) -sin(alpha_2) 0 0;...
        sin(alpha_2) cos(alpha_2) 0 0; ...
        0 0 1 0;...
        0 0 0 1];
R_43 = [...
        1 0 0 0; 
        0 cos(theta) -sin(theta) 0;...
        0 sin(theta) cos(theta) 0;...
        0 0 0 1];

% tx = rand(1, 5)*10;
% ty = rand(1, 5)*10;
tx = [8.147236863931790,9.057919370756192,...
    1.269868162935061,9.133758561390193,6.323592462254095];
ty = [0.975404049994095,2.784982188670484,...
    5.468815192049839,9.575068354342976,9.648885351992766];

x_1 = H_10 * x_0;
y_1 = H_10 * y_0;
z_1 = H_10 * z_0;

P_0 = [0 0 0 1]';
P_1 = H_10*P_0;

hold off
xy = [tx; ty];
figure(1)
fnplt(cscvn(xy));
hold on
[points, t]= fnplt(cscvn(xy));
plot(tx, ty, 'bo')

points_1 = zeros(4,length(points));
points_1(4,:)=1;

x = zeros(4,length(points));
x(4,:)=1;
x = zeros(4,length(points));

y(4,:)=1;
y = zeros(4,length(points));
y(4,:)=1;

z(4,:)=1;
z = zeros(4,length(points));
z(4,:)=1;

for i = 1:length(points)-1
    theta_1 = atan2((points(2,i) - points(2,i+1)), (points(1,i) - points(1,i+1))) + pi/2
    H_21 = [...
            1 0 0 points(1,i);...
            0 1 0 points(2,i);...
            0 0 1 0;...
            0 0 0 1];
    H_32 = [...
        cos(theta_1) -sin(theta_1) 0 0;...
        sin(theta_1) cos(theta_1) 0 0; ...
        0 0 1 0;...
        0 0 0 1];

    points_1(:,i) = H_10*H_21*[0 0 0 1]';
    x(:,i) = H_10*H_21*H_32*R_43*[1 0 0 1]';
    y(:,i) = H_10*H_21*H_32*R_43*[0 1 0 1]';
    z(:,i) = H_10*H_21*H_32*R_43*[0 0 1 1]';
end

figure(2);
quiver3(origin(1), origin(2), origin(3), x_0(1)-origin(1), x_0(2)-origin(2), x_0(3)-origin(3), 'r', 'LineWidth', 3)
hold on
quiver3(origin(1), origin(2), origin(3), y_0(1)-origin(1), y_0(2)-origin(2), y_0(3)-origin(3), 'g', 'LineWidth', 3)
quiver3(origin(1), origin(2), origin(3), z_0(1)-origin(1), z_0(2)-origin(2), z_0(3)-origin(3), 'b', 'LineWidth', 3)

quiver3(P_1(1), P_1(2), P_1(3), x_1(1)-P_1(1), x_1(2)-P_1(2), x_1(3)-P_1(3), 'r', 'LineWidth', 3)
quiver3(P_1(1), P_1(2), P_1(3), y_1(1)-P_1(1), y_1(2)-P_1(2), y_1(3)-P_1(3), 'g', 'LineWidth', 3)
quiver3(P_1(1), P_1(2), P_1(3), z_1(1)-P_1(1), z_1(2)-P_1(2), z_1(3)-P_1(3), 'b', 'LineWidth', 3)

plot3(points_1(1,1:end-1), points_1(2,1:end-1), points_1(3,1:end-1) ,'-r','LineWidth', 2)

q = [0 0 0 1]';
w = [10 0 0 1]';
e = [10 10 0 1]';
r = [0 10 0 1]';
t = [0 0 0 1]';
k = [ H_10*q H_10*w H_10*e H_10*r H_10*t] 

h = fill3(k(1,:), k(2,:), k(3,:), 'b'); % plot in light blue
h.FaceAlpha = 0.1; % canvas transparency to 0.1
axis equal;

% Initialize quiver objects for red, green, and blue quivers
q_red = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'r', 'LineWidth', 2);
q_green = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'g', 'LineWidth', 2);
q_blue = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'b', 'LineWidth', 2);

% Loop through the points and update the quiver plot every 0.5 seconds
for i = 1:3:length(points_1)
    % Calculate the differences for each quiver
    u_red = x(1, i) - points_1(1, i);
    v_red = x(2, i) - points_1(2, i);
    w_red = x(3, i) - points_1(3, i);
    
    u_green = y(1, i) - points_1(1, i);
    v_green = y(2, i) - points_1(2, i);
    w_green = y(3, i) - points_1(3, i);
    
    u_blue = z(1, i) - points_1(1, i);
    v_blue = z(2, i) - points_1(2, i);
    w_blue = z(3, i) - points_1(3, i);

    % Update the quiver data incrementally
    set(q_red, 'XData', points_1(1, i), 'YData', points_1(2, i), 'ZData', points_1(3, i), ...
                'UData', u_red, 'VData', v_red, 'WData', w_red);
    set(q_green, 'XData', points_1(1, i), 'YData', points_1(2, i), 'ZData', points_1(3, i), ...
                  'UData', u_green, 'VData', v_green, 'WData', w_green);
    set(q_blue, 'XData', points_1(1, i), 'YData', points_1(2, i), 'ZData', points_1(3, i), ...
                 'UData', u_blue, 'VData', v_blue, 'WData', w_blue);

    % Pause for 0.5 seconds before updating the next set of quivers
    pause(0.1);
end

quiver3(points_1(1,1:3:end), points_1(2,1:3:end),points_1(3,1:3:end), x(1,1:3:end)-points_1(1,1:3:end), x(2,1:3:end)-points_1(2,1:3:end), x(3,1:3:end)-points_1(3,1:3:end), 'r', 'LineWidth', 2)
quiver3(points_1(1,1:3:end), points_1(2,1:3:end),points_1(3,1:3:end), y(1,1:3:end)-points_1(1,1:3:end), y(2,1:3:end)-points_1(2,1:3:end), y(3,1:3:end)-points_1(3,1:3:end), 'g', 'LineWidth', 2)
quiver3(points_1(1,1:3:end), points_1(2,1:3:end),points_1(3,1:3:end), z(1,1:3:end)-points_1(1,1:3:end), z(2,1:3:end)-points_1(2,1:3:end), z(3,1:3:end)-points_1(3,1:3:end), 'b', 'LineWidth', 2)
