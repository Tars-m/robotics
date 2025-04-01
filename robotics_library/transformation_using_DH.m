% robotics_lib.m
function [DH, P] = transformation_using_DH(figureHandle, DH_0, P_0, a, alpha, d, phi)
    DH = [...
        cos(phi) -sin(phi)*cos(alpha) sin(phi)*sin(alpha) a*cos(phi); ...
        sin(phi) cos(phi)*cos(alpha) -cos(phi)*sin(alpha) a*sin(phi); ...
        0 sin(alpha), cos(alpha), d; ...
        0 0 0 1];

    x_0 = [1 0 0 1]';
    y_0 = [0 1 0 1]';
    z_0 = [0 0 1 1]';
    origin = [0 0 0 1]';

    xyz = DH_0 * DH * [x_0, y_0, z_0];
    P = DH_0 * DH * origin;
    plotReferenceSystem(figureHandle, P, xyz(:,1), xyz(:,2), xyz(:,3));
    plotLink(figureHandle, P, P_0);
end

function plotReferenceSystem(figureHandle, P, x, y, z)
    % This function plots the reference system given the origin and the 
    % axes vectors in the specified figure window.

    figure(figureHandle);
    
    quiver3(P(1), P(2), P(3), x(1)-P(1), x(2)-P(2), x(3)-P(3), 'r', 'LineWidth', 3)
    quiver3(P(1), P(2), P(3), y(1)-P(1), y(2)-P(2), y(3)-P(3), 'g', 'LineWidth', 3)
    quiver3(P(1), P(2), P(3), z(1)-P(1), z(2)-P(2), z(3)-P(3), 'b', 'LineWidth', 3)

end

function plotLink(figureHandle, P_a, P_b)
    % This function plots link between two points.

    figure(figureHandle);

    plot3([P_a(1)  P_b(1)], [P_a(2)  P_b(2)], [P_a(3)  P_b(3)], 'k-.','LineWidth', 4)
end