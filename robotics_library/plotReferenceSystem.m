function plotReferenceSystem(ax, P, x, y, z, theta, rotationAxis)
    % Plots a 3D reference system and an optional rotation arc, oriented with local axes

    quiver3(ax, P(1), P(2), P(3), x(1)-P(1), x(2)-P(2), x(3)-P(3), 'r', 'LineWidth', 3);
    quiver3(ax, P(1), P(2), P(3), y(1)-P(1), y(2)-P(2), y(3)-P(3), 'g', 'LineWidth', 3);
    quiver3(ax, P(1), P(2), P(3), z(1)-P(1), z(2)-P(2), z(3)-P(3), 'b', 'LineWidth', 3);

    if nargin < 6 || isempty(rotationAxis)
        return;
    end

    ux = (x - P) / norm(x - P);
    uy = (y - P) / norm(y - P);
    uz = (z - P) / norm(z - P);

    switch lower(rotationAxis)
        case 'x'
            origin = P + 0.5 * (x - P);
            u1 = uy; u2 = uz;
        case 'y'
            origin = P + 0.5 * (y - P);
            u1 = uz; u2 = ux;
        case 'z'
            origin = P + 0.5 * (z - P);
            u1 = ux; u2 = uy;
        otherwise
            warning('Invalid rotation axis.');
            return;
    end


    r = 0.2;
    nPoints = 50;
    angle = linspace(0, 3/2*pi, nPoints);
    arc = (origin + r * (cos(angle).* u1 + sin(angle).* u2))';

    plot3(ax, arc(:,1), arc(:,2), arc(:,3), 'm', 'LineWidth', 1);

    arrowStart = arc(end-1, :);
    arrowEnd = arc(end, :);
    dir = arrowEnd - arrowStart;
    
    quiver3(ax, arrowStart(1), arrowStart(2), arrowStart(3), ...
            dir(1), dir(2), dir(3), 2, 'm', 'LineWidth', 1, 'MaxHeadSize', 1);

    angle = linspace(3/2*pi, 2*pi, nPoints);
    arc = (origin + r * (cos(angle).* u1 + sin(angle).* u2))';
    midArc = arc(round(nPoints/2), :);
    text(ax, midArc(1), midArc(2), midArc(3), ...
         theta, ...
         'FontSize', 11, 'Color', 'm', 'HorizontalAlignment', 'center');
end