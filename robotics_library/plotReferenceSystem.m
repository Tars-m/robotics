function plotReferenceSystem(figureHandle, P, x, y, z)
    % This function plots the reference system given the origin and the 
    % axes vectors in the specified figure window.

    figure(figureHandle);
    quiver3(P(1), P(2), P(3), x(1)-P(1), x(2)-P(2), x(3)-P(3), 'r', 'LineWidth', 3)
    quiver3(P(1), P(2), P(3), y(1)-P(1), y(2)-P(2), y(3)-P(3), 'g', 'LineWidth', 3)
    quiver3(P(1), P(2), P(3), z(1)-P(1), z(2)-P(2), z(3)-P(3), 'b', 'LineWidth', 3)
    axis equal;
end
