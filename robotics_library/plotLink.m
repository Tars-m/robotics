function plotLink(figureHandle, P_a, P_b)
    % This function plots link between two points.

    figure(figureHandle);
    plot3([P_a(1)  P_b(1)], [P_a(2)  P_b(2)], [P_a(3)  P_b(3)], 'k-.','LineWidth', 4)
    axis equal;
end