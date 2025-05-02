function plotLink(ax, P_a, P_b, d)
    % Plots a link (line) between two 3D points and optionally labels translation

    plot3(ax, [P_a(1) P_b(1)], [P_a(2) P_b(2)], [P_a(3) P_b(3)], 'k', 'LineWidth', 3);

    if nargin > 3
        midPoint = (P_a + P_b) / 2;
        text(ax, midPoint(1)+0.1, midPoint(2)+ 0.1, midPoint(3)+0.1, ...
             d, ...
             'FontSize', 15, 'Color', 'm', 'HorizontalAlignment', 'center');
    end
end
