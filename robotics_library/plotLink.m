function plotLink(ax, P_a, P_b, d, color)
    % Plots a link (line) between two 3D points and optionally labels translation and sets color

    if nargin < 5
        color = 'k';  % default color
    end
    if nargin < 4
        d = '';       % no text label
    end

    % Plot the line
    plot3(ax, [P_a(1) P_b(1)], [P_a(2) P_b(2)], [P_a(3) P_b(3)], ...
          'Color', color, 'LineWidth', 3);

    % Optionally plot text if d is not empty
    if ~isempty(d)
        midPoint = (P_a + P_b) / 2;
        offset = 0.1 * normalize(P_b - P_a); % offset along link direction
        text(ax, midPoint(1) + offset(1), midPoint(2) + offset(2), midPoint(3) + offset(3), ...
             d, 'FontSize', 15, 'Color', 'm', 'HorizontalAlignment', 'center');
    end
end