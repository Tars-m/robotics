function [P, xyz] = transformReferenceSystem(H)
    % This function calculates the position and the axes after the
    % transformation described by H

    x_0 = [1 0 0 1]';
    y_0 = [0 1 0 1]';
    z_0 = [0 0 1 1]';
    origin = [0 0 0 1]';
    xyz = H * [x_0, y_0, z_0];
    P = H * origin;
end

