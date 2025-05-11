function T = homogeneousTransformSingleAxis(axis, theta, v)
    % Computes the 4x4 homogeneous transformation matrix

    if nargin < 3
        v = [0 0 0];
    end
    
    switch axis
        case 'x'
            R = [1, 0, 0;
                 0, cos(theta), -sin(theta);
                 0, sin(theta), cos(theta)];
        case 'y'
            R = [cos(theta), 0, sin(theta);
                 0, 1, 0;
                 -sin(theta), 0, cos(theta)];
        case 'z'
            R = [cos(theta), -sin(theta), 0;
                 sin(theta), cos(theta), 0;
                 0, 0, 1];
        otherwise
            R = eye(3,3);
    end

    T = [R, v(:);
         0, 0, 0, 1];

end