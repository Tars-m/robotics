function [DH, P] = transformation_using_DH(ax, DH_0, P_0, a, alpha, d, phi, color, rotationAxis)
    % This function calculates the DH transformation and plot the
    % corresponding reference system and link from a previous DH_0 and P_0
    
    if nargin < 8
        color = 'k';
    end
    if nargin < 9
        plotRotationAxis = false;
    else
        plotRotationAxis = true;
    end
    for ii=1:length(a)
        DH_ = [...
            cos(phi(ii)) -sin(phi(ii))*cos(alpha(ii)) sin(phi(ii))*sin(alpha(ii)) a(ii)*cos(phi(ii)); ...
            sin(phi(ii)) cos(phi(ii))*cos(alpha(ii)) -cos(phi(ii))*sin(alpha(ii)) a(ii)*sin(phi(ii)); ...
            0 sin(alpha(ii)) cos(alpha(ii)) d(ii); ...
            0 0 0 1];
        
        [P, xyz] = transformReferenceSystem(DH_0 * DH_);
        if plotRotationAxis & ii < length(a)
            plotReferenceSystem(ax, P, xyz(:,1), xyz(:,2), xyz(:,3), rotationAxis{ii+1}, 'z');
        else
            plotReferenceSystem(ax, P, xyz(:,1), xyz(:,2), xyz(:,3));
        end
        plotLink(ax, P, P_0, '', color);
        DH_0 = DH_0*DH_;
        DH = DH_0;

        P_0 = P;
    end
end

