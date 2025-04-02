function [DH, P] = transformation_using_DH(figureHandle, DH_0, P_0, a, alpha, d, phi)
    % This function calculates the DH transformation and plot the
    % corresponding reference system and link from a previous DH_0 and P_0
    
    for i=1:length(a)
        DH_ = [...
            cos(phi(i)) -sin(phi(i))*cos(alpha(i)) sin(phi(i))*sin(alpha(i)) a(i)*cos(phi(i)); ...
            sin(phi(i)) cos(phi(i))*cos(alpha(i)) -cos(phi(i))*sin(alpha(i)) a(i)*sin(phi(i)); ...
            0 sin(alpha(i)) cos(alpha(i)) d(i); ...
            0 0 0 1];
        
        [P, xyz] = transformReferenceSystem(DH_0 * DH_);
        plotReferenceSystem(figureHandle, P, xyz(:,1), xyz(:,2), xyz(:,3));
        plotLink(figureHandle, P, P_0);
        DH_0 = DH_0*DH_;
        DH = DH_0;
        P_0 = P;
    end
end

