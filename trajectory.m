function [theta_lin, theta, xy, t_dis, t] = trajectory(t_max,a_max)
    t = linspace(0, t_max, 1000);
    
    a_1 = 1;
    a_2 = 1;
    
    x_m = [0.6, 0.6, 0, -0.6, -0.6];
    y_m = [0, 1.3, 0.9, 1.3, 0];
    xy = [x_m;y_m];
    t_dis = [0, t_max*0.2, t_max*0.5, t_max*0.8, t_max];
    
    theta_1 = zeros(1,length(t_dis));
    theta_2 = zeros(1,length(t_dis));
    
    for ii = 1:length(t_dis)
        [theta_1(ii),theta_2(ii)] = inverse_k([x_m(ii), y_m(ii)], a_1, a_2);
    end
    theta = [theta_1;theta_2];

    theta_1_dd_1 = sign(theta_1(2)-theta_1(1))*a_max;
    theta_2_dd_1 = sign(theta_2(2)-theta_2(1))*a_max;
    rad_1 = 2*(theta_1(2)-theta_1(1))/theta_1_dd_1;
    rad_2 = 2*(theta_2(2)-theta_2(1))/theta_2_dd_1;
    delta_t_1 = (t_dis(2)-t_dis(1))-sqrt((t_dis(2)-t_dis(1))^2 - rad_1);
    delta_t_2 = (t_dis(2)-t_dis(1))-sqrt((t_dis(2)-t_dis(1))^2 - rad_2);
    t_dis(1) = max(delta_t_1,delta_t_2)/2;

    theta_1_dd_n = sign(theta_1(end-1)-theta_1(end))*a_max;
    theta_2_dd_n = sign(theta_2(end-1)-theta_2(end))*a_max;
    rad_1 = 2*(theta_1(end)-theta_1(end-1))/theta_1_dd_n;
    rad_2 = 2*(theta_2(end)-theta_2(end-1))/theta_2_dd_n;
    delta_t_1 = (t_dis(end)-t_dis(end-1))-sqrt((t_dis(end)-t_dis(end-1))^2 - rad_1);
    delta_t_2 = (t_dis(end)-t_dis(end-1))-sqrt((t_dis(end)-t_dis(end-1))^2 - rad_2);
    t_dis(end) = t_dis(end)+min(delta_t_1,delta_t_2)/2;

    
    theta_1_d_k = zeros(1,length(t_dis)+1);
    theta_1_d_k(1) = 0;
    theta_1_d_k(end) = 0;
    theta_1_a0_k = zeros(1,length(t_dis)-1);
    theta_1_a1_k = zeros(1,length(t_dis)-1);
    
    theta_2_d_k = zeros(1,length(t_dis)+1);
    theta_2_d_k(1) = 0;
    theta_2_d_k(end) = 0;
    theta_2_a0_k = zeros(1,length(t_dis)-1);
    theta_2_a1_k = zeros(1,length(t_dis)-1);
    
    for ii = 2:length(theta_1_d_k)-1
        theta_1_d_k(ii)= (theta_1(ii)-theta_1(ii-1))/(t_dis(ii)-t_dis(ii-1));
        theta_2_d_k(ii)= (theta_2(ii)-theta_2(ii-1))/(t_dis(ii)-t_dis(ii-1));
    end
    

    for ii = 1:length(theta_1_a0_k)
        theta_1_a0_k(ii) = theta_1(ii);
        theta_1_a1_k(ii) = theta_1_d_k(ii+1);
        theta_2_a0_k(ii) = theta_2(ii);
        theta_2_a1_k(ii) = theta_2_d_k(ii+1);
    end
    
    theta_1_dd_k = zeros(1,length(t_dis));
    theta_2_dd_k = zeros(1,length(t_dis));
    
    theta_1_delta_k = zeros(1,length(t_dis));
    theta_2_delta_k = zeros(1,length(t_dis));
    
    theta_1_b0_k = zeros(1,length(t_dis));
    theta_1_b1_k = zeros(1,length(t_dis));
    theta_1_b2_k = zeros(1,length(t_dis));
    theta_2_b0_k = zeros(1,length(t_dis));
    theta_2_b1_k = zeros(1,length(t_dis));
    theta_2_b2_k = zeros(1,length(t_dis));
    
    for ii=1:length(theta_1_dd_k)
        theta_1_dd_k(ii) = sign(theta_1_d_k(ii+1)-theta_1_d_k(ii))*a_max;
        theta_2_dd_k(ii) = sign(theta_2_d_k(ii+1)-theta_2_d_k(ii))*a_max;
        theta_1_delta_k(ii) = (theta_1_d_k(ii+1)-theta_1_d_k(ii))/theta_1_dd_k(ii);
        theta_2_delta_k(ii) = (theta_2_d_k(ii+1)-theta_2_d_k(ii))/theta_2_dd_k(ii);
        % theta_1_delta_k(1)= delta_t_1;
    
        theta_1_b1_k(ii) = (theta_1_d_k(ii+1)+theta_1_d_k(ii))/2;
        theta_1_b2_k(ii) = theta_1_dd_k(ii)/2;
        theta_1_b0_k(ii) = theta_1(ii) + (theta_1_d_k(ii+1)-theta_1_d_k(ii))*theta_1_delta_k(ii)/8;
        theta_2_b1_k(ii) = (theta_2_d_k(ii+1)+theta_2_d_k(ii))/2;
        theta_2_b2_k(ii) = theta_2_dd_k(ii)/2;
        theta_2_b0_k(ii) = theta_2(ii) + (theta_2_d_k(ii+1)-theta_2_d_k(ii))*theta_2_delta_k(ii)/8;
    end
    
    
    theta_1_lin = zeros(1,length(t));
    theta_2_lin = zeros(1,length(t));
    
    kk = 1;
    for ii=1:length(t)
        if kk+1 <=length(t_dis)
            if t(ii) > t_dis(kk+1)-theta_1_delta_k(kk+1)/2
                kk = kk+1;
            end
        end
        if kk+1 <=length(t_dis)
            if t(ii) < t_dis(kk+1)-theta_1_delta_k(kk+1)/2 && ...
                    t(ii) > t_dis(kk)+theta_1_delta_k(kk)/2
                theta_1_lin(ii) =theta_1_a1_k(kk)*(t(ii)-t_dis(kk)) + theta_1_a0_k(kk);
            end
        end
        if t(ii) < t_dis(kk)+theta_1_delta_k(kk)/2 && ...
                t(ii) >= t_dis(kk)-theta_1_delta_k(kk)/2
            theta_1_lin(ii) = theta_1_b2_k(kk)*(t(ii)-t_dis(kk))^2 + ...
                                theta_1_b1_k(kk)*(t(ii)-t_dis(kk)) + theta_1_b0_k(kk);
        end
        if kk==1 && t(ii) < t_dis(kk)-theta_1_delta_k(kk)/2
            theta_1_lin(ii) = theta_1(kk);
        end
        if kk==length(t_dis) && t(ii) > t_dis(kk)+theta_1_delta_k(kk)/2
            theta_1_lin(ii) = theta_1(kk);
        end
    end
    
    kk = 1;
    for ii=1:length(t)
        if kk+1 <=length(t_dis)
            if t(ii) > t_dis(kk+1)-theta_2_delta_k(kk+1)/2
                kk = kk+1;
            end
        end
        if kk+1 <=length(t_dis)
            if t(ii) < t_dis(kk+1)-theta_2_delta_k(kk+1)/2 && ...
                    t(ii) > t_dis(kk)+theta_2_delta_k(kk)/2
                theta_2_lin(ii) =theta_2_a1_k(kk)*(t(ii)-t_dis(kk)) + theta_2_a0_k(kk);
            end
        end
        if t(ii) < t_dis(kk)+theta_2_delta_k(kk)/2 && ...
                t(ii) >= t_dis(kk)-theta_2_delta_k(kk)/2
            theta_2_lin(ii) = theta_2_b2_k(kk)*(t(ii)-t_dis(kk))^2 + ...
                                theta_2_b1_k(kk)*(t(ii)-t_dis(kk)) + theta_2_b0_k(kk);
        end
        if kk==1 && t(ii) < t_dis(kk)-theta_2_delta_k(kk)/2
            theta_2_lin(ii) = theta_2(kk);
        end
        if kk==length(t_dis) && t(ii) > t_dis(kk)+theta_2_delta_k(kk)/2
            theta_2_lin(ii) = theta_2(kk);
        end
    end
    theta_lin = [theta_1_lin; theta_2_lin];

end


function [theta_1, theta_2] = inverse_k(r, a_1, a_2)
    x = r(1);
    y = r(2);
    D = (x^2 + y^2 - a_1^2 - a_2^2) / (2 * a_1 * a_2);
    theta_2 = acos(D);
    theta_1 = atan2(y, x) - atan2(a_2 * sin(theta_2), a_1 + a_2 * cos(theta_2));
end
