  function [alpha_robot, a_robot, theta_robot, d_robot, ok] = InverseKinematic(x,y,z,yaw, working_limit)
    % Input: x y z yaw a1 a2
    % Output: theta1 theta2 d3 theta4
    global alpha a theta d
    
    alpha_robot = alpha;
    a_robot = a;
    theta_robot = theta;
    d_robot = d;
    
    theta1_max = working_limit(1);
    theta2_max = working_limit(2);
    d3_max = working_limit(3);

    a1 = a(1);
    a2 = a(2);
    d1 = d(1);
    c2 = (x^2 + y^2 - a1^2 - a2^2)/(2*a1*a2);
    if (abs(c2)<=1)
        s2 = sqrt(1-c2^2);
        theta21 = atan2(s2,c2);
        theta22 = atan2(-s2,c2);

        if abs(theta21 - theta(2)*pi/180) < pi
            theta2 = theta21;
        else
            theta2 = theta22;
            s2 = -s2;
        end

        t1 = [a1+a2*c2 -a2*s2;a2*s2 a1+a2*c2]^(-1)*[x;y];
        c1 = t1(1);
        s1 = t1(2);
        theta1 = atan2(s1,c1);

        d3 = z - d1;
        theta4 = yaw - ( theta1 + theta2 );

        if (abs(theta1*180/pi)>theta1_max)||(abs(theta2*180/pi)>theta2_max)||(d3<-d3_max)
            ok = 0;
            return
        else
            ok = 1;
            theta_robot(1) = theta1*180/pi;
            theta_robot(2) = theta2*180/pi;
            d_robot(3) = d3;
            theta_robot(4) = theta4*180/pi;
        end
    else
        ok = 0;
        return
    end
end