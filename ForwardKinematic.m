function [p_robot, o_robot, ok] = ForwardKinematic(alpha_deg, a, theta_deg, d, working_limit)
    global pos orien;
    alpha = alpha_deg*pi/180;
    theta = theta_deg*pi/180;

    theta1_max = working_limit(1);
    theta2_max = working_limit(2);
    d3_max = working_limit(3);

    if (abs(theta(1))*180/pi>theta1_max)||(abs(theta(2))*180/pi>theta2_max)||(d(3)<-d3_max)
        ok = 0;
        p_robot = pos;
        o_robot = orien;
        return
    end

    %% Ham tinh dong hoc thuan cua robot
    % Input: DH Parameter
    % Output: joint position p1 p2 p3 p4     (x y z)
    %         joint orientation o1 o2 o3 o4  (roll pitch yaw)
    %% FK Matrix
    A0_1 = Link_matrix(a(1),alpha(1),d(1),theta(1)) ;
    A1_2 = Link_matrix(a(2),alpha(2),d(2),theta(2)) ;
    A2_3 = Link_matrix(a(3),alpha(3),d(3),theta(3)) ;
    A3_4 = Link_matrix(a(4),alpha(4),d(4),theta(4)) ;

    A0_2=A0_1*A1_2;
    A0_3=A0_1*A1_2*A2_3;
    A0_4=A0_1*A1_2*A2_3*A3_4;  

    p0 = [0;0;0];
    [p1, o1] = cal_pose(A0_1,p0);
    [p2, o2] = cal_pose(A0_2,p0);
    [p3, o3] = cal_pose(A0_3,p0);
    [p4, o4] = cal_pose(A0_4,p0);
    ok = 1;
    p_robot = [p1 p2 p3 p4]';
    o_robot = [o1; o2; o3; o4];
end