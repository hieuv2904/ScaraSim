function [P,O] = cal_pose(A,p0)
%% Position
    p_extended  = [p0;1];
    P_temp =  A*p_extended;
    P = P_temp(1:3);
    
%% Orientation
    O(2)  = atan2(-A(3,1),sqrt(A(3,2)^2+A(3,3)^2));
    O(3)  = atan2(A(2,1)/cos(O(2)),A(1,1)/cos(O(2)));
    O(1)  = atan2(A(3,2)/cos(O(2)),A(3,3)/cos(O(2)));
end