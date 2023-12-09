function [P,O] = cal_pose(A,p0)
%% Position
    p_extended  = [p0;1];
    P_temp =  A*p_extended;
    P = P_temp(1:3);
    
%% Orientation
    o = tform2eul(A,'ZYX');
    O(1) = o(3);
    O(2) = o(2);
    O(3) = o(1);
end