function [p_1,p_2]=outerTangentPoints(c1,c2,r1,r2,circle)
% InPut: 
%   c1: center of first circle
%   c2: center of second circle
%   r1: radius of first circle
%   r2: radius of second circle
%   circle: determine which circle (if 1 then circle 1 tangent points)
% 
% OutPut:
%   p_1, p_2: coordinates of 2 external tangent points on 1 circle

d = norm([c2(1)-c1(1), c2(2)-c1(2)]);
CenterLineUnitVector = [c2(1)-c1(1), c2(2)-c1(2)] / d;
PerpendicularUnitVector = [c1(2)-c2(2), c2(1)-c1(1)] / d;
theta = acos((r1-r2)/d);

if circle == 1
    p_1 = c1 + r1*cos(theta)*CenterLineUnitVector + r1*sin(theta)*PerpendicularUnitVector;
    p_2 = c1 + r1*cos(theta)*CenterLineUnitVector - r1*sin(theta)*PerpendicularUnitVector;
else
    p_1 = c2 + r1*cos(theta)*CenterLineUnitVector + r1*sin(theta)*PerpendicularUnitVector;
    p_2 = c2 + r1*cos(theta)*CenterLineUnitVector - r1*sin(theta)*PerpendicularUnitVector;
end
