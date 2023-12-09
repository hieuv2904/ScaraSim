function DrawRobot(handles,Az,El)

global plot_pos a d alpha theta working_limit pos orien;

set(handles.pos_ori_table,'Data',[pos orien*180/pi]);

robot_sim = handles.robot_sim;
cla(robot_sim,'reset') 
hold(robot_sim,'on')
grid(robot_sim,'on')

if ~isempty(plot_pos)
    plot3(robot_sim,plot_pos(:,1),plot_pos(:,2),plot_pos(:,3),'b','LineWidth',1.5);
end

p0 = [0 0 0];
p1 = pos(1,:);
p2 = pos(2,:);
p3 = pos(3,:);
p4 = pos(4,:);
o1 = orien(1,:);
o2 = orien(2,:);
o3 = orien(3,:);
o4 = orien(4,:);

% define links
line1=[[p0(1) p1(1)];[p0(2) p1(2)];[p0(3) p1(3)]];
line2=[[p1(1) p2(1)];[p1(2) p2(2)];[p1(3) p2(3)]];
line3=[[p2(1) p3(1)];[p2(2) p3(2)];[p2(3) p3(3)]];
line4=[[p3(1) p4(1)];[p3(2) p4(2)];[p3(3) p4(3)]];

xlabel(robot_sim,'x');
ylabel(robot_sim,'y');
zlabel(robot_sim,'z');

%draw ground
t = (1/16:1/32:1)'*2*pi;
x = (a(1)+a(2))*cos(t);
y = (a(1)+a(2))*sin(t);
z = zeros(length(t),1);
fill3(robot_sim,x,y,z,'g','FaceAlpha',0.25);

%% Ve base
VeHop(handles,0,0,0,0.21,0.18,0.179,[0.7 0.7 0.7])
VeHinhTru(handles,line1(1,1),line1(2,1),line1(3,2),0.04,0.072,[0 0.4470 0.7410])
rotate3d on;
hold on
 
%% link 1
VeHinhTru(handles,line1(1,1),line1(2,1),line1(3,2),0.06,0.01,[0.7 0.7 0.7]);
VeHinhTru(handles,line1(1,1),line1(2,1),line1(3,2)+0.072,0.06,-0.01,[0.7 0.7 0.7]);
VeHinhTru(handles,line1(1,2),line1(2,2),line1(3,2)+0.072,0.06,-0.01,[0.7 0.7 0.7]);
VeHinhTru(handles,line1(1,2),line1(2,2),line1(3,2),0.06,0.01,[0.7 0.7 0.7]);

VeHinhTru(handles,line1(1,1),line1(2,1),line1(3,2),0.06,0.062,[0.7 0.7 0.7]);
VeHinhTru(handles,line1(1,2),line1(2,2),line1(3,2),0.06,0.062,[0.7 0.7 0.7]);

[pp1,pp2]=outerTangentPoints([line1(1,1) line1(2,1)],[line1(1,2) line1(2,2)],0.06,0.06,1); %base
[pp3,pp4]=outerTangentPoints([line1(1,1) line1(2,1)],[line1(1,2) line1(2,2)],0.06,0.06,2); %joint axes 1

fill3(robot_sim,[pp1(1) pp3(1) pp4(1) pp2(1)],[pp1(2) pp3(2) pp4(2) pp2(2)],[line1(3,2) line1(3,2) line1(3,2) line1(3,2)],[0.7 0.7 0.7],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp1(1) pp3(1) pp4(1) pp2(1)],[pp1(2) pp3(2) pp4(2) pp2(2)],[line1(3,2)+0.072 line1(3,2)+0.072 line1(3,2)+0.072 line1(3,2)+0.072],[0.7 0.7 0.7],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp1(1) pp3(1) pp3(1) pp1(1)],[pp1(2) pp3(2) pp3(2) pp1(2)],[line1(3,2) line1(3,2) line1(3,2)+0.072 line1(3,2)+0.072],[0.7 0.7 0.7],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp2(1) pp4(1) pp4(1) pp2(1)],[pp2(2) pp4(2) pp4(2) pp2(2)],[line1(3,2) line1(3,2) line1(3,2)+0.072 line1(3,2)+0.072],[0.7 0.7 0.7], 'FaceAlpha', 0.2)
fill3(robot_sim,[pp1(1) pp2(1) pp2(1) pp1(1)],[pp1(2) pp2(2) pp2(2) pp1(2)],[line1(3,2) line1(3,2) line1(3,2)+0.072 line1(3,2)+0.072],[0.7 0.7 0.7],'FaceAlpha', 0.2)
fill3(robot_sim,[pp3(1) pp4(1) pp4(1) pp3(1)],[pp3(2) pp4(2) pp4(2) pp3(2)],[line1(3,2) line1(3,2) line1(3,2)+0.072 line1(3,2)+0.072],[0.7 0.7 0.7],'FaceAlpha', 0.2)

%% link2
VeHinhTru(handles,line2(1,1),line2(2,1),line1(3,2),0.04,0.072,[0 0.4470 0.7410]);%truc 1
VeHinhTru(handles,line2(1,1),line2(2,1),line1(3,2)+0.2935,0.06,-0.01,[0.7 0.7 0.7]);
VeHinhTru(handles,line2(1,1),line2(2,1),line1(3,2)+0.072,0.06,0.221,[0.7 0.7 0.7])
% %  
[pp1,pp2]=outerTangentPoints([line2(1,1) line2(2,1)],[line2(1,2) line2(2,2)],0.06,0.06,1);
[pp3,pp4]=outerTangentPoints([line2(1,1) line2(2,1)],[line2(1,2) line2(2,2)],0.06,0.06,2);
% %  
fill3(robot_sim,[pp1(1) pp3(1) pp4(1) pp2(1)],[pp1(2) pp3(2) pp4(2) pp2(2)],[line1(3,2)+0.072 line1(3,2)+0.072 line1(3,2)+0.072 line1(3,2)+0.072],[0.3010 0.7450 0.9330],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp1(1) pp3(1) pp4(1) pp2(1)],[pp1(2) pp3(2) pp4(2) pp2(2)],[line1(3,2)+0.18275 line1(3,2)+0.18275 line1(3,2)+0.18275 line1(3,2)+0.18275],[0.3010 0.7450 0.9330],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp1(1) pp3(1) pp3(1) pp1(1)],[pp1(2) pp3(2) pp3(2) pp1(2)],[line1(3,2)+0.072 line1(3,2)+0.072 line1(3,2)+0.18275 line1(3,2)+0.18275],[0.3010 0.7450 0.9330],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp2(1) pp4(1) pp4(1) pp2(1)],[pp2(2) pp4(2) pp4(2) pp2(2)],[line1(3,2)+0.072 line1(3,2)+0.072 line1(3,2)+0.18275 line1(3,2)+0.18275],[0.3010 0.7450 0.9330],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp1(1) pp2(1) pp2(1) pp1(1)],[pp1(2) pp2(2) pp2(2) pp1(2)],[line1(3,2)+0.072 line1(3,2)+0.072 line1(3,2)+0.18275 line1(3,2)+0.18275],[0.3010 0.7450 0.9330],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp3(1) pp4(1) pp4(1) pp3(1)],[pp3(2) pp4(2) pp4(2) pp3(2)],[line1(3,2)+0.072 line1(3,2)+0.072 line1(3,2)+0.18275 line1(3,2)+0.18275],[0.3010 0.7450 0.9330],'FaceAlpha' ,0.2)
% % 
pp5 = (pp1+pp3)/2;
pp6 = (pp2+pp4)/2;
% % 
fill3(robot_sim,[pp1(1) pp5(1) pp5(1) pp1(1)],[pp1(2) pp5(2) pp5(2) pp1(2)],[line1(3,2)+0.18275 line1(3,2)+0.18275 line1(3,2)+0.2935 line1(3,2)+0.2935],[0.7 0.7 0.7],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp2(1) pp6(1) pp6(1) pp2(1)],[pp2(2) pp6(2) pp6(2) pp2(2)],[line1(3,2)+0.18275 line1(3,2)+0.18275 line1(3,2)+0.2935 line1(3,2)+0.2935],[0.7 0.7 0.7],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp1(1) pp5(1) pp6(1) pp2(1)],[pp1(2) pp5(2) pp6(2) pp2(2)],[line1(3,2)+0.2935 line1(3,2)+0.2935 line1(3,2)+0.2935 line1(3,2)+0.2935],[0.7 0.7 0.7],'FaceAlpha' ,0.2)
fill3(robot_sim,[pp5(1) pp3(1) pp5(1)],[pp5(2) pp3(2) pp5(2)],[line1(3,2)+0.18275 line1(3,2)+0.18275 line1(3,2)+0.2935],[0.7 0.7 0.7],'FaceAlpha' ,0.2);
fill3(robot_sim,[pp6(1) pp4(1) pp6(1)],[pp6(2) pp4(2) pp6(2)],[line1(3,2)+0.18275 line1(3,2)+0.18275 line1(3,2)+0.2935],[0.7 0.7 0.7],'FaceAlpha' ,0.2);
fill3(robot_sim,[pp5(1) pp3(1) pp4(1) pp6(1)],[pp5(2) pp3(2) pp4(2) pp6(2)],[line1(3,2)+0.2935 line1(3,2)+0.18275 line1(3,2)+0.18275 line1(3,2)+0.2935],[0.7 0.7 0.7],'FaceAlpha' ,0.2)

%% link3
VeHinhTru(handles,line2(1,2),line2(2,2),line1(3,2)+0.072,0.06,-0.018,[0.3010 0.7450 0.9330])
VeHinhTru(handles,line2(1,2),line2(2,2),line1(3,2)+0.054,0.03,-0.036,[0.3010 0.7450 0.9330])
VeHinhTru(handles,line3(1,2),line3(2,2),line3(3,2),0.02,0.5,[0 0 0]);
VeHinhTru(handles,line3(1,2),line3(2,2),line3(3,2)+0.5,0.035,0.01,[0 0 0]);
VeHinhTru(handles,line3(1,2),line3(2,2),line3(3,2),0.035,0.01,[0 0 0]);
VeHinhTru(handles,line2(1,2),line2(2,2),line1(3,2)+0.072,0.06,0.2215,[0.7 0.7 0.7])

%% link 4
VeHinhTru(handles,line4(1,2),line4(2,2),line4(3,2),0.02,-0.02,[0.4660 0.6740 0.1880])

%% vẽ trục tọa độ
%%% Line 1
xUnitVectorLine1 = [0.1*cos(o1(3)), 0.1*sin(o1(3))] ;
yUnitVectorLine1 = [-xUnitVectorLine1(2), xUnitVectorLine1(1)];
quiver3(line1(1,1), line1(2,1), line1(3,2), xUnitVectorLine1(1), xUnitVectorLine1(2), 0, 'r', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1)+0.1, line1(2,1), line1(3,2)+0.1, 'x', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
quiver3(line1(1,1), line1(2,1), line1(3,2), yUnitVectorLine1(1), yUnitVectorLine1(2), 0, 'b', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1), line1(2,1)+0.1, line1(3,2)+0.1, 'y', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
quiver3(line1(1,1), line1(2,1), line1(3,2), 0, 0, 0.1, 'g', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1), line1(2,1), line1(3,2)+0.2, 'Z', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');

%%% Line 2
xUnitVectorLine2 = [0.1*cos(o2(3)), 0.1*sin(o2(3))];
yUnitVectorLine2 = [-xUnitVectorLine2(2), xUnitVectorLine2(1)];
quiver3(line2(1,1), line2(2,1), line2(3,2), xUnitVectorLine2(1), xUnitVectorLine2(2), 0, 'r', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1)+0.1, line1(2,1), line1(3,2)+0.1, 'x', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
quiver3(line2(1,1), line2(2,1), line2(3,2), yUnitVectorLine2(1), yUnitVectorLine2(2), 0, 'b', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1), line1(2,1)+0.1, line1(3,2)+0.1, 'y', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
quiver3(line2(1,1), line2(2,1), line2(3,2), 0, 0, 0.1, 'g', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1), line1(2,1), line1(3,2)+0.2, 'Z', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');

%%%Line 3
%dLine3 = norm([line3(1,2)-line3(1,1), line3(2,2)-line3(2,1)]);
xUnitVectorLine3 = [0.1*cos(o3(3)), 0.1*sin(o3(3))];
yUnitVectorLine3 = [-xUnitVectorLine2(2), xUnitVectorLine2(1)];
quiver3(line3(1,1), line3(2,1), line3(3,2), xUnitVectorLine3(1), xUnitVectorLine3(2), 0, 'r', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1)+0.1, line1(2,1), line1(3,2)+0.1, 'x', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
quiver3(line3(1,1), line3(2,1), line3(3,2), yUnitVectorLine3(1), yUnitVectorLine3(2), 0, 'b', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1), line1(2,1)+0.1, line1(3,2)+0.1, 'y', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
quiver3(line3(1,1), line3(2,1), line3(3,2), 0, 0, 0.1, 'g', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1), line1(2,1), line1(3,2)+0.2, 'Z', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');

%%%Line 4 
xUnitVectorLine4 = [0.1*cos(o4(3)), 0.1*sin(o4(3))];
yUnitVectorLine4 = [-xUnitVectorLine4(2), xUnitVectorLine4(1)];
quiver3(line4(1,1), line4(2,1), line4(3,2), xUnitVectorLine4(1), xUnitVectorLine4(2), 0, 'r', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1)+0.1, line1(2,1), line1(3,2)+0.1, 'x', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
quiver3(line4(1,1), line4(2,1), line4(3,2), yUnitVectorLine4(1), yUnitVectorLine4(2), 0, 'b', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1), line1(2,1)+0.1, line1(3,2)+0.1, 'y', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
quiver3(line4(1,1), line4(2,1), line4(3,2), 0, 0, 0.1, 'g', 'LineWidth', 1, 'MaxHeadSize', 1);
%text(line1(1,1), line1(2,1), line1(3,2)+0.2, 'Z', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');

%% 
view(robot_sim,Az,El)
axis(robot_sim,'equal')