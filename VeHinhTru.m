function VeHinhTru(handles,x0,y0,z0,r,h,colr)
% x0 y0 z0: toa do tam
% r: ban kinh day
% h: chieu cao
[X,Y,Z] = cylinder(r,100);
X = X + x0;
Y = Y + y0;
Z = Z*h + z0;
surf(handles.robot_sim,X,Y,Z,'FaceAlpha' ,0.2,'LineStyle','none');
fill3(handles.robot_sim,X(1,:),Y(1,:),Z(1,:),colr,'FaceAlpha' ,0.2)
fill3(handles.robot_sim,X(2,:),Y(2,:),Z(2,:),colr,'FaceAlpha' ,0.2)