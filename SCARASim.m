function varargout = SCARASim(varargin)
% SCARASIM MATLAB code for SCARASim.fig
%      SCARASIM, by itself, creates a new SCARASIM or raises the existing
%      singleton*.
%
%      H = SCARASIM returns the handle to a new SCARASIM or the handle to
%      the existing singleton*.
%
%      SCARASIM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SCARASIM.M with the given input arguments.
%
%      SCARASIM('Property','Value',...) creates a new SCARASIM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SCARASim_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SCARASim_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SCARASim

% Last Modified by GUIDE v2.5 05-Dec-2023 09:35:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SCARASim_OpeningFcn, ...
                   'gui_OutputFcn',  @SCARASim_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before SCARASim is made visible.
function SCARASim_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SCARASim (see VARARGIN)

set(handles.btn_Theta1_qva,'BackgroundColor',[0 1 1]);
set(handles.btn_Theta2_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_d3_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_Theta4_qva,'BackgroundColor',[0.502 0.502 0.502]);

set(handles.qva_Theta1_panel,'Visible','on');
set(handles.qva_Theta2_panel,'Visible','off');
set(handles.qva_d3_panel,'Visible','off');
set(handles.qva_Theta4_panel,'Visible','off');

global plot_pos;

plot_pos = [];

% Choose default command line output for SCARASim
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SCARASim wait for user response (see UIRESUME)
SetRobotVariables()
DrawRobot(handles, 22, 22);

% --- Outputs from this function are returned to the command line.
function varargout = SCARASim_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_2 as text
%        str2double(get(hObject,'String')) returns contents of Theta_2 as a double


% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_1 as text
%        str2double(get(hObject,'String')) returns contents of Theta_1 as a double


% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_4 as text
%        str2double(get(hObject,'String')) returns contents of Theta_4 as a double


% --- Executes during object creation, after setting all properties.
function Theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d3_Callback(hObject, eventdata, handles)
% hObject    handle to d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d3 as text
%        str2double(get(hObject,'String')) returns contents of d3 as a double


% --- Executes during object creation, after setting all properties.
function d3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in Setbutton.
function Setbutton_Callback(hObject, eventdata, handles)
global a d alpha theta working_limit pos orien;
% hObject    handle to Setbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
old_theta1 = theta(1);
old_theta2 = theta(2);
old_d3 = d(3);
old_theta4 = theta(4);

theta(1) = str2double(get(handles.Theta_1,'String'));
theta(2) = str2double(get(handles.Theta_2,'String'));
d(3) = str2double(get(handles.d3,'String'));
theta(4) = str2double(get(handles.Theta_4,'String'));

[~, ~, ok] = ForwardKinematic(alpha, a, theta, d, working_limit);
if ok ~= 0
    theta1_diff = theta(1) - old_theta1;
    theta2_diff = theta(2) - old_theta2;
    theta4_diff = theta(4) - old_theta4;
    d3_diff = d(3) - old_d3;

    alpha_temp = alpha;
    a_temp = a;
    theta_temp = theta;
    d_temp = d;

    for t = 1:1:9
        theta(1) = old_theta1 + t*theta1_diff/10;
        theta(2) = old_theta2 + t*theta2_diff/10;
        d(3) = old_d3 + t*d3_diff/10;
        theta(4) = old_theta4 + t*theta4_diff/10;
        [pos, orien, ~] = ForwardKinematic(alpha, a, theta, d, working_limit);
        DrawRobot(handles, 22, 22);
        pause(0.2);
    end

    alpha = alpha_temp;
    a = a_temp;
    theta = theta_temp;
    d = d_temp;
    [pos, orien, ~] = ForwardKinematic(alpha, a, theta, d, working_limit);
    DrawRobot(handles, 22, 22);
else
    disp('out of range')
    theta(1) = old_theta1;
    theta(2) = old_theta2;
    d(3) = old_d3;
    theta(4) = old_theta4;
    [pos, orien, ~] = ForwardKinematic(alpha, a, theta, d, working_limit);
    DrawRobot(handles, 22, 22);
end

% --- Executes during object creation, after setting all properties.
function pos_ori_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_ori_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in Home_btn.
function Home_btn_Callback(hObject, eventdata, handles)
% hObject    handle to Home_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta a d alpha working_limit pos orien
old_theta1 = theta(1);
old_theta2 = theta(2);
old_d3 = d(3);
old_theta4 = theta(4);

SetRobotVariables()

theta1_diff = theta(1) - old_theta1;
theta2_diff = theta(2) - old_theta2;
theta4_diff = theta(4) - old_theta4;
d3_diff = d(3) - old_d3;

alpha_temp = alpha;
a_temp = a;
theta_temp = theta;
d_temp = d;

for t = 1:1:9
    theta(1) = old_theta1 + t*theta1_diff/10;
    theta(2) = old_theta2 + t*theta2_diff/10;
    d(3) = old_d3 + t*d3_diff/10;
    theta(4) = old_theta4 + t*theta4_diff/10;
    [pos, orien] = ForwardKinematic(alpha, a, theta, d, working_limit);
    DrawRobot(handles, 22, 22);
    pause(0.2);
end

alpha = alpha_temp;
a = a_temp;
theta = theta_temp;
d = d_temp;
[pos, orien] = ForwardKinematic(alpha, a, theta, d, working_limit);
DrawRobot(handles, 22, 22);


% --- Executes on mouse press over axes background.
function robot_sim_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to robot_sim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rotate3d(hObject, 'on');



function x_Callback(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x as text
%        str2double(get(hObject,'String')) returns contents of x as a double


% --- Executes during object creation, after setting all properties.
function x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_Callback(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y as text
%        str2double(get(hObject,'String')) returns contents of y as a double


% --- Executes during object creation, after setting all properties.
function y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_Callback(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z as text
%        str2double(get(hObject,'String')) returns contents of z as a double


% --- Executes during object creation, after setting all properties.
function z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw_Callback(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw as text
%        str2double(get(hObject,'String')) returns contents of yaw as a double


% --- Executes during object creation, after setting all properties.
function yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Setbutton_Inverse.
function Setbutton_Inverse_Callback(hObject, eventdata, handles)
global x y z yaw alpha a theta d working_limit pos orien;
% hObject    handle to Setbutton_Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% x_old = x;
% y_old = y;
% z_old = z;
% yaw_old = yaw;
x_old = pos(4,1);
y_old = pos(4,2);
z_old = pos(4,3);
yaw_old = orien(4,3);

x_new = str2double(get(handles.x,'String'));
y_new = str2double(get(handles.y,'String'));
z_new = str2double(get(handles.z,'String'));
yaw_new = str2double(get(handles.yaw,'String'))*pi/180;

x_diff = x_new - x_old;
y_diff = y_new - y_old;
z_diff = z_new - z_old;
yaw_diff = yaw_new - yaw_old;
%disp(yaw_new);
[~, ~, ~, ~, ok] = InverseKinematic(x_new,y_new,z_new,yaw_new, working_limit);
if ok ~= 0
    for t = 1:1:9
        x = x_old + t*x_diff/10;
        y = y_old + t*y_diff/10;
        z = z_old + t*z_diff/10;
        yaw = yaw_old + t*yaw_diff/10;
        [alpha, a, theta, d, ~] = InverseKinematic(x, y, z, yaw, working_limit);
        [pos, orien] = ForwardKinematic(alpha, a, theta, d, working_limit);
        DrawRobot(handles, 22, 22);
        pause(0.2);
    end
    
    x = x_new;
    y = y_new;
    z = z_new;
    yaw = yaw_new;
    [alpha, a, theta, d, ~] = InverseKinematic(x, y, z, yaw, working_limit);
    [pos, orien] = ForwardKinematic(alpha, a, theta, d, working_limit);
    DrawRobot(handles, 22, 22);
    pause(0.2);
else
    disp('out of range')
end


% --- Executes on button press in Homebutton_Inverse.
function Homebutton_Inverse_Callback(hObject, eventdata, handles)
% hObject    handle to Homebutton_Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_Theta1_qva.
function btn_Theta1_qva_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Theta1_qva (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.btn_Theta1_qva,'BackgroundColor',[0 1 1]);
set(handles.btn_Theta2_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_d3_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_Theta4_qva,'BackgroundColor',[0.502 0.502 0.502]);

set(handles.qva_Theta1_panel,'Visible','on');
set(handles.qva_Theta2_panel,'Visible','off');
set(handles.qva_d3_panel,'Visible','off');
set(handles.qva_Theta4_panel,'Visible','off');



% --- Executes on button press in btn_Theta2_qva.
function btn_Theta2_qva_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Theta2_qva (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.btn_Theta1_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_Theta2_qva,'BackgroundColor',[0 1 1]);
set(handles.btn_d3_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_Theta4_qva,'BackgroundColor',[0.502 0.502 0.502]);

set(handles.qva_Theta1_panel,'Visible','off');
set(handles.qva_Theta2_panel,'Visible','on');
set(handles.qva_d3_panel,'Visible','off');
set(handles.qva_Theta4_panel,'Visible','off');

% --- Executes on button press in btn_d3_qva.
function btn_d3_qva_Callback(hObject, eventdata, handles)
% hObject    handle to btn_d3_qva (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.btn_Theta1_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_Theta2_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_d3_qva,'BackgroundColor',[0 1 1]);
set(handles.btn_Theta4_qva,'BackgroundColor',[0.502 0.502 0.502]);

set(handles.qva_Theta1_panel,'Visible','off');
set(handles.qva_Theta2_panel,'Visible','off');
set(handles.qva_d3_panel,'Visible','on');
set(handles.qva_Theta4_panel,'Visible','off');

% --- Executes on button press in btn_Theta4_qva.
function btn_Theta4_qva_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Theta4_qva (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.btn_Theta1_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_Theta2_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_d3_qva,'BackgroundColor',[0.502 0.502 0.502]);
set(handles.btn_Theta4_qva,'BackgroundColor',[0 1 1]);

set(handles.qva_Theta1_panel,'Visible','off');
set(handles.qva_Theta2_panel,'Visible','off');
set(handles.qva_d3_panel,'Visible','off');
set(handles.qva_Theta4_panel,'Visible','on');

% --- Executes on button press in TrajectoryBtn.
function TrajectoryBtn_Callback(hObject, eventdata, handles)
global alpha a theta d working_limit pos orien;
% hObject    handle to Setbutton_Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% global t

a_max = str2double(get(handles.amax,'String'));
%v_max = 6.3;
v_max = [660 660 1120 1500];
q1 = [];
q2 = [];
q3 = [];
q4 = [];
v1 = [];
v2 = [];
v3 = [];
v4 = [];
a1 = [];
a2 = [];
a3 = [];
a4 = [];
t_c = [];
q_tc = [];
t_f = [];

x4_old = pos(4,1);
y4_old = pos(4,2);
z4_old = pos(4,3);
yaw4_old = orien(4,3);

theta_1_old = theta(1);
theta_2_old = theta(2);
d_3_old = d(3);
theta_4_old = theta(4);

x4_new = str2double(get(handles.X_trajectory,'String'));
y4_new = str2double(get(handles.Y_trajectory,'String'));
z4_new = str2double(get(handles.Z_trajectory,'String'));
yaw4_new = str2double(get(handles.Yaw_trajectory,'String'))*pi/180;

x4_diff = x4_new - x4_old;
y4_diff = y4_new - y4_old;
z4_diff = z4_new - z4_old;
yaw4_diff = yaw4_new - yaw4_old;

[alpha_new, a_new, theta_new, d_new, ok] = InverseKinematic(x4_new, y4_new, z4_new, yaw4_new, working_limit);

if ok ~= 0
%     theta_temp = theta_new;
%     d_temp = d_new;
    
    theta_1_new = theta_new(1);
    theta_2_new = theta_new(2);
    d_3_new = d_new(3);
    theta_4_new = theta_new(4);
%     [pos_new, orien_new] = ForwardKinematic(alpha_new, a_new, theta_new, d_new);
%     x_new = pos_new(1:4,1); 
%     y_new = pos_new(1:4,2);
%     z_new = pos_new(1:4,3);
%     yaw_new = orien_new(1:4,3);
    
%     for joint = 1:1:4
%         q_max(joint) = norm([x_new(joint)-x_old(joint) y_new(joint)-y_old(joint) z_new(joint)-z_old(joint)]);
%         [t_c(joint), q_tc(joint), t_f(joint)] = timeForLSPB(q_max(joint), v_max, a_max);
%     end
    q_max(1) = theta_1_new - theta_1_old; %goc theta 1
    q_max(2) = theta_2_new - theta_2_old; %goc theta 2
    q_max(3) = (d_3_new - d_3_old)*1000;         %d3
    q_max(4) = theta_4_new - theta_4_old; %goc theta 4
    disp(q_max);
    for joint = 1:1:4
        [t_c(joint), q_tc(joint), t_f(joint)] = timeForLSPB(abs(q_max(joint)), v_max(joint), a_max);
    end
    
    t_f_max = max(t_f);
    t_f_min = min(t_f);
    factor = floor(t_f_max/t_f_min) + 1;
    disp('time tf');
    disp(t_f);
%     disp('time tf max');
%     disp(t_f_max);
%     disp('factor');
%     disp(factor);
%     disp('t f min/50');
%     disp(t_f_min/50);
%     disp(t_f_min*factor);
    time = [];
%     disp('----------------------------------------------------------');
    for t = 0:t_f_min/50:(t_f_min*factor)
        time = [time, t];
%         disp('length time');
%         disp(length(time)-1);
        if t < t_f(1)
            [q_link1,v_link1,a_link1,v_max_link1] = LSPB(abs(q_max(1)), v_max(1), a_max, t_c(1), q_tc(1), t_f(1), t);
        else
            [q_link1,v_link1,a_link1,v_max_link1] = LSPB(abs(q_max(1)), v_max(1), a_max, t_c(1), q_tc(1), t_f(1), t_f(1)-0.001);
            a_link1 = 0;
        end
        q1 = [q1, q_link1];
        v1 = [v1, v_link1];
        a1 = [a1, a_link1];
        
        if t < t_f(2)
            [q_link2,v_link2,a_link2,v_max_link2] = LSPB(abs(q_max(2)), v_max(2), a_max, t_c(2), q_tc(2), t_f(2), t);
        else
            [q_link2,v_link2,a_link2,v_max_link2] = LSPB(abs(q_max(2)), v_max(2), a_max, t_c(2), q_tc(2), t_f(2), t_f(2)-0.001);
            a_link2 = 0;
        end
        q2 = [q2, q_link2];
        v2 = [v2, v_link2];
        a2 = [a2, a_link2];
        
        if t < t_f(3)
            [q_link3,v_link3,a_link3,v_max_link3] = LSPB(abs(q_max(3)), v_max(3), a_max, t_c(3), q_tc(3), t_f(3), t);
        else
            [q_link3,v_link3,a_link3,v_max_link3] = LSPB(abs(q_max(3)), v_max(3), a_max, t_c(3), q_tc(3), t_f(3), t_f(3)-0.001);
            a_link3 = 0;
        end
        q3 = [q3, q_link3];
        v3 = [v3, v_link3];
        a3 = [a3, a_link3];
            
        if t < t_f(4)
            [q_link4,v_link4,a_link4,v_max_link4] = LSPB(abs(q_max(4)), v_max(4), a_max, t_c(4), q_tc(4), t_f(4), t);
        else
            [q_link4,v_link4,a_link4,v_max_link4] = LSPB(abs(q_max(4)), v_max(4), a_max, t_c(4), q_tc(4), t_f(4), t_f(4)-0.001);
            a_link4 = 0;
        end
        q4 = [q4, q_link4];
        v4 = [v4, v_link4];
        a4 = [a4, a_link4];
%         disp('time');
%         disp(t);
%         disp('-----------------------------------------------------------------------------------------------------');
    end
    for t = 1:length(time)
        x = x4_old + x4_diff*(q1(t)+q2(t)+q3(t)+q4(t))/(abs(q_max(1))+ abs(q_max(2))+abs(q_max(3))+abs(q_max(4)));
        y = y4_old + y4_diff*(q1(t)+q2(t)+q3(t)+q4(t))/(abs(q_max(1))+ abs(q_max(2))+abs(q_max(3))+abs(q_max(4)));
        z = z4_old + z4_diff*(q1(t)+q2(t)+q3(t)+q4(t))/(abs(q_max(1))+ abs(q_max(2))+abs(q_max(3))+abs(q_max(4)));
        yaw = yaw4_old + yaw4_diff*(q1(t)+q2(t)+q3(t)+q4(t))/(abs(q_max(1))+ abs(q_max(2))+abs(q_max(3))+abs(q_max(4)));
        [alpha, a, theta, d, ~] = InverseKinematic(x, y, z, yaw, working_limit);
        [pos, orien] = ForwardKinematic(alpha, a, theta, d, working_limit);
        DrawRobot(handles, 22, 22);
        Draw_qva(handles.theta1_q, handles.theta1_v, handles.theta1_a, q1(1:t), v1(1:t), a1(1:t), time(1:t));
        Draw_qva(handles.Theta2_q, handles.Theta2_v, handles.Theta2_a, q2(1:t), v2(1:t), a2(1:t), time(1:t));
        Draw_qva(handles.d3_q, handles.d3_v, handles.d3_a, q3(1:t)/1000, v3(1:t), a3(1:t), time(1:t));
        Draw_qva(handles.Theta4_q, handles.Theta4_v, handles.Theta4_a, q4(1:t), v4(1:t), a4(1:t), time(1:t));
        pause(0.3);
    end
%     Draw_qva(handles.theta1_q, handles.theta1_v, handles.theta1_a, q1, v1, a1, time);
%     Draw_qva(handles.Theta2_q, handles.Theta2_v, handles.Theta2_a, q2, v2, a2, time);
%     Draw_qva(handles.d3_q, handles.d3_v, handles.d3_a, q3, v3, a3, time);
%     Draw_qva(handles.Theta4_q, handles.Theta4_v, handles.Theta4_a, q4, v4, a4, time);
else
    disp('out of range')
end


function X_trajectory_Callback(hObject, eventdata, handles)
% hObject    handle to X_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of X_trajectory as text
%        str2double(get(hObject,'String')) returns contents of X_trajectory as a double


% --- Executes during object creation, after setting all properties.
function X_trajectory_CreateFcn(hObject, eventdata, handles)
% hObject    handle to X_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Y_trajectory_Callback(hObject, eventdata, handles)
% hObject    handle to Y_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Y_trajectory as text
%        str2double(get(hObject,'String')) returns contents of Y_trajectory as a double


% --- Executes during object creation, after setting all properties.
function Y_trajectory_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Y_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Z_trajectory_Callback(hObject, eventdata, handles)
% hObject    handle to Z_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Z_trajectory as text
%        str2double(get(hObject,'String')) returns contents of Z_trajectory as a double


% --- Executes during object creation, after setting all properties.
function Z_trajectory_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Z_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Yaw_trajectory_Callback(hObject, eventdata, handles)
% hObject    handle to Yaw_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Yaw_trajectory as text
%        str2double(get(hObject,'String')) returns contents of Yaw_trajectory as a double


% --- Executes during object creation, after setting all properties.
function Yaw_trajectory_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Yaw_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function qva_Theta1_panel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qva_Theta1_panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function qva_Theta2_panel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qva_Theta2_panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function qva_Theta4_panel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qva_Theta4_panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function qva_d3_panel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qva_d3_panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function amax_Callback(hObject, eventdata, handles)
% hObject    handle to amax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of amax as text
%        str2double(get(hObject,'String')) returns contents of amax as a double


% --- Executes during object creation, after setting all properties.
function amax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to amax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
