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

% Last Modified by GUIDE v2.5 05-Nov-2023 22:21:57

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

%global myScara;
global plot_pos;

%myScara = SCARA(0.2,0.3,0.179,125,145,0.15);
plot_pos = [];

% Choose default command line output for SCARASim
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SCARASim wait for user response (see UIRESUME)
% uiwait(handles.figure1);
% UpdateRobot(myScara,handles,22,22); 
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
        pause(0.5);
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
    theta(1) = old_theta1 + theta1_diff/10;
    theta(2) = old_theta2 + theta2_diff/10;
    d(3) = old_d3 + d3_diff/10;
    theta(4) = old_theta4 + theta4_diff/10;
    [pos, orien] = ForwardKinematic(alpha, a, theta, d, working_limit);
    DrawRobot(handles, 22, 22);
    pause(1);
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
