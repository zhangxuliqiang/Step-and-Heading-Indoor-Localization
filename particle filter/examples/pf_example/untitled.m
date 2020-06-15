function varargout = untitled(varargin)
% UNTITLED MATLAB code for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED.M with the given input arguments.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 28-Oct-2013 20:13:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
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


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitled (see VARARGIN)

% Choose default command line output for untitled
handles.output = hObject;
% Update handles structure
no_Particles=2000;
fwd_Noise=1;
turn_Noise=0.05;
sense_Noise=10;
turn_Angle=0.1;
distance=10;
grid_Size=100;
no_Iterations=10
set(handles.no_Particles,'String',num2str(no_Particles));
set(handles.fwd_Noise,'String',num2str(fwd_Noise));
set(handles.turn_Noise,'String',num2str(turn_Noise));
set(handles.sense_Noise,'String',num2str(sense_Noise));
set(handles.turn_Angle,'String',num2str(turn_Angle));
set(handles.distance,'String',num2str(distance));
set(handles.grid_Size,'String',num2str(grid_Size));
set(handles.no_Iterations,'String',num2str(no_Iterations));
guidata(hObject, handles);

% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function fwd_Noise_Callback(hObject, eventdata, handles)
% hObject    handle to fwd_Noise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fwd_Noise as text
%        str2double(get(hObject,'String')) returns contents of fwd_Noise as a double


% --- Executes during object creation, after setting all properties.
function fwd_Noise_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fwd_Noise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function turn_Noise_Callback(hObject, eventdata, handles)
% hObject    handle to turn_Noise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of turn_Noise as text
%        str2double(get(hObject,'String')) returns contents of turn_Noise as a double


% --- Executes during object creation, after setting all properties.
function turn_Noise_CreateFcn(hObject, eventdata, handles)
% hObject    handle to turn_Noise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sense_Noise_Callback(hObject, eventdata, handles)
% hObject    handle to sense_Noise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sense_Noise as text
%        str2double(get(hObject,'String')) returns contents of sense_Noise as a double


% --- Executes during object creation, after setting all properties.
function sense_Noise_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sense_Noise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function turn_Angle_Callback(hObject, eventdata, handles)
% hObject    handle to turn_Angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of turn_Angle as text
%        str2double(get(hObject,'String')) returns contents of turn_Angle as a double


% --- Executes during object creation, after setting all properties.
function turn_Angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to turn_Angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function distance_Callback(hObject, eventdata, handles)
% hObject    handle to distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of distance as text
%        str2double(get(hObject,'String')) returns contents of distance as a double


% --- Executes during object creation, after setting all properties.
function distance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function no_Particles_Callback(hObject, eventdata, handles)
% hObject    handle to no_Particles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of no_Particles as text
%        str2double(get(hObject,'String')) returns contents of no_Particles as a double


% --- Executes during object creation, after setting all properties.
function no_Particles_CreateFcn(hObject, eventdata, handles)
% hObject    handle to no_Particles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function grid_Size_Callback(hObject, eventdata, handles)
% hObject    handle to grid_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of grid_Size as text
%        str2double(get(hObject,'String')) returns contents of grid_Size as a double


% --- Executes during object creation, after setting all properties.
function grid_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to grid_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in no_Robots.
function no_Robots_Callback(hObject, eventdata, handles)
% hObject    handle to no_Robots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns no_Robots contents as cell array
%        contents{get(hObject,'Value')} returns selected item from no_Robots


% --- Executes during object creation, after setting all properties.
function no_Robots_CreateFcn(hObject, eventdata, handles)
% hObject    handle to no_Robots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function no_Iterations_Callback(hObject, eventdata, handles)
% hObject    handle to no_Iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of no_Iterations as text
%        str2double(get(hObject,'String')) returns contents of no_Iterations as a double


% --- Executes during object creation, after setting all properties.
function no_Iterations_CreateFcn(hObject, eventdata, handles)
% hObject    handle to no_Iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in run_Filter.
function run_Filter_Callback(hObject, eventdata, handles)
% hObject    handle to run_Filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
grid_Size = get(handles.grid_Size,'String');
no_Particles = get(handles.no_Particles,'String');
fwd_Noise = get(handles.fwd_Noise,'String');
turn_Noise = get(handles.turn_Noise,'String');
sense_Noise = get(handles.sense_Noise,'String');
turn_Angle = get(handles.turn_Angle,'String');
distance = get(handles.distance,'String');
no_Robots=get(handles.no_Robots,'String');
no_Iterations=get(handles.no_Iterations,'String');

fwd_Noise=str2double(fwd_Noise);
turn_Noise=str2double(turn_Noise);
sense_Noise=str2double(sense_Noise);
turn_Angle=str2double(turn_Angle);
distance=str2double(distance);
grid_Size=str2double(grid_Size);
no_Particles=str2double(no_Particles);
no_Robots=str2double(no_Robots);
no_Iterations=str2double(no_Iterations);

guidata(hObject,handles);
clear robot;
ParticleFilter(fwd_Noise,turn_Noise,sense_Noise,turn_Angle,distance,grid_Size,no_Particles,no_Robots,no_Iterations);


