function varargout = FT2D_GUI(varargin)
% FT2D_GUI MATLAB code for FT2D_NT.fig
%      FT2D_GUI, by itself, creates a new FT2D_GUI or raises the existing
%      singleton*.
%
%      H = FT2D_GUI returns the handle to a new FT2D_GUI or the handle to
%      the existing singleton*.
%
%      FT2D_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FT2D_GUI.M with the given input arguments.
%
%      FT2D_GUI('Property','Value',...) creates a new FT2D_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before FT2D_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to FT2D_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help FT2D_GUI

% Last Modified by GUIDE v2.5 24-Mar-2019 17:29:44

% Begin initialization code - DO NOT EDIT.
% if datenum(date) > datenum('12-Dec-2013')
%     return;
% end

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...                                                                                                                                                                                                                                              
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @FT2D_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @FT2D_GUI_OutputFcn, ...
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


% --- Executes just before FT2D_NT is made visible.
function FT2D_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to FT2D_GUI (see VARARGIN)
global dataGlobal;
dataGlobal.im = [];


%for selecting optimal filter
dataGlobal.sigma = 1;

dataGlobal.offset = 0;
% Choose default command line output for FT2D_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% UIWAIT makes FT2D_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = FT2D_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in PB1. (Load image)
function PB1_Callback(hObject, eventdata, handles)
% hObject    handle to PB1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Read Image file
global dataGlobal;
[FileName,PathName,~] = uigetfile('*.tif');

if FileName ~= 0
    i = find('.'==FileName);
dataGlobal.FileName = FileName(1:i-1);
    dataGlobal.imi=[PathName,FileName];
    dataGlobal.im = imread(dataGlobal.imi);
    dataGlobal.im2 = dataGlobal.im;
    dataGlobal.im = im2double(dataGlobal.im);


    %Check if im is RGB or gray
    dataGlobal.DD = size(dataGlobal.im);
    dataGlobal.DD2 = size(dataGlobal.im2);
    if numel(dataGlobal.DD) == 3
        dataGlobal.im = dataGlobal.im(:,:,2);
        dataGlobal.im2 = dataGlobal.im2(:,:,2);
    end
    dataGlobal.im = mat2gray(dataGlobal.im);
    
    %Gaussian filter the image
    if dataGlobal.sigma*6 <= 3
        fsize = [3 3];
    else
        fsize = ceil([dataGlobal.sigma*6, dataGlobal.sigma*6]);
    end
    f = fspecial('gaussian',fsize, dataGlobal.sigma);
    dataGlobal.imfil = imfilter(dataGlobal.im,f,'symmetric');
    %dataGlobal.imfil=dataGlobal.im;
    
%     dataGlobal.imt = zeros([dataGlobal.DD(1) dataGlobal.DD(2) 3]);
%     dataGlobal.imt2 = zeros([dataGlobal.DD2(1) dataGlobal.DD2(2) 3]);
%     dataGlobal.imt(:,:,2) = dataGlobal.im;
%     dataGlobal.imt2(:,:,2) = dataGlobal.im2;
    axes(handles.axes1);
    cla;
    imshow(dataGlobal.imfil)
%  Fiber orientation calculation
    [dx,dy] = gradient(dataGlobal.imfil);
    temp = ones(size(dataGlobal.imfil));
    temp = temp - padarray(ones(size(dataGlobal.imfil)-2),[1 1]);
    dx = dx.*(1-temp);
    dy = dy.*(1-temp);
 %   dx = dx(dataGlobal.imfil>threshold);
 %   dy = dy(dataGlobal.imfil>threshold);
    exclude_points = (dx == 0) & (dy == 0);
    dx(exclude_points) = [];
    dy(exclude_points) = [];
    if ~isempty(dx)
        theta = atan2(dx,dy) + dataGlobal.offset;
        theta = mod(theta,pi);
        theta = sort(theta)';
        assignin('base','theta',theta);
        %use sliding window method to calculate best mean and circular
        %variance
        steps = 90;
        theta_temp = theta*ones(1,steps);
        theta_shift = ones(length(theta),1)*linspace(0,pi-pi/steps,steps);
        theta_temp = theta_temp+(theta_temp<theta_shift)*pi;
        theta_mean = angle(sum(exp(1i*theta_temp)));
        r = abs(mean(exp(1i*theta_temp)));
        re= real(mean(exp(1i*theta_temp)));
        circ_var = 1 - r;
        %%
        theta_mean = theta_mean(circ_var == min(circ_var));
        theta_mean = theta_mean(1);
        dataGlobal.circ_var = min(circ_var);
        
        theta_mean(theta_mean < 0) = theta_mean(theta_mean < 0) + pi;
        theta_mean(theta_mean > pi) = theta_mean(theta_mean > pi) - pi;
    end
    
    %     FT test
       
    
    theta2=theta-theta_mean;
    theta2=theta2(theta2>0);
    m1=mean(cos(theta2).^2);
    dataGlobal.H=1.5*m1-0.5;

%     plot polar plot
    axes(handles.axes2);
     dataGlobal.num_bars= 360/5;
    rose([theta;theta+pi], dataGlobal.num_bars*2);
    dataGlobal.theta_mean = theta_mean/pi*180;
    set(handles.AOCV,'visible','on');
    set(handles.AOCV,'string',{sprintf('Average Orientation = %.1f', dataGlobal.theta_mean);sprintf('Circular variance = %.3f', dataGlobal.circ_var*2);sprintf('Hermans orientation parameter = %.3f', dataGlobal.H)});
    dataGlobal.theta=theta;
end
% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function IT_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
%initial value


% --- Executes during object creation, after setting all properties.
function DT_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
%initial value


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1
axis off;


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, ~)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global dataGlobal;
a = findobj('type','figure');
a = a(mod(a,1)==0);
close(a);


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear -global dataGlobal;

% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2
axis off;



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
global dataGlobal
dataGlobal.sigma = str2double(get(hObject,'String'));

if dataGlobal.sigma == 0
    dataGlobal.sigma = 1;
    set(hObject,'String','1')
end

if ~isempty(dataGlobal.im)
    if dataGlobal.sigma*6 <= 3
        fsize = [3 3];
    else
        fsize = ceil([dataGlobal.sigma*6, dataGlobal.sigma*6]);
    end
    f = fspecial('gaussian',fsize, dataGlobal.sigma);
    dataGlobal.imfil = imfilter(dataGlobal.im,f,'symmetric');
    axes(handles.axes1);
    cla;
    imshow(dataGlobal.imfil)  
%    threshold = dataGlobal.dark_th*max(dataGlobal.imfil(:));

    [dx,dy] = gradient(dataGlobal.imfil);
    temp = ones(size(dataGlobal.imfil));
    temp = temp - padarray(ones(size(dataGlobal.imfil)-2),[1 1]);
    dx = dx.*(1-temp);
    dy = dy.*(1-temp);
    exclude_points = (dx == 0) & (dy == 0);
    dx(exclude_points) = [];
    dy(exclude_points) = [];
    if ~isempty(dx)
        theta = atan2(dx,dy) + dataGlobal.offset;
        theta = mod(theta,pi);
        theta = sort(theta)';

        %use sliding window method to calculate best mean and circular
        %variance
        steps = 90;
        theta_temp = theta*ones(1,steps);
        theta_shift = ones(length(theta),1)*linspace(0,pi-pi/steps,steps);
        theta_temp = theta_temp+(theta_temp<theta_shift)*pi;

        theta_mean = angle(sum(exp(1i*theta_temp)));
        r = abs(mean(exp(1i*theta_temp)));
        circ_var = 1 - r;

        theta_mean = theta_mean(circ_var == min(circ_var));
        theta_mean = theta_mean(1);
        dataGlobal.circ_var = min(circ_var);

        theta_mean(theta_mean < 0) = theta_mean(theta_mean < 0) + pi;
        theta_mean(theta_mean > pi) = theta_mean(theta_mean > pi) - pi;
    end
        theta2=theta-theta_mean;
    theta2=theta2(theta2>0);
    m1=mean(cos(theta2).^2);
    dataGlobal.H=1.5*m1-0.5;

%     plot polar plot
    axes(handles.axes2);
    cla;
    dataGlobal.num_bars= 360/5;
    rose([theta;theta+pi], dataGlobal.num_bars*2);
    dataGlobal.theta_mean = theta_mean/pi*180;
    set(handles.AOCV,'visible','on');
    set(handles.AOCV,'string',{sprintf('Average Orientation = %.1f', dataGlobal.theta_mean);sprintf('Circular variance = %.3f', dataGlobal.circ_var*2);sprintf('Hermans orientation parameter = %.3f', dataGlobal.H)});
    dataGlobal.theta=theta;
end


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global dataGlobal
figure
rose([dataGlobal.theta;dataGlobal.theta+pi], dataGlobal.num_bars*2);
title({sprintf('Average Orientation = %.1f', dataGlobal.theta_mean);sprintf('Circular variance = %.3f', dataGlobal.circ_var*2);sprintf('Hermans orientation parameter = %.3f', dataGlobal.H)});
savefig([dataGlobal.FileName,'_roseplot.fig']);