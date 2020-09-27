function varargout = dwl7(varargin)
% DWL7 MATLAB code for dwl7.fig
%      DWL7 by itself, creates a new DWL7 or raises the
%      existing singleton*.
%
%      H = DWL7 returns the handle to a new DWL7 or the handle to
%      the existing singleton*.
%
%      DWL7('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DWL7.M with the given input arguments.
%
%      DWL7('Property','Value',...) creates a new DWL7 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before dwl7_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to dwl7_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help dwl7

% Last Modified by GUIDE v2.5 21-Sep-2020 12:47:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @dwl7_OpeningFcn, ...
                   'gui_OutputFcn',  @dwl7_OutputFcn, ...
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

% --- Executes just before dwl7 is made visible.
function dwl7_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to dwl7 (see VARARGIN)

% Choose default command line output for dwl7

handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = dwl7_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% The figure can be deleted now


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.output = get(hObject,'String');

% Update handles structure
guidata(hObject, handles);

% Use UIRESUME instead of delete because the OutputFcn needs
% to get the updated handles structure.
uiresume(handles.figure1);

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.output = get(hObject,'String');

% Update handles structure
guidata(hObject, handles);

% Use UIRESUME instead of delete because the OutputFcn needs
% to get the updated handles structure.
uiresume(handles.figure1);


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if isequal(get(hObject, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(hObject);
else
    % The GUI is no longer waiting, just close it
    delete(hObject);
end


% --- Executes on key press over figure1 with no controls selected.
function figure1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Check for "enter" or "escape"
if isequal(get(hObject,'CurrentKey'),'escape')
    % User said no by hitting escape
    handles.output = 'No';
    
    % Update handles structure
    guidata(hObject, handles);
    
    uiresume(handles.figure1);
end    
    
if isequal(get(hObject,'CurrentKey'),'return')
    uiresume(handles.figure1);
end    



function zhouqi_f_Callback(hObject, eventdata, handles)
% hObject    handle to zhouqi_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zhouqi_f as text
%        str2double(get(hObject,'String')) returns contents of zhouqi_f as a double


% --- Executes during object creation, after setting all properties.
function zhouqi_f_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zhouqi_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pinlv_f_Callback(hObject, eventdata, handles)
% hObject    handle to pinlv_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pinlv_f as text
%        str2double(get(hObject,'String')) returns contents of pinlv_f as a double


% --- Executes during object creation, after setting all properties.
function pinlv_f_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pinlv_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function zhouqi_t_Callback(hObject, eventdata, handles)
% hObject    handle to zhouqi_t (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zhouqi_t as text
%        str2double(get(hObject,'String')) returns contents of zhouqi_t as a double


% --- Executes during object creation, after setting all properties.
function zhouqi_t_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zhouqi_t (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pinlv_t_Callback(hObject, eventdata, handles)
% hObject    handle to pinlv_t (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pinlv_t as text
%        str2double(get(hObject,'String')) returns contents of pinlv_t as a double


% --- Executes during object creation, after setting all properties.
function pinlv_t_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pinlv_t (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fuzhi_t_Callback(hObject, eventdata, handles)
% hObject    handle to fuzhi_t (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fuzhi_t as text
%        str2double(get(hObject,'String')) returns contents of fuzhi_t as a double


% --- Executes during object creation, after setting all properties.
function fuzhi_t_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fuzhi_t (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xiangwei_t_Callback(hObject, eventdata, handles)
% hObject    handle to xiangwei_t (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xiangwei_t as text
%        str2double(get(hObject,'String')) returns contents of xiangwei_t as a double


% --- Executes during object creation, after setting all properties.
function xiangwei_t_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xiangwei_t (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function junzhi_Callback(hObject, eventdata, handles)
% hObject    handle to junzhi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of junzhi as text
%        str2double(get(hObject,'String')) returns contents of junzhi as a double


% --- Executes during object creation, after setting all properties.
function junzhi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to junzhi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function junfangzhi_Callback(hObject, eventdata, handles)
% hObject    handle to junfangzhi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of junfangzhi as text
%        str2double(get(hObject,'String')) returns contents of junfangzhi as a double


% --- Executes during object creation, after setting all properties.
function junfangzhi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to junfangzhi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fangcha_Callback(hObject, eventdata, handles)
% hObject    handle to fangcha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fangcha as text
%        str2double(get(hObject,'String')) returns contents of fangcha as a double


% --- Executes during object creation, after setting all properties.
function fangcha_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fangcha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fengzhi_Callback(hObject, eventdata, handles)
% hObject    handle to fengzhi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fengzhi as text
%        str2double(get(hObject,'String')) returns contents of fengzhi as a double


% --- Executes during object creation, after setting all properties.
function fengzhi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fengzhi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function qishidian_Callback(hObject, eventdata, handles)
% hObject    handle to qishidian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of qishidian as text
%        str2double(get(hObject,'String')) returns contents of qishidian as a double


% --- Executes during object creation, after setting all properties.
function qishidian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qishidian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function zhongzhidian_Callback(hObject, eventdata, handles)
% hObject    handle to zhongzhidian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zhongzhidian as text
%        str2double(get(hObject,'String')) returns contents of zhongzhidian as a double


% --- Executes during object creation, after setting all properties.
function zhongzhidian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zhongzhidian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in fenxisuoyoudian.
function fenxisuoyoudian_Callback(hObject, eventdata, handles)
% hObject    handle to fenxisuoyoudian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of fenxisuoyoudian
if get(hObject,'Value')==0.0
    set(handles.qishidian,'Enable','on');
    set(handles.zhongzhidian,'Enable','on');
else
    set(handles.qishidian,'String','1','Enable','off');
    set(handles.zhongzhidian,'String',get(handles.n,'String'),'Enable','off');
end% Hint: get(hObject,'Value') returns toggle state of checkbox2



function fs_Callback(hObject, eventdata, handles)
% hObject    handle to fs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fs as text
%        str2double(get(hObject,'String')) returns contents of fs as a double


% --- Executes during object creation, after setting all properties.
function fs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function n_Callback(hObject, eventdata, handles)
% hObject    handle to n (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of n as text
%        str2double(get(hObject,'String')) returns contents of n as a double


% --- Executes during object creation, after setting all properties.
function n_CreateFcn(hObject, eventdata, handles)
% hObject    handle to n (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function luyinshijian_Callback(hObject, eventdata, handles)
% hObject    handle to luyinshijian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of luyinshijian as text
%        str2double(get(hObject,'String')) returns contents of luyinshijian as a double


% --- Executes during object creation, after setting all properties.
function luyinshijian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to luyinshijian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in kaishiluyin.
function kaishiluyin_Callback(hObject, eventdata, handles)
% hObject    handle to kaishiluyin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Fs=str2double(get(findobj('Tag','fs'),'String'));
a=str2double(get(findobj('Tag','luyinshijian'),'String'));
R = audiorecorder;
%直接调用audiorecorder函数，参数FS默认8000hz，8位，单声道，不改变时不用写出
disp('开始录音')
recordblocking(R,a);
disp('录音结束')
%录音时间
handles.xxx = getaudiodata(R);
handles.shuruboxing=1;
guidata(hObject,handles);
luyinsize=size(handles.xxx);
set(handles.n,'String',num2str(luyinsize(1)));
[filename,pathname]=uiputfile({'*.wav';'*,*'},'Save as','/luyin')
audiowrite([pathname,filename],handles.xxx,8000);
%手动保存录音文件，保存为wav格式，推荐保存在matlab路径下
Length=size(handles.xxx);
plot(handles.axes6,handles.xxx);
%更新数据
guidata(hObject,handles);


function wav_Callback(hObject, eventdata, handles)
% hObject    handle to wav (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of wav as text
%        str2double(get(hObject,'String')) returns contents of wav as a double


% --- Executes during object creation, after setting all properties.
function wav_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wav (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function pinyufenxi_Callback(hObject, eventdata, handles)
% hObject    handle to pinyufenxi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Fs=str2double(get(findobj('Tag','fs'),'String'));
N=str2double(get(findobj('Tag','n'),'String'));
%读取采样频率和采样点数
if handles.shuruboxing==0
        msgbox('无波形， 请重新输入');
        return;
end
zhongzhidian=str2double(get(handles.qishidian,'String'));
qishidian=str2double(get(handles.zhongzhidian,'String'));
%读取起始点与终止点
a=(qishidian-zhongzhidian+1)/2;
f=linspace(0,Fs/2,(qishidian-zhongzhidian+1)/2);
yangben=handles.xxx(zhongzhidian:qishidian);
Y=fft(yangben,qishidian-zhongzhidian+1);
[C,I]=max(abs(Y));
set(handles.zhouqi_f,'String',1/f(I));
set(handles.pinlv_f,'String',f(I));
Y=Y(1:a);
b=2*sqrt(Y.*conj(Y));
plot(handles.axes1,f,b);
plot(handles.axes2,f,angle(Y));
plot(handles.axes3,f,real(Y));
plot(handles.axes4,f,imag(Y));
plot(handles.axes5,f,abs(Y).^2);
xlabel(handles.axes1,'F');
xlabel(handles.axes2,'F');
xlabel(handles.axes3,'F');
xlabel(handles.axes4,'F');
xlabel(handles.axes5,'F');
ylabel(handles.axes1,'AMP');
ylabel(handles.axes2,'PHASE');
ylabel(handles.axes3,'REAL');
ylabel(handles.axes4,'Imag');
ylabel(handles.axes5,'POWER');

function shengdao_Callback(hObject, eventdata, handles)
% hObject    handle to shengdao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of shengdao as text
%        str2double(get(hObject,'String')) returns contents of shengdao as a double


% --- Executes during object creation, after setting all properties.
function shengdao_CreateFcn(hObject, eventdata, handles)
% hObject    handle to shengdao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in dakai.
function dakai_Callback(hObject, eventdata, handles)
% hObject    handle to dakai (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

file=uigetfile('*.wav;*.flac;*.mp3');
%调用uigetfile函数打开音频文件，file返回的是文件名
[s fs]=audioread(file);
%利用audioread读取音频，temp用于计算时长，fs为默认的对数据的采样频率
%[temp,fs] = audioread(get(findobj('Tag','wav'),'String'));
shengdao=str2double(get(handles.shengdao,'String'));
if get(handles.wavjiazao,'Value')==0.0
  temp=s;  
else
    SNR=10;
   temp=awgn(s,SNR);

N= length(temp);
r2=randn(size(temp));
%r2为白噪声信号
b=fir1(31,0.5);
r21=filter(b,1,r2);
[temp,~]=add_noisedata(temp,r21,fs,fs,SNR);
%temp为原信号加噪声信号

end
if get(handles.wavlvbo,'Value')==0
else
    M=32;
    mu=0.001;
    lms1=dsp.LMSFilter(M,'StepSize',mu);
%调用dsp.LMSFilter函数进行自适应滤波
    [z,e,w] = lms1(temp,s(:));
    temp=z;
end
handles.shuruboxing=2;
handles.xxx=temp(:,shengdao);
t = (length(temp(:,1))/fs) ;
set(handles.shichang,'String',t);
%返回音频时长,单位为s
set(handles.fs,'String',fs);
guidata(hObject,handles);
plot(handles.axes6,handles.xxx);
title('WAV');
ysize=size(handles.xxx)
set(handles.n,'String',num2str(ysize(1)));

% --- Executes on selection change in boxing.
function boxing_Callback(hObject, eventdata, handles)
% hObject    handle to boxing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns boxing contents as cell array
%        contents{get(hObject,'Value')} returns selected item from boxing


% --- Executes during object creation, after setting all properties.
function boxing_CreateFcn(hObject, eventdata, handles)
% hObject    handle to boxing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pinlv_x_Callback(hObject, eventdata, handles)
% hObject    handle to pinlv_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pinlv_x as text
%        str2double(get(hObject,'String')) returns contents of pinlv_x as a double


% --- Executes during object creation, after setting all properties.
function pinlv_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pinlv_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fuzhi_x_Callback(hObject, eventdata, handles)
% hObject    handle to fuzhi_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fuzhi_x as text
%        str2double(get(hObject,'String')) returns contents of fuzhi_x as a double


% --- Executes during object creation, after setting all properties.
function fuzhi_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fuzhi_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xiangwei_x_Callback(hObject, eventdata, handles)
% hObject    handle to xiangwei_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xiangwei_x as text
%        str2double(get(hObject,'String')) returns contents of xiangwei_x as a double


% --- Executes during object creation, after setting all properties.
function xiangwei_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xiangwei_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in hundie.
function hundie_Callback(hObject, eventdata, handles)
% hObject    handle to hundie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of hundie


% --- Executes on button press in shengcheng.
function shengcheng_Callback(hObject, eventdata, handles)
% hObject    handle to shengcheng (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Fs=str2double(get(findobj('Tag','fs'),'String'));
N=str2double(get(findobj('Tag','n'),'String'));
x=linspace(0,N/Fs,N);
soundtype=get(handles.boxing,'Value');
pinlv=str2double(get(handles.pinlv_x,'String'));
fuzhi=str2double(get(handles.fuzhi_x,'String'));
xiangwei=str2double(get(handles.xiangwei_x,'String'));
switch soundtype
    case 1
        aa=2*pi*x*pinlv;
        xxx=fuzhi*sin(2*pi*x*pinlv+xiangwei);
        %产生正弦波
    case 2
        xxx=fuzhi*(2*rand(size(x))-1);
        %产生白噪声
    case 3
        aa=2*pi*x*pinlv;
        xxx=fuzhi*sawtooth(2*pi*x*pinlv+xiangwei);
        %产生锯齿波
    case 4
        aa=2*pi*x*pinlv;
        xxx=fuzhi*sign(sin(2*pi*x*pinlv+xiangwei));
        %产生方波
    case 5
        aa=2*pi*x*pinlv;
        xxx=fuzhi*sawtooth(2*pi*x*pinlv+xiangwei,0.5);
        %产生三角波
    otherwise
        errordlg('波形错误');
end
s=xxx;
%if get(handles.jiazao,'Value')==0.0
%判断是否选择了了加噪选项
%    xxx=xxx;
%else
%    xxx=awgn(xxx,20,'measured');
    %加入白噪声，参数20表示信噪比
%end
if get(handles.hundie,'Value')==0.0
    handles.xxx=xxx;
else
    handles.xxx=handles.xxx+xxx;
end

if get(handles.jiazao,'Value')==0.0
  temp=s;  
else
    
    SNR=10;
%信噪比为10
   temp=awgn(s,SNR);

N= length(temp);
r2=randn(size(temp));
%r2为白噪声信号
b=fir1(31,0.5);
r21=filter(b,1,r2);
[temp,~]=add_noisedata(temp,r21,Fs,Fs,SNR);
%temp为原信号加上噪声信号
handles.xxx=temp;
end
if get(handles.lvbo,'Value')==0
else
    M=32;
    mu=0.001;
    lms1=dsp.LMSFilter(M,'StepSize',mu);
%调用dsp.LMSFilter函数进行自适应滤波
    [z,e,w] = lms1(temp,s(:));
    temp=z;
    handles.xxx=temp;
end
handles.shuruboxing=3;
guidata(hObject,handles);
plot(handles.axes6,handles.xxx);
axis(handles.axes6,[0 N -str2double(get(handles.fuzhi_x,'String')) str2double(get(handles.fuzhi_x,'String'))]);



% --- Executes on button press in bofang.
function bofang_Callback(hObject, eventdata, handles)
% hObject    handle to bofang (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%a=str2double(get(findobj('Tag','fs'),'String'));

[file,b]=uigetfile('*.wav;*.flac;*.mp3');
%通过uigetfile函数打开音频文件，支持wav，mp3，flac格式的音频文件
%文件需要包含在matlab路径下，否则会报错
[y Fs]=audioread(file);
%读取保存的录音文件
play(audioplayer(y,Fs));
sound(y,Fs);
%调用sound函数播放录音，可在命令行窗口输入clear sound来停止


function shiyufenxi_Callback(hObject, eventdata, handles)
% hObject    handle to shiyufenxi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Fs=str2double(get(findobj('Tag','fs'),'String'));
N=str2double(get(findobj('Tag','n'),'String'));
if handles.shuruboxing==0
       msgbox('无波形，请重新输入');
       return;
end
%定义变量
nn=1;
%初始化最大值和最小值为0
zuidazhi=0;
zuixiaozhi=0;
j=3;
zhouqi=[];

TT=[];
fudu=[];
zuidazhi=max([handles.xxx(1) handles.xxx(2)]);
zuixiaozhi=min([handles.xxx(1) handles.xxx(2)]);
qidian=str2double(get(handles.qishidian,'String'));
zhongdian=str2double(get(handles.zhongzhidian,'String'));
if qidian<1 | zhongdian-qidian<5;
    msgbox('分析范围错误');
    return;
end
for j=qidian+2:zhongdian-1;
    %开始过零检测，分析起始点到终止点
    if handles.xxx(j-1)<0 & handles.xxx(j-2)<0 & handles.xxx(j)>=0 & handles.xxx(j+1)>0
      %此时j点或j-1点为零点
        if handles.xxx(j)==0
            TT(nn)=j;
        else
            TT(nn)=j-handles.xxx(j)/(handles.xxx(j)-handles.xxx(j-1));

        end
        fudu(nn)=(zuidazhi-zuixiaozhi)/2;
        zuidazhi=0;
        zuixiaozhi=0;
        nn=nn+1;
    else
        if zuidazhi<handles.xxx(j)
            zuidazhi=handles.xxx(j);
        end
        if zuixiaozhi>handles.xxx(j)
            zuixiaozhi=handles.xxx(j);
        end
    end
end
nn=nn-1;
for j=1:nn-1
    zhouqi(j)=TT(j+1)-TT(j);
end
freq=Fs/mean(zhouqi)
set(handles.zhouqi_t,'String',1/freq);
%周期输出
set(handles.pinlv_t,'String',num2str(freq));
set(handles.fuzhi_t,'String',num2str(mean(fudu(2:nn-1))));
%幅值要取平均值
phase=2*pi*(1-(TT(1:nn-1)-1)./zhouqi+floor((TT(1:nn-1)-1)./zhouqi));
set(handles.xiangwei_t,'String',num2str(mean(phase)));
set(handles.fengzhi,'String',(max(handles.xxx(qidian:zhongdian))-min(handles.xxx(qidian:zhongdian)))/2);
%峰值为波形的最大值
set(handles.junzhi,'String',mean(handles.xxx(qidian:zhongdian)));
set(handles.junfangzhi,'String',mean(handles.xxx(qidian:zhongdian).^2));
set(handles.fangcha,'String',std(handles.xxx(qidian:zhongdian))^2);
%调用std函数计算方差
% Hint: get(hObject,'Value') returns toggle state of timeanalyse



function shichang_Callback(hObject, eventdata, handles)
% hObject    handle to shichang (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of shichang as text
%        str2double(get(hObject,'String')) returns contents of shichang as a double


% --- Executes during object creation, after setting all properties.
function shichang_CreateFcn(hObject, eventdata, handles)
% hObject    handle to shichang (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in xinhaofashengqi.
function xinhaofashengqi_Callback(hObject, eventdata, handles)
% hObject    handle to xinhaofashengqi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of xinhaofashengqi
%信号发生器使能开关
set(findobj('Tag','pinlv_x'),'enable','on');
set(findobj('Tag','fuzhi_x'),'enable','on');
set(findobj('Tag','xiangwei_x'),'enable','on');
set(findobj('Tag','shengcheng'),'enable','on');
set(findobj('Tag','boxing'),'enable','on');
set(findobj('Tag','hundie'),'enable','on');
set(findobj('Tag','jiazao'),'enable','on');
set(findobj('Tag','lvbo'),'enable','on');
%wav文件使能开关
set(findobj('Tag','wav'),'enable','off');
set(findobj('Tag','shengdao'),'enable','off');
set(findobj('Tag','dakai'),'enable','off');
set(findobj('Tag','shichang'),'enable','off');
set(findobj('Tag','wavjiazao'),'enable','off');
set(findobj('Tag','wavlvbo'),'enable','off');
%录音使能开关
set(findobj('Tag','kaishiluyin'),'enable','off');
set(findobj('Tag','luyinshijian'),'enable','off');
set(findobj('Tag','bofang'),'enable','off');


% --- Executes on button press in wavwenjian.
function wavwenjian_Callback(hObject, eventdata, handles)
% hObject    handle to wavwenjian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of wavwenjian
%wav文件使能开关
set(findobj('Tag','wav'),'enable','on');
set(findobj('Tag','shengdao'),'enable','on');
set(findobj('Tag','dakai'),'enable','on');
set(findobj('Tag','shichang'),'enable','on');
set(findobj('Tag','wavjiazao'),'enable','on');
set(findobj('Tag','wavlvbo'),'enable','on');
%信号发生器使能开关
set(findobj('Tag','pinlv_x'),'enable','off');
set(findobj('Tag','fuzhi_x'),'enable','off');
set(findobj('Tag','xiangwei_x'),'enable','off');
set(findobj('Tag','boxing'),'enable','off');
set(findobj('Tag','hundie'),'enable','off');
set(findobj('Tag','shengcheng'),'enable','off');
set(findobj('Tag','jiazao'),'enable','off');
set(findobj('Tag','lvbo'),'enable','off');
%录音使能开关
set(findobj('Tag','kaishiluyin'),'enable','off');
set(findobj('Tag','luyinshijian'),'enable','off');
set(findobj('Tag','bofang'),'enable','off');


% --- Executes on button press in shengka.
function shengka_Callback(hObject, eventdata, handles)
% hObject    handle to shengka (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of shengka
%wav文件使能开关
set(findobj('Tag','wav'),'enable','off');
set(findobj('Tag','shengdao'),'enable','off');
set(findobj('Tag','dakai'),'enable','off');
set(findobj('Tag','shichang'),'enable','off');
set(findobj('Tag','wavjiazao'),'enable','off');
set(findobj('Tag','wavlvbo'),'enable','off');
%信号发生器使能开关
set(findobj('Tag','pinlv_x'),'enable','off');
set(findobj('Tag','fuzhi_x'),'enable','off');
set(findobj('Tag','xiangwei_x'),'enable','off');
set(findobj('Tag','boxing'),'enable','off');
set(findobj('Tag','hundie'),'enable','off');
set(findobj('Tag','shengcheng'),'enable','off');
set(findobj('Tag','jiazao'),'enable','off');
set(findobj('Tag','lvbo'),'enable','off');
%录音使能开关
set(findobj('Tag','kaishiluyin'),'enable','on');
set(findobj('Tag','luyinshijian'),'enable','on');
set(findobj('Tag','bofang'),'enable','on');


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in checkbox3.
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3


% --- Executes on button press in jiazao.
function jiazao_Callback(hObject, eventdata, handles)
% hObject    handle to jiazao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of jiazao


% --- Executes on selection change in heng.
function heng_Callback(hObject, eventdata, handles)
% hObject    handle to heng (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns heng contents as cell array
%        contents{get(hObject,'Value')} returns selected item from heng


% --- Executes during object creation, after setting all properties.
function heng_CreateFcn(hObject, eventdata, handles)
% hObject    handle to heng (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in zong.
function zong_Callback(hObject, eventdata, handles)
% hObject    handle to zong (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns zong contents as cell array
%        contents{get(hObject,'Value')} returns selected item from zong


% --- Executes during object creation, after setting all properties.
function zong_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zong (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function hengzuobiao_Callback(hObject, eventdata, handles)
% hObject    handle to hengzuobiao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
aa=get(hObject,'value');
aa=10^(aa*4+1);
x=get(handles.hengzuobiao,'value')*str2double(get(handles.n,'string'));
switch get(handles.heng,'value');
    case 1
        axis(handles.axes6,[0 aa+x -inf inf]);
    case 2
        axis(handles.axes1,[0 aa+x -inf inf]);
    case 3
        axis(handles.axes2,[0 aa+x -inf inf]);
    case 4
        axis(handles.axes3,[0 aa+x -inf inf]);
    case 5
        axis(handles.axes4,[0 aa+x -inf inf]);
    case 6
        axis(handles.axes5,[0 aa+x -inf inf]);
    otherwise
        errordlg('选择错误');
end



% --- Executes during object creation, after setting all properties.
function hengzuobiao_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hengzuobiao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function zongzuobiao_Callback(hObject, eventdata, handles)
% hObject    handle to zongzuobiao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
aa=get(hObject,'value');
amp=get(handles.fuzhi_x,'value')
bb=10^(aa*4+1);
cc=aa/5;
aa=10^(aa*5+1);
y=get(handles.zongzuobiao,'value')*str2double(get(handles.n,'string'));
switch get(handles.zong,'value');
    case 1
        axis(handles.axes6,[-inf inf -inf amp+y]);
    case 2
        axis(handles.axes1,[-inf inf 0 aa+y]);
    case 3
        axis(handles.axes2,[-inf inf -inf cc+y/10]);
    case 4
        axis(handles.axes3,[-inf inf -inf aa+y]);
    case 5
        axis(handles.axes4,[-inf inf -inf bb+y]);
    case 6
        axis(handles.axes5,[-inf inf 0 aa+y]);
    otherwise
        errordlg('选择错误');
end


% --- Executes during object creation, after setting all properties.
function zongzuobiao_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zongzuobiao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in zidong.
function zidong_Callback(hObject, eventdata, handles)
% hObject    handle to zidong (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%axis([handles.axes1,handles.axes2,handles.axes3,handles.axes4,handles.axes5,handles.axes6]auto);
axis(handles.axes6,[-inf inf -inf inf]);
axis(handles.axes5,[-inf inf -inf inf]);
axis(handles.axes4,[-inf inf -inf inf]);
axis(handles.axes3,[-inf inf -inf inf]);
axis(handles.axes2,[-inf inf -inf inf]);
axis(handles.axes1,[-inf inf -inf inf]);


% --- Executes on button press in lvbo.
function lvbo_Callback(hObject, eventdata, handles)
% hObject    handle to lvbo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of lvbo


% --- Executes on button press in wavjiazao.
function wavjiazao_Callback(hObject, eventdata, handles)
% hObject    handle to wavjiazao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of wavjiazao


% --- Executes on button press in wavlvbo.
function wavlvbo_Callback(hObject, eventdata, handles)
% hObject    handle to wavlvbo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of wavlvbo


% --- Executes during object creation, after setting all properties.
function shiyufenxi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to shiyufenxi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
