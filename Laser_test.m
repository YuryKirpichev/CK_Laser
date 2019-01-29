function varargout = Laser_test(varargin)
% LASER_TEST MATLAB code for Laser_test.fig
%      LASER_TEST, by itself, creates a new LASER_TEST or raises the existing
%      singleton*.
%
%      H = LASER_TEST returns the handle to a new LASER_TEST or the handle to
%      the existing singleton*.
%
%      LASER_TEST('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LASER_TEST.M with the given input arguments.
%
%      LASER_TEST('Property','Value',...) creates a new LASER_TEST or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Laser_test_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Laser_test_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Laser_test

% Last Modified by GUIDE v2.5 01-Oct-2017 19:02:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Laser_test_OpeningFcn, ...
                   'gui_OutputFcn',  @Laser_test_OutputFcn, ...
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


% --- Executes just before Laser_test is made visible.
function Laser_test_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Laser_test (see VARARGIN)

% Choose default command line output for Laser_test
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Laser_test wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Laser_test_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ++++++++++++++++++++++++++
% кнопка импорта изображения
% ++++++++++++++++++++++++++

% выбираем файл для открытия
filename = uigetfile({'*.jpg;*.tif;*.png;*.gif','All Image Files';...
          '*.*','All Files' });
global RGB1;


% выводим нашу картинку
RGB1=imread(filename);
axes(handles.axes1)
imshow(RGB1);

 



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ++++++++++++++++++++++++++++++++++++++++++++++++++
% кнопка поворота изображения против часовой стрелке
% ++++++++++++++++++++++++++++++++++++++++++++++++++

global RGB1
RGB1 = rot90(RGB1,1);
axes(handles.axes1)
cla;
imshow(RGB1);

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ++++++++++++++++++++++++++++++++++++++++++++++
% кнопка поворота изображения по часовой стрелке
% ++++++++++++++++++++++++++++++++++++++++++++++

global RGB1
RGB1 = rot90(RGB1,3);
axes(handles.axes1)
cla;
imshow(RGB1);


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% +++++++++++++++++++++++++++++++++++++++++++++++++
% кнопка поворота изображения по горизонтальной оси
% +++++++++++++++++++++++++++++++++++++++++++++++++

global RGB1
RGB1 = flip(RGB1,1);
axes(handles.axes1)
cla;
imshow(RGB1);


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% +++++++++++++++++++++++++++++++++++++++++++++++
% кнопка поворота изображения по вертикальной оси
% +++++++++++++++++++++++++++++++++++++++++++++++

global RGB1

RGB1 = flip(RGB1,2);
axes(handles.axes1)
cla;


imshow(RGB1);


function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


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


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ++++++++++++++++++++++++++++
% Кнопка рассчета центра пучка
% ++++++++++++++++++++++++++++

global RGB1;
global BG;
global xl;
global yl;
% RGB -изображение минус фон, если он есть

% проверить этот иф

if isempty(BG)
        RGB = RGB1;
        fprintf('нет фона')
else
     for i = 1:3
        RGB(:,:,i) = RGB1(:,:,i) - mean(mean(BG(:,:,i)));
     end
end
 

%Преобразуем изображение в бинарное для последующего получения координат границ с помощью функции bwtraceboundary.

% I=rgb2gray(RGB);
% Работаем только в красном канале
I = RGB(:,:,1);
I = imadjust(I);
threshold=graythresh(I);
% BW = im2bw(I,threshold);
BW = imbinarize(I,threshold);
% BW = medfilt2(BW,[5,5]);
BW = wiener2(BW,[5,5]);
% BW = edge(I, 'sobel')
% удаление всех объектов, содержащих меньше чем 30 пикселей
BW=bwareaopen(BW,500);
% заполнение пустот

% Для реализации функции bwtraceboundary нужно указать начальную точку определения границы. Эта точка используется для начала отслеживания границы.

dim=size(BW);

%%%%%%
% удаляем шум

%%%%%%
%imshow (BW)

%     % Ищем начало круга
%     col=dim(1)/2;
%     % row = findRow(BW,col);
%     col = 1218
%     row = 256;
% 
%     % Результатом работы функции bwtraceboundary является поиск (X, Y) - координат точек границ исследуемого объекта. Для более точного определения радиуса исследуемого объекта нужно использовать максимальное число точек, которые принадлежат границе.
%     connectivity=8;
%     num_points=3000;
%     %contour=bwtraceboundary(BW, [row, col], 'N', connectivity, num_points);
[B,L]=bwboundaries(BW,'holes');
% imshow (I)
% imshow(BW);

B{1,1} = [0,0];
for i = 1:size(B,1)
    s(i) = size(B{i,1},1);
end

[m,k] = max(s)

boundary=B{k};

axes(handles.axes1);
hold on;
plot(boundary(:, 2), boundary(:, 1), 'r', 'LineWidth', 2)

hold on;
% plot(contour(:, 2), contour(:, 1), 'g', 'LineWidth', 0.5);
x=boundary(:, 2);
y=boundary(:, 1);
% при вычислении радиуса будет
% использоваться метод наименьших квадратов
abc=[x y ones(length(x), 1)]\[-(x.^2+y.^2)];
a=abc(1); b=abc(2); c=abc(3);
% вычислим положение центра и радиус
xc=-a/2;
yc=-b/2;
radius=sqrt((xc^2+yc^2)-c);
% отобразим вычисленный центр
axes(handles.axes1);
hold on;
plot(xc, yc, 'x', 'LineWidth', 2);
% отобразим полную окружность
theta=0:0.01:2*pi;
% используем параметрическое представление
% окружности для получения координат точек
Xfit=radius*cos(theta)+xc;
Yfit=radius*sin(theta)+yc;
axes(handles.axes1);
hold on;
plot(Xfit, Yfit);

% выводим значения вычисленного центра
set(handles.text9,'String',strcat('x = ',num2str(xc,4), 'px'));
set(handles.text10,'String',strcat('y = ',num2str(yc,4),'px'));

% берем значение разрешения сканера 
res = str2double(get(handles.edit3,'String'));

% выводим значение вычисленного радиуса
set(handles.edit2,'string',num2str(radius*2*25.4/res,4));

% вывоодим значения разницы центров
dx = (xc - xl)*25.4/res;
dy = (yc - yl)*25.4/res;
set(handles.text14,'String',strcat('dx = ',num2str(dx,4), 'mm'));
set(handles.text15,'String',strcat('dy = ',num2str(dy,4), 'mm'));

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)

% +++++++++++++++++++++++++++++
% кнопка указания центра лазера
% +++++++++++++++++++++++++++++

% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xl;
global yl;

[xl, yl] = ginput(1);
set(handles.text12,'String',strcat('x =  ',num2str(xl,4), 'px'));
set(handles.text13,'String',strcat('y =  ',num2str(yl,4),'px'));

hold on;
plot(xl, yl, 'o', 'LineWidth', 2);


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ++++++++++++++++++++++++++++++++++
% кнопка масштабирования изображения
% ++++++++++++++++++++++++++++++++++

global RGB1;

axes(handles.axes1)
zoom;


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ++++++++++++++++++++++++++++++
% кнопка перемещения изображения
% ++++++++++++++++++++++++++++++

global RGB1;

axes(handles.axes1)
pan;

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ++++++++++++++++++++++++++++++++
% кнопка сброса изменения масштаба
% ++++++++++++++++++++++++++++++++

global RGB1;

axes(handles.axes1)
zoom out;


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)

% Button 2 open the second image
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ++++++++++++++++++++++++++
% кнопка импорта изображения
% ++++++++++++++++++++++++++

% выбираем файл для открытия
filename = uigetfile({'*.jpg;*.tif;*.png;*.gif','All Image Files';...
          '*.*','All Files' });
global RGB2;


% выводим нашу картинку
RGB2=imread(filename);
axes(handles.axes2)
imshow(RGB2);





function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% ++++++++++++++++++++++++++++++++++++++++++++++++++
% кнопка поворота изображения против часовой стрелке
% ++++++++++++++++++++++++++++++++++++++++++++++++++
global RGB2
RGB2 = rot90(RGB2,1);
axes(handles.axes2)
cla;
imshow(RGB2);


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% ++++++++++++++++++++++++++++++++++++++++++++++
% кнопка поворота изображения по часовой стрелке
% ++++++++++++++++++++++++++++++++++++++++++++++

global RGB2
RGB2 = rot90(RGB2,3);
axes(handles.axes2)
cla;
imshow(RGB2);

% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global RGB2

% +++++++++++++++++++++++++++++++++++++++++++++++++
% кнопка поворота изображения по горизонтальной оси
% +++++++++++++++++++++++++++++++++++++++++++++++++

RGB2 = flip(RGB2,1);
axes(handles.axes2)
cla;
imshow(RGB2);


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% +++++++++++++++++++++++++++++++++++++++++++++++
% кнопка поворота изображения по вертикальной оси
% +++++++++++++++++++++++++++++++++++++++++++++++

global RGB2

RGB2 = flip(RGB2,2);
axes(handles.axes2)
cla;
imshow(RGB2);


function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


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


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% ++++++++++++++++++++++++++++
% Кнопка рассчета центра пучка
% ++++++++++++++++++++++++++++

global RGB2;
global BG;
global x2l;
global y2l;
% RGB -изображение минус фон, если он есть

% проверить этот иф

if isempty(BG)
        RGB = RGB2;
        fprintf('нет фона')
else
     for i = 1:3
        RGB(:,:,i) = RGB2(:,:,i) - mean(mean(BG(:,:,i)));
     end
end
 

%Преобразуем изображение в бинарное для последующего получения координат границ с помощью функции bwtraceboundary.

% I=rgb2gray(RGB);
% Работаем только в красном канале
I = RGB(:,:,1);
I = imadjust(I);
threshold=graythresh(I);
% BW = im2bw(I,threshold);
BW = imbinarize(I,threshold);
% BW = medfilt2(BW,[5,5]);
BW = wiener2(BW,[5,5]);
% BW = edge(I, 'sobel')
% удаление всех объектов, содержащих меньше чем 30 пикселей
BW=bwareaopen(BW,500);
% заполнение пустот

% Для реализации функции bwtraceboundary нужно указать начальную точку определения границы. Эта точка используется для начала отслеживания границы.

dim=size(BW);

%%%%%%
% удаляем шум

%%%%%%
%imshow (BW)

%     % Ищем начало круга
%     col=dim(1)/2;
%     % row = findRow(BW,col);
%     col = 1218
%     row = 256;
% 
%     % Результатом работы функции bwtraceboundary является поиск (X, Y) - координат точек границ исследуемого объекта. Для более точного определения радиуса исследуемого объекта нужно использовать максимальное число точек, которые принадлежат границе.
%     connectivity=8;
%     num_points=3000;
%     %contour=bwtraceboundary(BW, [row, col], 'N', connectivity, num_points);
[B,L]=bwboundaries(BW,'holes');
% imshow (I)
% imshow(BW);



B{1,1} = [0,0];
for i = 1:size(B,1)
    s(i) = size(B{i,1},1);
end

[m,k] = max(s)

boundary=B{k};

% axes(handles.axes2)
% hold on;
% plot(boundary(:, 2), boundary(:, 1), 'r', 'LineWidth', 2)
% 
% hold on;
% plot(contour(:, 2), contour(:, 1), 'g', 'LineWidth', 0.5);
x=boundary(:, 2);
y=boundary(:, 1);
% при вычислении радиуса будет
% использоваться метод наименьших квадратов
abc=[x y ones(length(x), 1)]\[-(x.^2+y.^2)];
a=abc(1); b=abc(2); c=abc(3);
% вычислим положение центра и радиус
x2c=-a/2;
y2c=-b/2;
radius=sqrt((x2c^2+y2c^2)-c);
% отобразим вычисленный центр
plot(x2c, y2c, 'x', 'LineWidth', 2);
% отобразим полную окружность
theta=0:0.01:2*pi;
% используем параметрическое представление
% окружности для получения координат точек
Xfit=radius*cos(theta)+x2c;
Yfit=radius*sin(theta)+y2c;
plot(handles.axes2,Xfit, Yfit);

% выводим значения вычисленного центра
set(handles.text21,'String',strcat('x = ',num2str(x2c,4), 'px'));
set(handles.text22,'String',strcat('y = ',num2str(y2c,4),'px'));

% берем значение разрешения сканера 
res = str2double(get(handles.edit3,'String'));

% выводим значение вычисленного радиуса
set(handles.edit5,'string',num2str(radius*2*25.4/res,4));

% вывоодим значения разницы центров
dx = (x2c - x2l)*25.4/res;
dy = (y2c - y2l)*25.4/res;
set(handles.text25,'String',strcat('dx = ',num2str(dx,4), 'mm'));
set(handles.text26,'String',strcat('dy = ',num2str(dy,4), 'mm'));


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% +++++++++++++++++++++++++++++
% кнопка указания центра лазера
% +++++++++++++++++++++++++++++

% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x2l;
global y2l;

[x2l, y2l] = ginput(1);
set(handles.text23,'String',strcat('x =  ',num2str(x2l,4), 'px'));
set(handles.text24,'String',strcat('y =  ',num2str(y2l,4),'px'));

hold on;
plot(x2l, y2l, 'o', 'LineWidth', 2);


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% ++++++++++++++++++++++++++++++++++
% кнопка масштабирования изображения
% ++++++++++++++++++++++++++++++++++

global RGB2;

axes(handles.axes2)
zoom;


% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global RGB2;

axes(handles.axes2)
pan;


% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global RGB2;

axes(handles.axes2)
zoom out;


% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% +++++++++++++++++++++++++++++++++++++
% кнопка выбора фона (пленки без пучка)
% +++++++++++++++++++++++++++++++++++++

global BG
filename = uigetfile({'*.jpg;*.tif;*.png;*.gif','All Image Files';...
          '*.*','All Files' });

% выводим нашу картинку
BG=imread(filename);




% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ++++++++++++++++++++++++++
% кнопка очищения граффика 1
% ++++++++++++++++++++++++++

axes(handles.axes1)
cla
