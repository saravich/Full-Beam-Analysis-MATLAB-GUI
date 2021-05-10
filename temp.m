function varargout = temp(varargin)
% TEMP MATLAB code for temp.fig
%      TEMP, by itself, creates a new TEMP or raises the existing
%      singleton*.
%
%      H = TEMP returns the handle to a new TEMP or the handle to
%      the existing singleton*.
%
%      TEMP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEMP.M with the given input arguments.
%
%      TEMP('Property','Value',...) creates a new TEMP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before temp_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to temp_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help temp

% Last Modified by GUIDE v2.5 06-Jul-2019 19:04:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @temp_OpeningFcn, ...
                   'gui_OutputFcn',  @temp_OutputFcn, ...
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


% --- Executes just before temp is made visible.
function temp_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to temp (see VARARGIN)

% Choose default command line output for temp
handles.unit=[];
handles.output = hObject;
handles.y=[];handles.V=[];handles.M=[];
handles.load.P=[];
handles.load.Torq=[];
handles.load.uni=[];
handles.load.line=[];
handles.PJ=[];
handles.FJ=[];
handles.roller=[];
handles.beam.prop=[];
handles.beam.x=[];
handles.Iprofile.mom=[];
handles.Uprofile.mom=[];
handles.d=[];
handles.bf=[];
handles.tf=[]; handles.tw=[];
handles.Q=[];
handles.valuetype=[];
handles.Syt=[]; 
handles.Sys=[];
handles.Syc=[];
handles.c='nothing';
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes temp wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = temp_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in unitsys.
function unitsys_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
v=get(handles.unitsys,'Value');
handles.unit=v;
guidata(hObject,handles);
% hObject    handle to unitsys (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns unitsys contents as cell array
%        contents{get(hObject,'Value')} returns selected item from unitsys


% --- Executes during object creation, after setting all properties.
function unitsys_CreateFcn(hObject, eventdata, handles)
% hObject    handle to unitsys (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
a=get(handles.slider1,'Value');
b=str2num(get(handles.length,'String'));
set(handles.x1,'String',b*a);
c=linspace(0,b,500);
a=b*a;
for i=2:500
    if a<c(5)
        yi=handles.y(1);
    elseif a>c(490)
        yi=handles.y(500);
    end
    if a>c(i-1) && a>c(i)
        yi=handles.y(i);
    end
end
set(handles.def,'String',yi)
guidata(hObject,handles);
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton9.

function pushbutton9_Callback(hObject, eventdata, handles)

handles=guidata(hObject);
%% generating y(x)
beam.x=handles.beam.x; % toole tir
beam.prop=handles.beam.prop;
E=beam.prop(1); I=beam.prop(2);
fJ=handles.FJ; % makane mafasele fix
PJ=[handles.PJ;handles.roller]; % makane mafasele pin va roller ke dar mohasebat yeksannd
handles.PJ=PJ;
load.P=handles.load.P; % moshakhasate niroo haye noghteei be soorat [p , x] dar har radif
load.line=handles.load.line; %location & size of tri forces[a , b, xi , xf]
load.Torq=handles.load.Torq; %location & size of single torques [T , x]
load.uni=handles.load.uni; %location & size of uniformly distributed loads forces[w , xi , xf]

n=2*length(fJ)+length(PJ)+2;  % tedade majhoolat
syms x
% FOR I=1:fj , C(2I)==GASHTAVARE fix joint , c(2i-1)=nirooye fix joint
syms(sym('c' , [1 n]));
var=sym('c' , [1 n]);
c=var;
y=@(x)0;
for i=1:size(load.P , 1)
    y=@(x)(y(x)+Poload(load.P(i ,1),load.P(i ,2) , x));
end
for i=1:size(load.line , 1)
   y=@(x)(y(x)+linload(load.line(i ,1),load.line(i ,2) ,load.line(i ,3),load.line(i ,4), x)); 
end
for i=1:size(load.Torq , 1)
   y=@(x)(y(x)+Torq(load.Torq(i , 1), load.Torq(i , 2) , x)); 
end
for i=1:size(load.uni , 1)
   y=@(x)(y(x)+uniload(load.uni(i , 1),load.uni(i , 2), load.uni(i , 3) , x)); 
end
for i=1:length(fJ)
   y=@(x)(y(x)+c(2*i)*Torq(1 , fJ(i) , x)+c(2*i-1)*Poload(1 , fJ(i) , x)); 
end
for i=(2*length(fJ)+1):(n-2)
   y=@(x)(y(x)+c(i)*Poload(1 , PJ(i-2*length(fJ)) , x)); 
end
y=@(x)(y(x)+c(n-1)*x + c(n));
%% generating equilibrium eqn. for solving y(x)
F=@(x)0;
M=@(x)0;
for i=1:length(fJ)
    F=@(x)(F(x)+c(2*i-1));
    M=@(x)(M(x)-fJ(i)*c(2*i-1)+c(2*i));
end
for i=(2*length(fJ)+1):(n-2)
    F=@(x)(F(x)+c(i));
    M=@(x)(M(x)-PJ(i-2*length(fJ))*c(i));
end
for i=1:size(load.uni , 1)
    [f , x0]=Funiload(load.uni(i , 1),load.uni(i , 2) , load.uni(i , 3));
    F=@(x)(F(x)+f);
    M=@(x)(M(x)-x0*f);
end
for i=1:size(load.line,1)
   [f , x0]=Flinload(load.line(i , 1),load.line(i , 2),load.line(i , 3) , load.line(i , 4));
   F=@(x)(F(x)+f);
   M=@(x)(M(x)-f*x0);
end
for i=1:size(load.Torq , 1)
   M=@(x)(M(x)+load.Torq(i , 1)); 
end
for i=1:size(load.P , 1)
    F=@(x)(F(x)+load.P(i , 1));
    M=@(x)(M(x)-load.P(i , 1)*load.P(i , 2));
    
end
syms x
eqn=[M(x)==0 ; F(x)==0];
%% generating B.C.
syms y2(x) V(x) M(x)
y2(x)=y(x);
dy=diff(y2 , x);
% baraye sharayet marzi dy=0 dar fix joint ha b dalil voojood tabe dirac
% dar moshtaghe heaviside, az dy(fj-eps) estefade shode
for i=1:length(fJ)
    b=eps(fJ(i));
    k=2;
    i1=1;
    if fJ(i)==0
        k=1;
    end
    while isnan(dy(fJ(i)-(-1)^k *i1*b))
        i1=i1+1;
    end
%     dy= ((-1)^k *y(fJ(i) + (-1)^k *i1*b )- (-1)^k *y(fJ(i)) )/i1*b;

    eqn=[eqn; dy(fJ(i)-(-1)^k *i1*b)==0 ; y(fJ)==0];
end
for i=1: length(PJ)
    eqn=[eqn ; y(PJ(i))==0];
end
M=diff(y2 , x , 2);
V=diff(y2 , x , 3);
%% solving & plotting
s=solve(eqn , var);
x=linspace(0 , beam.x,500);
M=M(x);
V=V(x);
y=y2(x)/(E*I);
y=subs(y , s);
V=subs(V , s);
M=subs(M , s);
y=double(y);
V=double(V);
M=double(M);
handles.y=y;
handles.M=M;
handles.V=V;
c=subs(c,s);
c=double(c);
handles.c=c;
%% mohasebeye hadeaxar tanesh ha
Vmax=max(abs(V));
Tr=Vmax*handles.Q/I/handles.tw; % Trans.. stress

Mmax=max(abs(M));
handles.cup=handles.d/2;
handles.cdown=handles.cup;
if Mmax==max(M)
    cc=handles.cup; % c compressive
    ct=handles.cdown; % c tensile
else
    ct=handles.cup;
    cc=handles.cdown;
end
CS=Mmax*cc/I; % Compresive Stress
TS=Mmax*ct/I; % Tensile stress
XNormal=x(find(max(M))); % makane bishtarin tanesh jaye kesheshi v feshari
XShear=x(find(max(V))); % makane bishtarin taneshe boreshi
  Max.mm=max(M);
  Max.vv=max(V);
  Max.yy=max(y);


%% mohr circle
OC=CS/2;
OT=TS/2;
RC=sqrt(Tr^2+(CS-OC)^2);
RT=sqrt(Tr^2+(TS-OT)^2);
TrMax=max(RC , RT); % bishtarin taneshe boreshi
set(handles.maxv,'String',TrMax);
TSMax=RT+OT; % bishtarin tanesh feshari da kol tir
CSMax=RC+OC; % hamoon kesheshish :)
set(handles.maxcom,'String',CSMax);
set(handles.maxtens,'String',TSMax);
if TSMax<handles.Syt
    set(handles.tensilefail,'String','SAFE!','ForegroundColor','green');
else
    set(handles.tensilefail,'String','UNSAFE!','ForegroundColor','red');
end
if TrMax<handles.Sys
    set(handles.shearfail,'String','SAFE!','ForegroundColor','green');
else
    set(handles.shearfail,'String','UNSAFE!','ForegroundColor','red');
end
if CSMax<handles.Syc
    set(handles.compfail,'String','SAFE!','ForegroundColor','green');
else
    set(handles.compfail,'String','UNSAFE!','ForegroundColor','red');
end
    %%%%%
  set(handles.edit34,'String',XShear);
   set(handles.edit35,'String',XNormal);
      set(handles.edit31,'String',TrMax);
   set(handles.edit32,'String',TSMax);

   
FBD(handles)
set(handles.axes1,'Visible','on');
axes(handles.axes1)
plot(x , y );
xlabel('x')
ylabel('Deflection')
hold on
y0=zeros(500 , 1);
plot(x , y0 , '--' );
hold off
set(handles.axes7,'Visible','on');
axes(handles.axes7)
plot(x , M);
xlabel('x')
ylabel('M')
set(handles.axes8,'Visible','on');
axes(handles.axes8)
plot(x , V);
xlabel('x')
ylabel('V')

guidata(hObject,handles);

% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=1+str2num(get(handles.edit2,'String'));
set(handles.edit2,'String',m);
if handles.unit==1
prompt={'Magnitude(N)=','x(m)='};
else
    prompt={'Magnitude(lb)=','x(in)='};
end
pointload=inputdlg(prompt,'Enter Load Data',[1 40]);
loadP(1,1)=str2double(pointload{1});
loadP(1,2)=str2double(pointload{2});
if m==1
    handles.load.P=loadP;
else
    handles.load.P=[handles.load.P;loadP];
end
handles.load.P;
FBD(handles)
guidata(hObject,handles);
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=1+str2num(get(handles.edit3,'String'));
set(handles.edit3,'String',m);
if handles.unit==1
prompt={'Magnitude(N.m)=','x(m)='};
else
prompt={'Magnitude(lb.in)=','x(in)='};
end    
pointtorq=inputdlg(prompt,'Enter Load Data',[1 40]);
loadTorq(1,1)=str2double(pointtorq{1});
loadTorq(1,2)=str2double(pointtorq{2});
if m==1
    handles.load.Torq=loadTorq;
else
    handles.load.Torq=[handles.load.Torq;loadTorq];
end
handles.load.Torq;
FBD(handles)
guidata(hObject,handles);
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.

function pushbutton5_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=1+str2num(get(handles.edit7,'String'));
set(handles.edit7,'String',m);
if handles.unit==1
prompt={'w1(N/m)=','w2(N/m)','x1(m)=','x2(m)='};
else
prompt={'w1(lb/in)=','w2(lb/in)','x1(in)=','x2(in)='};
end
lineload=inputdlg(prompt,'Enter Load Data (input 0 for q1 in case of triangular load)',[1 40]);
loadlin(1,1)=str2double(lineload{1});
loadlin(1,2)=str2double(lineload{2});
loadlin(1,3)=str2double(lineload{3});
loadlin(1,4)=str2double(lineload{4});
if m==1
    handles.load.line=loadlin;
else
    handles.load.line=[handles.load.line;loadlin];
end
handles.load.line;
FBD(handles)
guidata(hObject,handles);
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=1+str2num(get(handles.fj,'String'));
set(handles.fj,'String',m);
if handles.unit==1
prompt={'Enter location(m)='};
else
prompt={'Enter location(in)='};
end
fj1=inputdlg(prompt,'Add Fixed Joint:',[1 40]);
fj=str2num(fj1{1});
if m==1
    handles.FJ=fj;
else
    handles.FJ=[handles.FJ;fj];
end
handles.FJ;
FBD(handles)
guidata(hObject,handles);

% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in roll.
function pushbutton7_Callback(hObject, eventdata, handles)

% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=1+str2num(get(handles.pj,'String'));
set(handles.pj,'String',m);
if handles.unit==1
prompt={'Enter location(m)='};
else
    prompt={'Enter location(in)='};
end
pj1=inputdlg(prompt,'Add Pinned Joint:',[1 40]);
pj=str2num(pj1{1});
handles.PJ=[handles.PJ;pj];
FBD(handles)
guidata(hObject,handles);

% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in sectype.
function sectype_Callback(hObject, eventdata, handles)
% hObject    handle to sectype (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles=guidata(hObject);
valuetype=get(handles.sectype,'Value');
if valuetype==1      
    prompt={' b=',' h='};
    dimension=inputdlg(prompt,'Enter dimesnions of the cross section:');
    b=str2num(dimension{1});
    h=str2num(dimension{2});
    I=b*(h^3)/12;
    handles.beam.prop(2)=I;
    handles.d=h;
handles.Q=b*h^2/8;
handles.tw=b;
elseif valuetype==2
    if handles.unit==1
        Iprofile.name=['W310*129';'W310*074';'W310*067';'W310*039';'W250*149';'W250*080';'W250*067'];
handles.Iprofile.mom=[308 165 145 84.4 259 126 104]*10^(-6);
handles.d=[318 310 306 310 282 256 257]*0.001;
handles.tw=[13.1 9.4 8.51 5.84 17.3 9.4 8.89]*0.001;
handles.bf=[308 205 204 165 263 255 204]*0.001;
handles.tf=[20.6 16.3 14.6 9.7 28.4 15.6 15.7]*0.001;
    else 
        Iprofile.name=['W024*104';'W024*094';'W024*084';'W018*065';'W018*060';'W016*057';'W014*053'];
    handles.Iprofile.mom=[3100 2700 2370 1070 984 758 541];
    handles.d=[24.06 24.31 24.1 18.35 18.24 16.8 15.6];
    handles.tw=[0.5 0.515 0.47 0.45 0.415 0.43 0.37];
    handles.bf=[12.75 9.065 9.02 7.59 7.555 7.12 8.06];
    handles.tf=[0.75 0.875 0.77 0.75 0.695 0.715 0.66];
    
    
    end
         set(handles.type,'String','I-Profile');
    set(handles.typemenu,'String',Iprofile.name);
elseif valuetype==3
    if handles.unit==1
        Uprofile.name=['C380*074';'C380*060';'C380*050';'C250*045';'C250*037';'C250*030';'C200*028'];
handles.Uprofile.mom=[168 145 131 42.9 38 32.8 18.3]*10^(-6);
handles.d=[381 381 381 254 254 254 203]*0.001;
handles.tw=[18.2 13.2 10.2 17.1 13.4 9.63 12.4]*0.001;
handles.bf=[94.4 89.4 86.4 77 73.3 69.6 64.2]*0.001;
handles.tf=[16.5 16.5 16.5 11.1 11.1 11.1 9.9]*0.001;
    else
        Uprofile.name=['C015*050';'C012*030';'C010*030';'C009*020';'C009*015';'C006*013';'C005*009'];
        handles.Uprofile.mom=[404 162 103 60.9 51 17.4 8.9];
        handles.d=[15 12 10 9 9 6 5];
        handles.tw=[0.716 0.51 0.673 0.448 0.285 0.437 0.325];
        handles.bf=[3.716 3.17 3.033 2.648 2.485 2.157 1.885];
        handles.tf=[0.65 0.501 0.436 0.413 0.413 0.343 0.32];
    end
    set(handles.type,'String','C-Profile');
    set(handles.typemenu,'String',Uprofile.name);

elseif valuetype==4
prompt={'Diameter='};
d=str2double(inputdlg(prompt,'Enter Circle Diameter:=',[1 40]));
handles.beam.prop(2)=0.25*pi*(d/2)^4;
handles.Q=d^3/12;
handles.tw=d;
handles.d=d;

elseif valuetype==5
prompt={'Inner Diameter=','Outer Diameter='};
d=inputdlg(prompt,'Enter Circle Diameter:=',[1 40]);
d1=str2double(d{1});
d2=str2double(d{2});
handles.beam.prop(2)=0.25*pi*((d2/2)^4-(d1/2)^4);
handles.Q=(d2^3-d1^3)/96;
handles.tw=(d2-d1);
handles.d=d2;
end
  handles.valuetype=valuetype;

guidata(hObject,handles);
% Hints: contents = cellstr(get(hObject,'String')) returns sectype contents as cell array
%        contents{get(hObject,'Value')} returns selected item from sectype


% --- Executes during object creation, after setting all properties.
function sectype_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sectype (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in material.
function material_Callback(hObject, eventdata, handles)
% hObject    handle to material (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles=guidata(hObject);

valmat=get(handles.material,'Value');
if valmat==1
if handles.unit==1
    prompt={'E(GPa)=','Tensile Yield Strength(MPa)','Shear Yield Strength(MPa)','Compressive Yield Strength(MPa)'};
else
    prompt={'E(ksi)=','Yield Strength(ksi)','Shear Yield Strength(ksi)','Compressive Yield Strength(ksi)'};
end

    matprop=inputdlg(prompt,'Enter material properties:',[1 40]);
if handles.unit==1
    E=(10^12)*str2double(matprop{1});
     Syt=(10^6)*str2double(matprop{2});
     Sys=(10^6)*str2double(matprop{3});
     Syc=(10^6)*str2double(matprop{4});
else
      E=(10^3)*str2double(matprop{1});
        Syt=(10^3)*str2double(matprop{2});
     Sys=(10^3)*str2double(matprop{3});
     Syc=(10^3)*str2double(matprop{3});
end
elseif valmat==2
if handles.unit==1
    E=200*10^9;
    Syt=250*10^6;
    Sys=145*10^6;
    Syc=250*10^6;
else 
E=29000000;
end
elseif valmat==3
if handles.unit==1
E=70*10^9;
Syt=95*10^6;
Syc=95*10^6;
Sys=55*10^6;
else
E=10.1*10^6;
Syt=14000;
Sys=8000;
Syc=14000;
end
end
handles.beam.prop(1)=E; 
handles.Syt=Syt; 
handles.Sys=Sys;
handles.Syc=Syc;
guidata(hObject,handles);
% Hints: contents = cellstr(get(hObject,'String')) returns material contents as cell array
%        contents{get(hObject,'Value')} returns selected item from material


% --- Executes during object creation, after setting all properties.
function material_CreateFcn(hObject, eventdata, handles)
% hObject    handle to material (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function length_Callback(hObject, eventdata, handles)
% hObject    handle to length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles=guidata(hObject);
handles.beam.x=str2double(get(handles.length,'String'));
guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of length as text
%        str2double(get(hObject,'String')) returns contents of length as a double


% --- Executes during object creation, after setting all properties.
function length_CreateFcn(hObject, eventdata, handles)
% hObject    handle to length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_2_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



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



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pj_Callback(hObject, eventdata, handles)
% hObject    handle to pj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pj as text
%        str2double(get(hObject,'String')) returns contents of pj as a double


% --- Executes during object creation, after setting all properties.
function pj_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=1+str2num(get(handles.edit4,'String'));
set(handles.edit4,'String',m);
if handles.unit==1
prompt={'Magnitude(N/m)=','xi(m)=','xf(m)'};
else
    prompt={'Magnitude(lb/in)=','xi(in)=','xf(in)'}
end
uniload=inputdlg(prompt,'Enter Load Data',[1 40]);
loadUni(1,1)=str2double(uniload{1});
loadUni(1,2)=str2double(uniload{2});
loadUni(1,3)=str2double(uniload{3});
if m==1
    handles.load.uni=loadUni;
else
    handles.load.uni=[handles.load.uni;loadUni];
end
handles.load.uni;
FBD(handles)

guidata(hObject,handles);
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function fj_Callback(hObject, eventdata, handles)
% hObject    handle to fj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fj as text
%        str2double(get(hObject,'String')) returns contents of fj as a double


% --- Executes during object creation, after setting all properties.
function fj_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
a=get(handles.slider5,'Value');
b=str2num(get(handles.length,'String'));
set(handles.x2,'String',b*a);
c=linspace(0,b,500);
a=b*a;
for i=2:500
    if a<c(5)
        yi=handles.M(1);
    elseif a>c(490)
        yi=handles.M(500);
    end
    if a>c(i-1) && a>c(i)
        yi=handles.M(i);
    end
end
     
set(handles.mom,'String',yi)
guidata(hObject,handles);
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function x2_Callback(hObject, eventdata, handles)
% hObject    handle to x2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x2 as text
%        str2double(get(hObject,'String')) returns contents of x2 as a double


% --- Executes during object creation, after setting all properties.
function x2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function mom_Callback(hObject, eventdata, handles)
% hObject    handle to mom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mom as text
%        str2double(get(hObject,'String')) returns contents of mom as a double


% --- Executes during object creation, after setting all properties.
function mom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x1_Callback(hObject, eventdata, handles)
% hObject    handle to x1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x1 as text
%        str2double(get(hObject,'String')) returns contents of x1 as a double


% --- Executes during object creation, after setting all properties.
function x1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function def_Callback(hObject, eventdata, handles)
% hObject    handle to def (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of def as text
%        str2double(get(hObject,'String')) returns contents of def as a double


% --- Executes during object creation, after setting all properties.
function def_CreateFcn(hObject, eventdata, handles)
% hObject    handle to def (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
a=get(handles.slider7,'Value');
b=str2num(get(handles.length,'String'));
set(handles.x3,'String',b*a);
c=linspace(0,b,500);
a=b*a;
for i=2:500
    if a<c(5)
        yi=handles.V(1);
    elseif a>c(490)
        yi=handles.V(500);
    end
    if a>c(i-1) && a>c(i)
        yi=handles.V(i);
    end
end

set(handles.shear,'String',yi)
guidata(hObject,handles);
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function x3_Callback(hObject, eventdata, handles)
% hObject    handle to x3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x3 as text
%        str2double(get(hObject,'String')) returns contents of x3 as a double


% --- Executes during object creation, after setting all properties.
function x3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function shear_Callback(hObject, eventdata, handles)
% hObject    handle to shear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of shear as text
%        str2double(get(hObject,'String')) returns contents of shear as a double


% --- Executes during object creation, after setting all properties.
function shear_CreateFcn(hObject, eventdata, handles)
% hObject    handle to shear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit19_Callback(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit19 as text
%        str2double(get(hObject,'String')) returns contents of edit19 as a double


% --- Executes during object creation, after setting all properties.
function edit19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double


% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit22_Callback(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit22 as text
%        str2double(get(hObject,'String')) returns contents of edit22 as a double


% --- Executes during object creation, after setting all properties.
function edit22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit23_Callback(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit23 as text
%        str2double(get(hObject,'String')) returns contents of edit23 as a double


% --- Executes during object creation, after setting all properties.
function edit23_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit24_Callback(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit24 as text
%        str2double(get(hObject,'String')) returns contents of edit24 as a double


% --- Executes during object creation, after setting all properties.
function edit24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function roll_Callback(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roll as text
%        str2double(get(hObject,'String')) returns contents of roll as a double


% --- Executes during object creation, after setting all properties.
function roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in typemenu.
function typemenu_Callback(hObject, eventdata, handles)
% hObject    handle to typemenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns typemenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from typemenu
   handles=guidata(hObject);
   v=handles.valuetype;
   val=get(handles.typemenu,'Value');
   I=[];
   if v==2
        I=handles.Iprofile.mom(val);
   elseif v==3
        I=handles.Uprofile.mom(val);
   end

tf=handles.tf(val);
bf=handles.bf(val);
d=handles.d(val);
tw=handles.tw(val);
handles.d=d;
handles.tw=tw;
handles.Q=(tf*bf)*(d-tf)/2+tw/2*(d/2-tf)^2;
   I
   handles.beam.prop(2)=I;
    guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function typemenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to typemenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.

% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=1+str2num(get(handles.roll,'String'));
set(handles.roll,'String',m);
if handles.unit==1
prompt={'Enter location(m)='};
else
prompt={'Enter location(in)='};
end
rol1=inputdlg(prompt,'Add Roller:',[1 40])
roller=str2num(rol1{1});
handles.roller=[handles.roller;roller];
FBD(handles)
guidata(hObject,handles);
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function maxv_Callback(hObject, eventdata, handles)
% hObject    handle to maxv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxv as text
%        str2double(get(hObject,'String')) returns contents of maxv as a double


% --- Executes during object creation, after setting all properties.
function maxv_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function maxm_Callback(hObject, eventdata, handles)
% hObject    handle to maxm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxm as text
%        str2double(get(hObject,'String')) returns contents of maxm as a double


% --- Executes during object creation, after setting all properties.
function maxm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function maxtens_Callback(hObject, eventdata, handles)
% hObject    handle to maxtens (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxtens as text
%        str2double(get(hObject,'String')) returns contents of maxtens as a double


% --- Executes during object creation, after setting all properties.
function maxtens_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxtens (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function maxcom_Callback(hObject, eventdata, handles)
% hObject    handle to maxcom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxcom as text
%        str2double(get(hObject,'String')) returns contents of maxcom as a double


% --- Executes during object creation, after setting all properties.
function maxcom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxcom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in PJUndo.
function PJUndo_Callback(hObject, eventdata, handles)
% hObject    handle to PJUndo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles=guidata(hObject);
m=str2double(get(handles.pj , 'string'));
handles.PJ(m)=[];
m=m-1;
set(handles.pj , 'string' ,m )
FBD(handles)
guidata(hObject , handles);


% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
handles=guidata(hObject);
m=str2double(get(handles.fj , 'string'));
handles.FJ(m)=[];
m=m-1;
set(handles.fj , 'string' ,m )
FBD(handles)
guidata(hObject , handles);
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton25.
function pushbutton25_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=str2double(get(handles.roll , 'string'));
handles.roller(m)=[];
m=m-1;
set(handles.roll , 'string' ,m )
FBD(handles)
guidata(hObject , handles);
% hObject    handle to pushbutton25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton26.
function pushbutton26_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=str2double(get(handles.edit3 , 'string'));
handles.load.Torq(m,:)=[];
m=m-1;
set(handles.edit3 , 'string' ,m )
FBD(handles)
guidata(hObject , handles);
% hObject    handle to pushbutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=str2double(get(handles.edit4 , 'string'));
handles.load.line(m,:)=[];
m=m-1;
set(handles.edit4 , 'string' ,m )
FBD(handles)
guidata(hObject , handles);
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton29.
function pushbutton29_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
m=str2double(get(handles.edit7 , 'string'));
handles.load.uni(m,:)=[];
m=m-1;
set(handles.edit4 , 'string' ,m )
FBD(handles)
guidata(hObject , handles);
% hObject    handle to pushbutton29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function edit31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function edit32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function edit33_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function edit34_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit35_Callback(hObject, eventdata, handles)
% hObject    handle to edit35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit35 as text
%        str2double(get(hObject,'String')) returns contents of edit35 as a double


% --- Executes during object creation, after setting all properties.
function edit35_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit36_Callback(hObject, eventdata, handles)
% hObject    handle to edit36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit36 as text
%        str2double(get(hObject,'String')) returns contents of edit36 as a double


% --- Executes during object creation, after setting all properties.
function edit36_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
