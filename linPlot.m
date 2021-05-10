function linPlot(linload , L,handles)
w1=linload(1);
w2=linload(2);
xi=linload(3);
xf=linload(4);
% L= length of beam
a=get(handles.axes9, 'Position');
thex=@(x)(a(1)+a(3)*x/L); %normalizing coordinates
they=@(y)(a(2)+a(4)*(y+1)/2);
x=thex([xi , xf]);
if abs(w1)<abs(w2)
    L1=.2;
    L2=.4;
else
    L1=.4;
    L2=.2;
end
if w1>0
    y1=[-.25/2 - L1 , -.25/2];
else
    y1=[.25/2 + L1 , .25/2];
end
if w2>0
    y2=[-.25/2 - L2 , -.25/2];
else
    y2=[.25/2 + L2 , .25/2];
end
y1=they(y1);
y2=they(y2);
if handles.unit==1
    b='N/m';
else
    b='lb/in';
end
annotation('textarrow' , 'x' , [x(1) x(1)] , 'y' , y1 , 'string' , ['w_1= ' num2str(w1) b], 'color' , 'm')
annotation('textarrow' , 'x' , [x(2) x(2)] , 'y' , y2 , 'string' , ['w_2= ' num2str(w2) b], 'color' , 'm')
annotation('line' , 'x' , x , 'y' , [y1(1) , y2(1)] , 'color' , 'm' )