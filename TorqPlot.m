function TorqPlot(Torqload , L,handles)
M=Torqload(1);
x=Torqload(2);
a=get(handles.axes9, 'Position');
thex=@(x)(a(1)+a(3)*x/L); %normalizing coordinates
they=@(y)(a(2)+a(4)*(y+1)/2);
tet=linspace(-pi/2 , pi/2);
x2=2*x/L-1; %normal to [-1 1]
x3=.15*cos(tet);
y3=.15*sin(tet);
x3=x3+ x2*ones(1 , 100);
set(handles.axes9,'Visible','on');
axes(handles.axes9)
plot(x3 , y3)
if M<0
    x3=[x ,x-.05 ];
    y3=[-.15 -.15];
else
    x3=[x , x-.05 ];
    y3=[.15 .15];
end
x3=thex(x3);
y3=they(y3);
thex2=@(x)(a(1)+a(3)*(x+1)/2);
annotation('arrow' , 'x' , x3 , 'y' , y3)
pos=[thex2(x2-.15) , they(.30) , .3*a(3)/2 , .14*a(4)/2];
if handles.unit==1
    b='N.m';
else
    b='lb.in';
end
annotation('textbox' , 'position' , pos , 'FitBoxToText' , 'on' , 'string' , ['M= ', num2str(M) , b] , 'EdgeColor' , 'none');