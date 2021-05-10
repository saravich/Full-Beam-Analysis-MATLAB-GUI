function pinplot(x,handles)
tet=linspace(0 ,2*pi);
t=(.25/3)*cos(tet);
y=(.25/3)*sin(tet);
t=t+x*ones(size(t));
set(handles.axes9,'Visible','on');
axes(handles.axes9)
plot(t, y);
hold on;
tet=linspace(0 , pi);
t=(.25/3+.02)*cos(tet);
t=t+x*ones(size(t));
y=(.25/3+.02)*sin(tet);
plot(t , y);
t2=[t(1) , t(100) ; t(1) , t(100)];
y2=[y(1) , y(100) ; y(1)-.3 , y(100)-.3];
plot(t2 , y2);
rectangle('position' , [t(100)-.02 , y(100)-.34 , 2*(.25/3 + .02)+.04 , .04  ] , 'faceColor','black')




