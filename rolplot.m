function rolplot(x,handles)
tet=linspace(0 ,2*pi);
t=(.25/3)*cos(tet);
t=t+x*ones(1 , 100);
y=(.25/3)*sin(tet);
y=y-(.25/3+.25/2);
% t=t+x*ones(1 , 100);
set(handles.axes9,'Visible','on');
axes(handles.axes9)
plot(t, y);
hold on;
rectangle('position' , [min(t) , min(y)-.02 , 2*.25/3 , .02] , 'faceColor','black' )
