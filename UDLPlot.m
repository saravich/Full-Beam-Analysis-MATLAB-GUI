function UDLPlot(uniload , L,handles)
w=uniload(1);
xi=uniload(2);
xf=uniload(3);
a=get(handles.axes9, 'Position');
thex=@(x)(a(1)+a(3)*x/L); %normalizing coordinates
they=@(y)(a(2)+a(4)*(y+1)/2);
x=[xi , (xi+xf)/2 , xf];
if w>0
    y=[-.25/2-.25 , -.25/2];
else
    y=[.25/2+.25 , .25/2];
end
x=thex(x);
y=they(y);
if handles.unit==1
    b='N/m';
else
    b='lb/in';
end
annotation('arrow' , 'x' ,[x(1) x(1)] , 'y' , y , 'color' , 'cyan');
annotation('textarrow' , 'x' ,[x(2) x(2)] , 'y' , y , 'string' , ['w= ' num2str(w) b], 'color' , 'cyan');
annotation('arrow' , 'x' ,[x(3) x(3)] , 'y' , y, 'color' , 'cyan');
annotation('line' , 'x' , [x(1) x(3)] , 'y' , [y(1) y(1)], 'color' , 'cyan');