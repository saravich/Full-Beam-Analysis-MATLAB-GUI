function PLPlot(Pload , L,handles)
% Pload(1)=P & Pload(2)=x
a=get(handles.axes9, 'Position');
thex=@(x)(a(1)+a(3)*x/L); %normalizing coordinates
they=@(y)(a(2)+a(4)*(y+1)/2);
if Pload(1)<0
    y=[.25+.25/2 , .25/2];
else
    y=[-.25-.25/2 , -.25/2];
end
x=[Pload(2) Pload(2)];
x=thex(x);
y=they(y);
andaze=num2str(Pload(1));
if handles.unit==1
    b='N';
else
    b='lb';
end
str=['P= ' andaze b];
annotation('textarrow' , 'x' , x ,'y' , y , 'string' , str  , 'color' , 'red');
    
