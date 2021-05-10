function FBD(handles)
cla(handles.axes9 , 'reset');
delete(findall(gcf, 'type' , 'annotation'))
axis ([-1 1 -1 1])
axis manual
hold on
if handles.beam.x>0
    L=handles.beam.x;
    rectangle('position',[-1 , -.25/2 , 2 , .25] , 'facecolor' , 'blue')
    
    if any(ischar(handles.c))
        for i=1:length(handles.roller)
           rolplot((2*handles.roller(i)/handles.beam.x)-1,handles)  % norm to [-1 1]
        end
        for i=1:length(handles.PJ)
           pinplot((2*handles.PJ(i)/handles.beam.x)-1,handles) 
        end
        for i=1:length(handles.FJ)
           fixplot((2*handles.FJ(i)/handles.beam.x)-1,handles) 
        end
    else
        for i=1:length(handles.FJ)
           TorqPlot([handles.c(2*i) , handles.FJ(i)] , L,handles)
           PLPlot([handles.c(2*i-1) , handles.FJ(i)] , L,handles)
        end
        for i=1:length(handles.PJ)
           PLPlot([handles.c(i+2*length(handles.FJ)) , handles.PJ(i)] , L,handles)
        end
    end

    for i=1:size(handles.load.P , 1)
       PLPlot(handles.load.P(i , :) , L ,handles) 
    end
    for i=1:size(handles.load.uni , 1)
       UDLPlot(handles.load.uni(i , :) , L,handles ) 
    end
    for i=1:size(handles.load.line , 1)
       linPlot(handles.load.line(i , :) , L,handles) 
    end
    for i=1:size(handles.load.Torq  , 1)
        TorqPlot(handles.load.Torq(i , :) , L,handles)
    end 
end
