
clear all

global data
global i
i= 0;

data = [];
h_fig = figure;
set(h_fig,'KeyPressFcn',@myfun);

function myfun(src,event)
    global data
    global i
    i = i + 1;
   disp(event.Key);
   data(i,1)=now;
   disp(data(i,1))
   data(i,2)=event.Key;

end