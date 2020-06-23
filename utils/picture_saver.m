function picture_saver()

h =  findobj('type','figure');
n = length(h);
time = datestr(now,'yyyymmdd_HHMM');

for i  = 1:1:n
    fig = figure(i);
    h1= gca;
    title = h1.Title;
    
    
    if isempty(title) || size(title,2)< 1
        identifier = ['fig_',int2str(i)];
    else
        identifier = title.String;
    end
    
    identifier = strrep(identifier," ", "_");
    pic_name = strcat('/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/pictures/',time,'_', identifier);
    
    
    saveas(fig,pic_name,'eps');
    saveas(fig,pic_name,'png');
    saveas(fig,pic_name,'fig');
end
end
