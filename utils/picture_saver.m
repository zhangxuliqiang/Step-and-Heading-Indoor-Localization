function picture_saver(name)

if nargin == 0
    name = '';
end

h =  findobj('type','figure');
n = length(h);
time = datestr(now,'yyyymmdd_HHMM');

for i  = 1:1:n
    fig = figure(i);
    h1= gca;
    title = h1.Title;
    
    if isempty(name)
        if isempty(title) || size(title,2)< 1
            identifier = ['fig_',int2str(i)];
        else
            identifier = title.String;
        end
    else        
        identifier = name;
    end
    
    identifier = strrep(identifier," ", "_");
    pic_name = strcat('/home/',getenv('USER'),'/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/pictures/',time,'_', identifier);
    

    
    saveas(fig,pic_name,'epsc');
    saveas(fig,pic_name,'png');
    saveas(fig,pic_name,'fig');
end
end
