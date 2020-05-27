%% generating particle filter map

figure()
imshow('pietheinstraat.png')
hold on
scale_points = ginput(2)


%%
close
image = imread('pietheinstraat.png');
I = rgb2gray(image);
white = I < 250;

fig = figure(1);
imshow(image)
hold on
scale_points = ginput(2);
hold off
close(fig)

points_per_meter = diff(scale_points(:,1))/20;

%%
map = occupancyMap(white,points_per_meter);

figure()
show(map)
hold on
plot([positions.x]+100,[positions.y] + 100,'-r','LineWidth',2) % Original location

%%
plot([positions.x],[positions.y])

%%
figure
plot(white)


%%
BW = imbinarize(I,250); 
imshowpair(I,BW,'montage')