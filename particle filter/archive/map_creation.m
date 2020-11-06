%% generating particle filter map

% defining the scale of the map
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


%% finding the start point

fig = figure(1);
imshow(image)
hold on
start_point_pixel = ginput(1);

start_point_meter = start_point_pixel/points_per_meter;

hold off
close(fig)
%% checking distances from occupancyMAp

fig = figure(1);
show(map)
hold on
measuring_points = ginput(2)
hold off
close(fig)
%%
diff(measuring_points)

%%
map = occupancyMap(white,points_per_meter);

figure()
show(map)
hold on
start_point_meter = ginput(1);
plot([positions.x]+start_point_meter(1,1),[positions.y] +start_point_meter(1,2),'-r','LineWidth',2) % Original location

x = [positions.x];
y = [positions.y];