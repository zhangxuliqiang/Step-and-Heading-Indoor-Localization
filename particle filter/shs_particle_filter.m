
%% generating particle filter map

% defining the scale of the map
image = imread('pietheinstraatv3.png');
I = rgb2gray(image);
white = I < 250;

[row_size, column_size] = size(white);

Red = image(:,:,1)>250 & image(:,:,3)<50 & image(:,:,2)<50 ;



[row,col] = find(Red);

start_point = [row_size - mean(row),mean(col)];

white(row,col) = 0;

fig = figure(1);
imshow(image)


% hold on
% scale_points = ginput(2);
% hold off
% close(fig)
% %
% points_per_meter = diff(scale_points(:,1))/20;

points_per_meter = 3.85;
map = occupancyMap(white,points_per_meter);
start_point_meter = start_point/points_per_meter;

figure()
hold on
show(map)
scatter(start_point_meter(2),start_point_meter(1),'bx')
hold off

% measure_distance = ginput(2);
% norm(measure_distance(:,2)-measure_distance(:,1))

%%

gps_data = ImportGPSData("/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/3_times_around_the_block/location-gps.txt");


 gps_position = lla2flat( [ gps_data.latitude, gps_data.longitude, gps_data.altitude ], ...
               [gps_data(1,:).latitude, gps_data(1,:).longitude], ...
              0, 0 );

gps_data.x_pos = p(:,2) + start_point_meter(2);
gps_data.y_pos = p(:,1) + start_point_meter(1);
          
figure()
hold on
show(map)
plot(gps_data.x_pos,gps_data.y_pos)
hold off   


figure()
geoplot( gps_data.latitude,gps_data.longitude, 'g-*')

%%

gps_data  = retime(gps_data,shs.steps.data.Time,'linear');


figure()
hold on
show(map)
plot(gps_data.x_pos,gps_data.y_pos)
hold off 

%% distribute particles over map

nr_particles = 100;

sl_pf = [];
counter = 0;

for std_sl = 0.1:0.1:1
    counter = counter +1;
    fprintf('delta_sl = %f \n', std_sl )
    realizations =[];
    
    for itteration = 1:10
        fprintf('    itteration: %i' ,itteration )
        
       [particle_lists, final_timestep] = ParticleFilter(start_point_meter,nr_particles, step_orient, std_sl, map);
        
        fprintf('    pf completed: %f \n',final_timestep/height(step_orient))
        realizations(itteration).percent_complete = final_timestep/height(step_orient);

    end

    sl_pf(counter).completed = sum([realizations.percent_complete] == 1);
end

%%
% close all
trajectory = og_positions;

figure()
hold on
show(map)
x = [trajectory.x] + + start_point_meter(2);
y = [trajectory.y] + start_point_meter(1);
plot(x,y, 'y')
hold off
%%

[specific_pf, final_timestep] = ParticleFilter(start_point_meter,nr_particles, ...
                                                step_orient, 4/10, map);
disp(['pf completed:' num2str(final_timestep/height(step_orient))])

%%
for i = 1 :10: length(specific_pf) 
    gps_point = gps_data(specific_pf(i).Time,:); 
    figure(1)
    show(map)
    hold on
    scatter([specific_pf(i).particle_lists.x]', [specific_pf(i).particle_lists.y]', '.')
    scatter(gps_point.x_pos, gps_point.y_pos, 'or')
    hold off
end





