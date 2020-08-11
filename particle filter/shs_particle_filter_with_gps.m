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

fig = figure();
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

%% Import GPS Data

gps_data = ImportGPSData("/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/3_times_around_the_block/location-gps.txt");


 gps_position = lla2flat( [ gps_data.latitude, gps_data.longitude, gps_data.altitude ], ...
               [gps_data(1,:).latitude, gps_data(1,:).longitude], ...
              0, 0 );

gps_data.x_pos = gps_position(:,2) + start_point_meter(2);
gps_data.y_pos = gps_position(:,1) + start_point_meter(1);
          
figure()
hold on
show(map)
plot(gps_data.x_pos,gps_data.y_pos)
hold off   

figure()
geoplot( gps_data.latitude,gps_data.longitude, 'g-*')

gps_data  = retime(gps_data,shs.steps.data.Time,'linear');


%%

gps_time_tol = withtol(step_orient.Time, seconds(0.01));
gps_tol = gps_data(gps_time_tol,:);

%%
clc
nr_particles = 100;

[specific_pf, final_timestep] = ParticleFilterSysResample(start_point_meter,nr_particles, ...
                                                step_orient, 0.06, 0.02, map, gps_tol);
disp(['pf completed:' num2str(final_timestep/height(step_orient))])

[pf_mean_error,pf_mean_std_error] = CompareToGPS(specific_pf,gps_data);

for ii = 1 :10: length(specific_pf) 
    gps_point = gps_data(specific_pf(ii).Time,:); 
    figure(1)
    show(map)
    hold on
    scatter([specific_pf(ii).particle_lists.x_pos]', [specific_pf(ii).particle_lists.y_pos]', '.')
    scatter(gps_point.x_pos, gps_point.y_pos, 'or')
    hold off
end





