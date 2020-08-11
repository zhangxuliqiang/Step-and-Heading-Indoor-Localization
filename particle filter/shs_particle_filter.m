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

% measure_distance = ginput(2);
% norm(measure_distance(:,2)-measure_distance(:,1))

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

%% itterate over both orientation and step length std to find best combination
clc
nr_particles = 100;
std_orient_counter = 0;

orient_pf = [];

for std_orient = 0.02:0.03:0.2
    std_orient_counter = std_orient_counter +1;
    
    fprintf('std_orient = %f \n', std_orient )
    
    sl_pf = [];
    sl_std_counter = 0;
    
    for std_sl = 0.1:0.1:1
        sl_std_counter = sl_std_counter +1;
        fprintf('       std_sl = %f \n', std_sl )
        realizations =[];
        
        for itteration = 1:10
            fprintf('       itteration: %i' ,itteration )
            
            [particle_lists, final_timestep] = ParticleFilter(start_point_meter,nr_particles, step_orient, std_sl,std_orient, map);
            
            fprintf('       pf completed: %f \n',final_timestep/height(step_orient))
            realizations(itteration).percent_complete = final_timestep/height(step_orient);
            
            [realizations(itteration).pf_mean_error, ...
                realizations(itteration).pf_mean_std_error] = ...
                CompareToGPS(particle_lists,gps_data);
        end
        sl_pf(sl_std_counter).std_sl = std_sl;
        sl_pf(sl_std_counter).realizations = realizations;
        sl_pf(sl_std_counter).completed = sum([realizations.percent_complete] == 1);
    end
    orient_pf(std_orient_counter).std_orient = std_orient;
    orient_pf(std_orient_counter).sl_pf = sl_pf;
end

%%
close all
plot_index = 1;
sub_plot_length = length(orient_pf) + 1;
std_sl_x_axis  = 0.1:0.1:1; 

 t = tiledlayout(length(orient_pf),1);
 
for i = 1:length(orient_pf)
    
    specific_sl_pf = orient_pf(i).sl_pf;
    
    bc_mean_error = [];
    bc_mean_std = [];
    
    for std_sl_index = 1:10
        track_completed_index = [specific_sl_pf(std_sl_index).realizations.percent_complete] == 1;
        bc_mean_error = [bc_mean_error; [specific_sl_pf(std_sl_index).realizations.pf_mean_error].*track_completed_index];
        bc_mean_std = [bc_mean_std; [specific_sl_pf(std_sl_index).realizations.pf_mean_std_error].*track_completed_index];
    end
    
    ax(i) = nexttile;
    b = bar(ax(i),std_sl_x_axis,bc_mean_error);
    title(['orientation std: '  num2str(orient_pf(i).std_orient)])
    set(ax(i),'fontsize',10)
    ylim([0,100])
    
    xtips2 = b(5).XEndPoints-0.005;
    ytips2 = 70.*ones(size(b(2).XEndPoints));
    labels2 = string([specific_sl_pf.completed]);
    text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom','color','b', 'fontsize',6)
    
end
title(t,'mean error from corresponding gps point')
xlabel(t,'step length std (m)')
ylabel(t,'Error (m)')
xticklabels(ax(1:end-1),{})
t.TileSpacing = 'compact';

%%
clc
[specific_pf, final_timestep] = ParticleFilter(start_point_meter,nr_particles, ...
                                                step_orient, 0.1, 0.08, map);
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





