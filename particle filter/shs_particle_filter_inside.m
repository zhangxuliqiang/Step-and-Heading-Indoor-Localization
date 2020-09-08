%% generating particle filter map


% defining the scale of the map
image = imread('binnenkaart.png');
white_regions_image  = rgb2gray(image)< 250;

[row_size, column_size] = size(white_regions_image);

red_regions_image = image(:,:,1)>250 & image(:,:,3)<50 & image(:,:,2)<50 ;

walls_image = white_regions_image ~= red_regions_image;

fig = figure(); 
imshow(image)
title('choose the points needed for meter to pixel conversion')
hold on
scale_points = ginput(2);
hold off
close(fig)

real_world_length = 12.25;
points_per_meter = norm(scale_points(:,1)-scale_points(:,2))/real_world_length;

walls = occupancyMap(walls_image,81.97);

door_pixel_location = bwboundaries(red_regions_image);

doors = grid2local(walls, cell2mat(door_pixel_location));

%%
fig = figure()
hold on
show(walls)
plot(doors(:,1),doors(:,2),'x')
% measure_points = ginput(2);
% hold on 
% plot(measure_points(1,1),measure_points(1,2),'x')
% plot(measure_points(2,1),measure_points(2,2),'x')
% 
% 
% norm(measure_points(1,:)-measure_points(2,:))

%%

fig = figure(); 
show(walls)
title('choose the starting point')
hold on
start_point = ginput(1);
hold off
close(fig)

shs_output = [og_positions.x, og_positions.y];

theta = pi;
rot_mat = [cos(theta),-sin(theta);
        sin(theta),cos(theta)];
    
shs_output_final = rot_mat* shs_output';

figure()
hold on
show(walls)
scatter(start_point(1),start_point(2),'bx')
plot(shs_output_final(1,:)+start_point(1),shs_output_final(2,:)+start_point(2))
hold off

%%


nr_particles = 200;

door_handle_use = ReferenceFile2Timetable('/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/lopen1/references.txt');


%%
clc

target = timetable(estimate1.Time);
target.est = [estimate1.est{:,:}]';

[og_positions, step_orient] = plotTrajectory(target,shs, door_handle_use);

%%
clc

[specific_pf, final_timestep] = ParticleFilter_inside(start_point,nr_particles, ...
                                                step_orient, 0.3, 0.15, walls, doors, door_handle_use);
disp(['pf completed:' num2str(final_timestep/height(step_orient))])

%%

gt_walkingroute =[];

for ii = 1 :1: length(specific_pf) 
    
    figure(1)
    show(walls)
    hold on
    if specific_pf(ii).door_detect == 1
    scatter([specific_pf(ii).particle_lists.x_pos]', [specific_pf(ii).particle_lists.y_pos]', '.r')
    set_point = ginput(1);    
    gt_walkingroute = [gt_walkingroute; seconds(specific_pf(ii).Time), set_point ];
    else
        scatter([specific_pf(ii).particle_lists.x_pos]', [specific_pf(ii).particle_lists.y_pos]', '.b')
    end
    title(['timestep: ' num2str(ii)])
    hold off
    

end
for ii = 1 :1: length(specific_pf) 
    
    figure(1)
    show(walls)
    hold on
    scatter([specific_pf(ii).particle_lists.x_pos]', [specific_pf(ii).particle_lists.y_pos]', '.')
    title(['itteration: ' num2str(ii)])
    hold off
end

%%
close all
h_fig = figure(1);
set(h_fig,'KeyPressFcn',{@myfun,specific_pf,walls});

function myfun(src,event,specific_pf,walls)
        persistent ii;
        if isempty(ii)
            ii = 0; 
        end
        ii = ii + 1;

        figure(1)
        set(gcf, 'Position', get(0, 'Screensize'));
        tiledlayout(2,1)

        % First plot
        ax1 = nexttile(1);
        show(walls)
        hold on
        scatter([specific_pf(ii).particle_lists.x_pos]', [specific_pf(ii).particle_lists.y_pos]', '.')
        title(['itteration: ' num2str(ii)])
        hold off

        % Second plot
        ax2 = nexttile(2);
        bar(specific_pf(ii).particle_lists.pre_resample_weight)

 

end




