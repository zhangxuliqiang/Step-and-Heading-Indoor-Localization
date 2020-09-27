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
    
%% ground truth generation
% 
% gt_walkingroute =[];

for ii = 1 :1: height(specific_pf) 
    particle_list = specific_pf(ii,:).particle_lists{1,1};
    figure(1)
    show(walls)
    hold on
    if specific_pf(ii,:).door_detect == 1
    scatter(particle_list(:,utils.index.x_pos), particle_list(:,utils.index.y_pos), '.r')
%     set_point = ginput(1);    
%     gt_walkingroute = [gt_walkingroute; seconds(specific_pf(ii).Time), set_point ];
    else
        scatter(particle_list(:,utils.index.x_pos), particle_list(:,utils.index.y_pos), '.b')
    end
    title(['timestep: ' num2str(ii)])
    hold off
    

end

%%
clc
gt_walkingroute = timetable(gt_walkingroute(:,2), ...
    gt_walkingroute(:,3), 'RowTimes', seconds(gt_walkingroute(:,1)), ...
    'VariableNames',{'x_pos','y_pos'});

%% Individual testing

 [specific_pf, utils] = ParticleFilter_inside_mat(start_point,nr_particles, ...
        step_orient, 0.3, 0.15, walls, doors, door_handle_use);
 disp(['pf completed:' num2str(utils.final_timestep/height(step_orient))])
    %%
    clc
pf_stat = CompareToGroundTruth(specific_pf, utils, gt_walkingroute)

%%
clc
nr_particles = 100;
std_orient_counter = 0;

orient_pf = [];

for std_orient = 0.05:0.05:0.3
    std_orient_counter = std_orient_counter +1;
    
    fprintf('std_orient = %f \n', std_orient )
    
    sl_pf = [];
    sl_std_counter = 0;
    
    for std_sl = 0.1:0.05:0.3
        sl_std_counter = sl_std_counter +1;
        fprintf('       std_sl = %f \n', std_sl )
        realizations =[];
        
        for itteration = 1:10
            fprintf('       itteration: %i' ,itteration )
            
% [output, utils] = ParticleFilter_inside_mat(start_point, nr_particles, step_orient, std_sl, std_orient, walls, doors, door_handle_use)           
            [specific_pf, utils] = ParticleFilter_inside_mat(start_point,nr_particles, ...
                step_orient, std_sl, std_orient, walls, doors, door_handle_use);
            disp(['       pf completed:' num2str(utils.final_timestep/height(step_orient))])
            
            realizations(itteration).percent_complete = utils.final_timestep/height(step_orient);
            
            [realizations(itteration).mean_error, realizations(itteration).mean_std_error] =  ...
                CompareToGroundTruth(specific_pf, utils, gt_walkingroute);
        end
        sl_pf(sl_std_counter).std_sl = std_sl;
        sl_pf(sl_std_counter).realizations = realizations;
        sl_pf(sl_std_counter).completed = sum([realizations.percent_complete] == 1);
    end
    orient_pf(std_orient_counter).std_orient = std_orient;
    orient_pf(std_orient_counter).sl_pf = sl_pf;
end
%%

plot_index = 1;
sub_plot_length = length(orient_pf) + 1;
std_sl_x_axis  = 0.1:0.1:0.5; 

 t = tiledlayout(length(orient_pf),1);
 
for i = 1:length(orient_pf)
    
    specific_sl_pf = orient_pf(i).sl_pf;
    
    bc_mean_error = [];
    bc_mean_std = [];
    
    for std_sl_index = 1:length(specific_sl_pf)
        track_completed_index = [specific_sl_pf(std_sl_index).realizations.percent_complete] == 1;
        bc_mean_error = [bc_mean_error; [specific_sl_pf(std_sl_index).realizations.mean_error].*track_completed_index];
        bc_mean_std = [bc_mean_std; [specific_sl_pf(std_sl_index).realizations.mean_std_error].*track_completed_index];
    end
    
    ax(i) = nexttile;
    b = bar(ax(i),std_sl_x_axis,bc_mean_error);
    title(['orientation std: '  num2str(orient_pf(i).std_orient)])
    set(ax(i),'fontsize',10)
    ylim([0,100])
    
    xtips2 = b(i).XEndPoints-0.005;
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


%% video maker

 writerObj = VideoWriter('myVideo1.avi');
 writerObj.FrameRate = 10;

 % open the video writer
 open(writerObj);

for ii = 1 :1: length(specific_pf) 
    
    figure(1)
    show(walls)
    hold on
    if specific_pf(ii).door_detect == 1
    scatter([specific_pf(ii).particle_lists.x_pos]', [specific_pf(ii).particle_lists.y_pos]', '.r')
    else
        scatter([specific_pf(ii).particle_lists.x_pos]', [specific_pf(ii).particle_lists.y_pos]', '.b')
    end
    title(['itteration: ' num2str(ii)])
    hold off
    
    frame = getframe(figure(1));
    writeVideo(writerObj, frame);
end

 % close the writer object
 close(writerObj);

%% step by step movement
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




