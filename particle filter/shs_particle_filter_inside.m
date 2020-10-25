%% generating particle filter map


% defining the scale of the map
image = imread('binnenkaart (copy).png');
white_regions_image  = rgb2gray(image)< 250;

[row_size, column_size] = size(white_regions_image);

red_regions_image = image(:,:,1)>250 & image(:,:,3)<50 & image(:,:,2)<50 ;

walls_image = white_regions_image ~= red_regions_image;

points_per_meter = 90; %<-- this value was generated through pixel to meter
                       %    conversion inside plotter file

walls = occupancyMap(walls_image,points_per_meter);

door_pixel_location = bwboundaries(red_regions_image);

doors = grid2local(walls, cell2mat(door_pixel_location));

%%

fig = figure(); 
show(walls)
title('choose the starting point')
hold on
start_point = ginput(1);
hold off
close(fig)


%% finding completed track
clc

nr_particles = 300;

while true
    tic
    [specific_pf, utils] = ParticleFilter_inside_mat(start_point,nr_particles, ...
        shs.step_and_orient, 0.2, 0.2, walls, doors, shs_sample.door_handle_use);
    disp(['pf completed:' num2str(utils.final_timestep/height(shs.step_and_orient))])
    toc
    
    if utils.final_timestep/height(shs.step_and_orient) > 0.90
        break
    end

end

%% Individual testing

 [specific_pf, utils] = ParticleFilter_inside_mat(start_point,nr_particles, ...
         shs.step_and_orient, 0.3, 0.1, walls, doors, shs_sample.door_handle_use);
 disp(['pf completed:' num2str(utils.final_timestep/height( shs.step_and_orient))])
    %%
    clc
pf_stat = CompareToGroundTruth(specific_pf,utils, pos_data)

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
    
    for std_sl = 0.05:0.05:0.3
        sl_std_counter = sl_std_counter +1;
        fprintf('       std_sl = %f \n', std_sl )
        realizations =[];
        
        for itteration = 1:10
            fprintf('       itteration: %i' ,itteration )
            
% [output, utils] = ParticleFilter_inside_mat(start_point, nr_particles,  shs.step_and_orient, std_sl, std_orient, walls, doors, door_handle_use)           
            [specific_pf, utils] = ParticleFilter_inside_mat(start_point,nr_particles, ...
                shs.step_and_orient, std_sl, std_orient, walls, doors, shs_sample.door_handle_use);
            disp(['       pf completed:' num2str(utils.final_timestep/height( shs.step_and_orient))])
            
            realizations(itteration).percent_complete = utils.final_timestep/height( shs.step_and_orient);
            
            [realizations(itteration).mean_error, realizations(itteration).mean_std_error] =  ...
                CompareToGroundTruth(specific_pf,utils, pos_data);
        end
        sl_pf(sl_std_counter).std_sl = std_sl;
        sl_pf(sl_std_counter).realizations = realizations;
        sl_pf(sl_std_counter).completed = sum([realizations.percent_complete] == 1);
    end
    orient_pf(std_orient_counter).std_orient = std_orient;
    orient_pf(std_orient_counter).sl_pf = sl_pf;
end
save(orient_pf)
%%

plot_index = 1;
sub_plot_length = length(orient_pf) + 1;
std_sl_x_axis  = 0.05:0.05:0.3; 

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
    ylim([0,10])
    
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




