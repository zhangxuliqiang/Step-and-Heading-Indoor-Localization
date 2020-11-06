%% generating particle filter map

% defining the scale of the map
image = imread('binnenkaart (copy).png');
not_white_regions_image  = rgb2gray(image)< 250;

[row_size, column_size] = size(not_white_regions_image);

red_regions_image = image(:,:,1)>250 & image(:,:,3)<50 & image(:,:,2)<50 ;

walls_image = not_white_regions_image ~= red_regions_image;

points_per_meter = 90; %<-- this value was generated through pixel to meter
                       %    conversion inside plotter file

walls = occupancyMap(walls_image,points_per_meter);

door_pixel_location = bwboundaries(red_regions_image);

doors = grid2local(walls, cell2mat(door_pixel_location));


fig = figure(); 
show(walls)
title('choose the starting point')
hold on
start_point = ginput(1);
hold off
close(fig)

%% Generate shs output

pf1.sample_name = 'lopen1.1';
pf1.shs = shs_estimation(pf1.sample_name);
pf1.gt = importVideoGroundTruth([pf1.sample_name '_gt_from_video.csv']);
pf1.doors = doors;
pf1.walls = walls;
pf1.std_sl = 0.18;
pf1.std_orient = 0.16;

pf2.sample_name = 'lopen1.2';
pf2.shs = shs_estimation(pf2.sample_name);
pf2.gt = importVideoGroundTruth([pf2.sample_name '_gt_from_video.csv']);
pf2.doors = doors;
pf2.walls = walls;
pf2.std_sl = 0.18;
pf2.std_orient = 0.16;

pf3.sample_name = 'lopen1.3';
pf3.shs = shs_estimation(pf3.sample_name);
pf3.gt = importVideoGroundTruth([pf3.sample_name '_gt_from_video.csv']);
pf3.doors = doors;
pf3.walls = walls;
pf3.std_sl = 0.18;
pf3.std_orient = 0.16;

pf5.sample_name = 'lopen1.5';
pf5.shs = shs_estimation(pf5.sample_name);
pf5.gt = importVideoGroundTruth([pf5.sample_name '_gt_from_video.csv']);
pf5.doors = doors;
pf5.walls = walls;
pf5.std_sl = 0.18;
pf5.std_orient = 0.16;

all_pf=[pf1,pf2,pf3];

%% finding completed track
clc
pf = pf1;
nr_particles = 2000;
% holding = [];
for i=1:7
    tic
    [pf.specific_pf, utils] = ParticleFilter_inside_mat(start_point,nr_particles, ...
        pf.std_sl, pf.std_orient, pf);
    disp([pf.sample_name 'pf completed:' num2str(utils.final_timestep/height(pf.shs.step_and_orient))])
    toc
    
    if utils.final_timestep/height(pf.shs.step_and_orient) == 1
        [pf.distance_rmse, pf.mean_traj] = CompareToGroundTruth(pf.shs,pf.specific_pf, utils, pf.gt);
        disp(['rmse: ' pf.distance_rmse])
        
        holding = [holding,pf];
    end
end

%%
pf = pf1;
finding_nr_particles = [];
% nr_particles = 1000;
for nr_particles = 1000:1000:5000
    for i = 1:5
        disp([pf.sample_name ': nr particles: ' num2str(nr_particles) ' itteration: ' num2str(i) ])
        tic
        [pf.specific_pf, utils] = ParticleFilter_inside_mat(start_point,nr_particles, ...
            pf.std_sl, pf.std_orient, pf);
        disp([pf.sample_name 'pf completed:' num2str(utils.final_timestep/height(pf.shs.step_and_orient))])
        toc
        pf.distance_rmse = nan;
        pf.mean_traj = nan;
        if utils.final_timestep/height(pf.shs.step_and_orient) == 1
            [pf.distance_rmse,pf.mean_traj] = CompareToGroundTruth(pf.shs,pf.specific_pf, utils, pf.gt);
        end
    finding_nr_particles = [finding_nr_particles;nr_particles,pf.distance_rmse];    
    end    
end

save([time '_' pf.sample_name  '_nr_particle_results.mat'],'finding_nr_particles')

%% finding completed track for a range
clc
nr_particles = 4000;
results = [];

for index =  1:length(all_pf)
    pf = all_pf(index);
    complete = 0;
    for i = 1:5
            disp([pf.sample_name ': itteration: ' num2str(i) ])
            tic
            [pf.specific_pf, utils] = ParticleFilter_inside_mat(start_point,nr_particles, ...
                pf.std_sl, pf.std_orient, pf);
            disp([pf.sample_name 'pf completed:' num2str(utils.final_timestep/height(pf.shs.step_and_orient))])
            toc
            pf.distance_rmse = nan;
            pf.mean_traj = nan;
            if utils.final_timestep/height(pf.shs.step_and_orient) == 1
            [pf.distance_rmse,pf.mean_traj] = CompareToGroundTruth(pf.shs,pf.specific_pf, utils, pf.gt);
            complete = complete + 1;
            disp(['completed: ' complete ' for ' pf.sample_name ...
               'with rmse: ' num2str(pf.distance_rmse) ])
            end
        
        results = [results;pf];
    end
end
time = datestr(now,'yyyymmdd_HHMM');
save([time '_range_three_trials_results_no_doors.mat'],'results')

%%
mean_traj = results(3).mean_traj;

figure
hold on
plot(mean_traj(:,1),mean_traj(:,2), 'r')
plot(pf1.gt.x,pf1.gt.y,'b')
hold off

%%
clc

pf = pf5;
itteration =[];
nr_particles = 600;
std_orient_counter = 0;

orient_pf = [];

for std_orient = 0.14:0.02:0.2
    std_orient_counter = std_orient_counter +1;
    
    fprintf('std_orient = %f \n', std_orient )
    
    sl_pf = [];
    sl_std_counter = 0;
    
    for std_sl = 0.16:0.02:0.2
        sl_std_counter = sl_std_counter +1;
        fprintf('       std_sl = %f \n', std_sl )
        
        realizations =repmat(struct('percent_complete', nan, 'distance_rmse', nan),5,1);
        
        for itteration = 1:5
            fprintf('       itteration: %i \n' ,itteration )
            
% [output, utils] = ParticleFilter_inside_mat(start_point, ...
%     nr_particles, std_sl, std_orient, pf)
             [specific_pf, utils] = ParticleFilter_inside_mat(start_point,nr_particles, ...
                std_sl, std_orient, pf);
            disp([pf.sample_name ' std_orient: ' num2str(std_orient) ' std_sl: ' num2str(std_sl) ...
                ' pf completed:' num2str(utils.final_timestep/height(pf.shs.step_and_orient))])
            
            realizations(itteration).percent_complete = utils.final_timestep/height( pf.shs.step_and_orient);
            
            if realizations(itteration).percent_complete == 1
                [realizations(itteration).distance_rmse, ~ ] =  CompareToGroundTruth(pf.shs, specific_pf, utils, pf.gt);
            else
                realizations(itteration).distance_rmse = nan; 
            end
        end
        sl_pf(sl_std_counter).std_sl = std_sl;
        sl_pf(sl_std_counter).realizations = realizations;
        sl_pf(sl_std_counter).completed = sum([realizations.percent_complete] == 1);
    end
    orient_pf(std_orient_counter).std_orient = std_orient;
    orient_pf(std_orient_counter).sl_pf = sl_pf;
end
save([pf.sample_name 'parameter search.mat'],'orient_pf')
