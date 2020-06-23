
%% generating particle filter map

% defining the scale of the map
image = imread('pietheinstraatv2.png');
I = rgb2gray(image);
white = I < 250;

[row_size, column_size] = size(white);

Red = image(:,:,2)<10;

[row,col] = find(Red);

start_point = [row_size - mean(row),mean(col)];

white(row,col) = 0;

fig = figure(1);
imshow(white)



hold on
% scale_points = ginput(2);
hold off
close(fig)
% 
points_per_meter = diff(scale_points(:,1))/20;

points_per_meter = 2.65;
map = occupancyMap(white,points_per_meter);
start_point_meter = start_point/points_per_meter;
%%
figure()
hold on
show(map)
scatter(start_point_meter(2),start_point_meter(1),'bx')
hold off

%% distribute particles over map

nr_particles = 100;
particle_lists = {};

delta_angle = [0; diff(step_orient.yaw)];

% particle initilization

particle_list = repmat(struct('x', start_point_meter(2), ...
        'y',start_point_meter(1), 'yaw', step_orient(1,:).yaw), nr_particles, 1 );

    
for timestep = 1: height(step_orient)
    
    disp(['pf completed:' num2str(timestep/height(step_orient))])
    
    % measurement update
    invalid_points = logical(checkOccupancy(map,[particle_list.x; particle_list.y]'));
    particle_list = particle_list(~invalid_points);
    
    % resampling
    new_particle_list = [];
      
    new_particle_index = randi(length(particle_list),nr_particles,1);
    particle_list = particle_list(new_particle_index);
 
    % time update
        
    sl_noise = random('Normal', 0.25, 0.2, nr_particles, 1);
    orientation_noise = random('Normal', 0, 0.4, nr_particles, 1);

    sl_realization = shs.steps.data.step_length(timestep) + sl_noise;
    orient_realization = [particle_list.yaw]'+ delta_angle(timestep) + orientation_noise;

    temp_angle = num2cell(orient_realization);    
    
    temp_x = num2cell([particle_list.x]' + cos(orient_realization).* sl_realization);
    
    temp_y = num2cell([particle_list.y]' + sin(orient_realization).* sl_realization');
    
    [particle_list.yaw] = temp_angle{:};
    [particle_list.x] = temp_x{:};
    [particle_list.y] = temp_y{:};
    
    particle_lists(timestep,:) = {particle_list};  

    
end

%%

for i = 1 :5: length(particle_lists)
figure(1)
show(map)
hold on
scatter([particle_lists{i}.x]', [particle_lists{i}.y]', '.')
hold off
end
