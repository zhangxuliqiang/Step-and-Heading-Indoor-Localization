function [particle_lists, final_timestep] = ParticleFilter(start_point_meter, nr_particles, step_orient, delta_sl, map)

delta_angle = [0; diff(step_orient.yaw)];

% particle initilization
particle_list = repmat(struct('x', start_point_meter(2), ...
    'y',start_point_meter(1), 'yaw', step_orient(1,:).yaw), nr_particles, 1 );

particle_lists = {};

for timestep = 1: height(step_orient)
    
    % measurement update
    invalid_points = logical(checkOccupancy(map,[particle_list.x; particle_list.y]'));
    particle_list = particle_list(~invalid_points);
    
    % if there are no valid points break, and continue to next
    % itteration
    
    if(size(particle_list,1) == 0 )
        break
    end
    
    % resampling
    
    if (length(particle_list)/nr_particles) < (2/3)
        
        new_particle_index = randi(length(particle_list),nr_particles,1);
        particle_list = particle_list(new_particle_index);
    end
    
    % time update
    
    sl_noise = random('Normal', 0.20, delta_sl, length(particle_list), 1);
    orientation_noise = random('Normal', 0, 0.05, length(particle_list), 1);
    
    sl_realization = step_orient.step_length(timestep) + sl_noise;
    orient_realization = [particle_list.yaw]'+ delta_angle(timestep) + orientation_noise;
    
    temp_angle = num2cell(orient_realization);
    
    temp_x = num2cell([particle_list.x]' + cos(orient_realization).* sl_realization);
    
    temp_y = num2cell([particle_list.y]' + sin(orient_realization).* sl_realization);
    
    [particle_list.yaw] = temp_angle{:};
    [particle_list.x] = temp_x{:};
    [particle_list.y] = temp_y{:};
    
    particle_lists(timestep,:) = {particle_list};
end
final_timestep = timestep;
end