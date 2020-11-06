function [output, final_timestep] = ParticleFilter(start_point_meter, ...
                                    nr_particles, step_orient, std_sl, ...
                                    std_orient, map)

delta_angle = [0; diff(step_orient.yaw)];

% particle initilization
particle_list = repmat(struct('x_pos', start_point_meter(2), ...
    'y_pos',start_point_meter(1), 'yaw', step_orient(1,:).yaw), nr_particles, 1 );

output = [];

for timestep = 1: height(step_orient)
    
    % measurement update
    invalid_points = logical(checkOccupancy(map,[particle_list.x_pos; particle_list.y_pos]'));
    particle_list = particle_list(~invalid_points);
    
    % if there are no valid points break, and continue to next
    % itteration
    
    if(size(particle_list,1) == 0 )
        break
    end
    
    % resampling    
        
        new_particle_index = randi(length(particle_list),nr_particles,1);
        particle_list = particle_list(new_particle_index);
    
    % time update
    
    sl_noise = random('Normal', 0.20, std_sl, length(particle_list), 1);
    orientation_noise = random('Normal', 0, std_orient, length(particle_list), 1);
    
    sl_realization = step_orient.step_length(timestep) + sl_noise;
    orient_realization = [particle_list.yaw]'+ delta_angle(timestep) + orientation_noise;
    
    temp_angle = num2cell(orient_realization);
    
    temp_x = num2cell([particle_list.x_pos]' + cos(orient_realization).* sl_realization);
    
    temp_y = num2cell([particle_list.y_pos]' + sin(orient_realization).* sl_realization);
    
    [particle_list.yaw] = temp_angle{:};
    [particle_list.x_pos] = temp_x{:};
    [particle_list.y_pos] = temp_y{:};
    
    output(timestep,:).Time = step_orient(timestep,:).Time;
    output(timestep,:).particle_lists = particle_list;
end
final_timestep = timestep;
end