function [output, final_timestep] = ParticleFilterSysResample(start_point_meter, ...
                                    nr_particles, step_orient, std_sl, ...
                                    std_orient, map, gps_tol)

delta_angle = [0; diff(step_orient.yaw)];

% particle initilization
x_pos = start_point_meter(2) * ones(nr_particles,1);
y_pos = start_point_meter(1) * ones(nr_particles,1);
yaw = step_orient(1,:).yaw * ones(nr_particles,1);
weight = 1/nr_particles * ones(nr_particles,1);

 particle_list = table(x_pos, y_pos, yaw, weight);
% particle_list = repmat(struct('x_pos', start_point_meter(2), ...
%                               'y_pos',start_point_meter(1), ...
%                               'yaw', step_orient(1,:).yaw, ...
%                               'weight', 1/nr_particles), ...
%                               nr_particles, 1 );

output = [];

for timestep = 1: height(step_orient)
    
       
    %# measurement update
    % first check whether the particles are outside passable region
    invalid_points = logical(checkOccupancy(map,[particle_list.x_pos, particle_list.y_pos]));
    normalization_weight = sum(particle_list.weight.* ~invalid_points);
    
    if isnan(normalization_weight) || normalization_weight == 0
        error('normalization is nan or 0')
    end
    
    % if there are no valid points break, and continue to next
    % itteration
    
    if(normalization_weight == 0 )
        break
    end
    
    particle_list.weight = (1/normalization_weight) * particle_list.weight.* ~invalid_points;
    
%     if sum((gps_tol.Time - step_orient(timestep,:).Time) < 0.01) == 1
%         dist = 1;
%     end

    % resampling
    resample_index = [];
    N = height(particle_list);
    Q = cumsum(particle_list.weight);
    
    T = ([1:N]-1 + rand(1))/N;
    
    i=1;
    j=1;
    
    while (i<=N)
        if (T(i)<Q(j))
            resample_index(i)=j;
            i=i+1;
        else
            j=j+1;
        end
        
        if j > N
            error('j is larger than N')
        end
    end

particle_list = particle_list(resample_index,:);
    
         
    % time update
    
    sl_noise = random('Normal', 0.20, std_sl, height(particle_list), 1);
    orientation_noise = random('Normal', 0, std_orient, height(particle_list), 1);
    
    sl_realization = step_orient.step_length(timestep) + sl_noise;
    orient_realization = particle_list.yaw+ delta_angle(timestep) + orientation_noise;
    
    particle_list.yaw = orient_realization;
    particle_list.x_pos = particle_list.x_pos + cos(orient_realization).* sl_realization;
    
    particle_list.y_pos = particle_list.y_pos + sin(orient_realization).* sl_realization;
        
    output(timestep,:).Time = step_orient(timestep,:).Time;
    output(timestep,:).particle_lists = particle_list;
end
final_timestep = timestep;
end