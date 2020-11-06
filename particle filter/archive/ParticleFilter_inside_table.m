function [output, final_timestep] = ParticleFilter_inside(start_point, ...
    nr_particles, step_orient, std_sl, ...
    std_orient, walls, doors, door_handle_use)

delta_angle = [0; diff(step_orient.yaw)];

% particle initilization
x_pos = start_point(1) * ones(nr_particles,1);
y_pos = start_point(2) * ones(nr_particles,1);
prev_x_pos = start_point(1) * ones(nr_particles,1);
prev_y_pos = start_point(2) * ones(nr_particles,1);
yaw = random('Uniform', 0, 2*pi, nr_particles, 1);
weight = 1/nr_particles * ones(nr_particles,1);
pre_resample_weight = zeros(nr_particles,1);
door_detect = zeros(nr_particles,1);

particle_list = table(x_pos, y_pos, prev_x_pos, prev_y_pos, ...
                      yaw, weight, pre_resample_weight, door_detect);

prev_time = seconds(0);

output = [];

for timestep = 1: height(step_orient)
    
    % check whether the path is a valid one
    for i = 1:nr_particles
        particle = particle_list(i,:);
        cur_pos = [particle.x_pos, particle.y_pos];
        prev_pos = [particle.prev_x_pos, particle.prev_y_pos];
        
        if isequal(cur_pos, prev_pos)
            invalid_points(i,1) = logical(checkOccupancy(walls,cur_pos));
        else
            [endpoints,midpoints] = raycast(walls,cur_pos,prev_pos);
            valid_path = logical(checkOccupancy(walls,[endpoints;midpoints],"grid"));
            invalid_points(i,1) = any(valid_path);
        end
    end
    
    if(sum(~invalid_points) == 0 )
        break
    end
    % normalize weighting over particles
    particle_list = MeasUpdate(particle_list, ~invalid_points);
    
    cur_time  = step_orient(timestep,:).Time;
    range_of_times = timerange(prev_time,cur_time);
    
    [~, which_rows] = withinrange(door_handle_use, range_of_times);
    
    if sum(which_rows)>0
        for i = 1:nr_particles
            particle = particle_list(i,:);
            mean = [particle.x_pos, particle.y_pos];
            covariance = [0.5 0; 0 0.5];
            particle_door_pdf = mvnpdf(doors,mean,covariance);
            distance_weighting(i,1) = max(particle_door_pdf);
        end
        particle_list = MeasUpdate(particle_list, distance_weighting);
        output(timestep,:).door_detect = 1;
    end
     
    particle_list = Resample(particle_list);
    
    % time update
    
    sl_noise = random('Normal', 0.20, std_sl, height(particle_list), 1);
    orientation_noise = random('Normal', 0, std_orient, height(particle_list), 1);
    
    sl_realization = step_orient.step_length(timestep) + sl_noise;
    orient_realization = particle_list.yaw + delta_angle(timestep) + orientation_noise;
    
    particle_list.prev_x_pos = particle_list.x_pos;
    particle_list.prev_y_pos = particle_list.y_pos;
    
    particle_list.yaw = orient_realization;
    particle_list.x_pos = particle_list.x_pos + cos(orient_realization).* sl_realization;
    
    particle_list.y_pos = particle_list.y_pos + sin(orient_realization).* sl_realization;
    
    prev_time  = step_orient(timestep,:).Time;
    output(timestep,:).Time = step_orient(timestep,:).Time;
    output(timestep,:).particle_lists = particle_list;
end
final_timestep = timestep;
end

function particle_list = Resample(particle_list)
% Systematic sampling sampling  with sort
% Source: Gustafsson - 2010 - Particle Filter Theory and Practice with Positioning

N = height(particle_list);

u=([0:N-1]'+(rand(N,1)))/N;
wc=cumsum(particle_list.weight) ;
wc=wc/wc(N) ;
[~,ind1]=sort([u;wc]);
ind2=find(ind1<=N);
resample_index = ind2-(0:N-1)';

dead_particles  = find(particle_list.weight == 0);

for i=1:length(dead_particles)
  isequal =(dead_particles(i,1)== resample_index);
  if(sum(isequal)> 0)
      disp('particle resurection')
  end
end

particle_list = particle_list(resample_index,:);
particle_list.pre_resample_weight = particle_list.weight;
particle_list.weight = ones(N,1)/N;
end

function particle_list = MeasUpdate(particle_list, post_prob)
normalization_weight = sum(particle_list.weight.*post_prob);

% if any(~post_prob)
%     disp('gone through wall')
% end
particle_list.weight = (1/normalization_weight) * particle_list.weight.* post_prob;
end