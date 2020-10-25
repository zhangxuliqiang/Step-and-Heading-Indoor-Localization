function [output, utils] = ParticleFilter_inside_mat(start_point, ...
    nr_particles, step_orient, std_sl, ...
    std_orient, walls, doors, door_handle_use)

delta_angle = [0; diff(step_orient.yaw)];

limits = [walls.XLocalLimits',   walls.YLocalLimits'];

% using mat format for speed
[index.x_pos, index.y_pos, index.prev_x_pos, ...
 index.prev_y_pos, index.yaw, index.weight, index.pre_resample_weight] = ...
    feval(@(x) x{:}, num2cell(1:7));

utils.index = index;

% particle initilization
particle_list(:,index.x_pos) = start_point(1) * ones(nr_particles,1);
particle_list(:,index.y_pos) = start_point(2) * ones(nr_particles,1);
particle_list(:,index.prev_x_pos) = start_point(1) * ones(nr_particles,1);
particle_list(:,index.prev_y_pos) = start_point(2) * ones(nr_particles,1);
particle_list(:,index.yaw) = random('Uniform', 0, 2*pi, nr_particles, 1);
particle_list(:,index.weight) = 1/nr_particles * ones(nr_particles,1);
particle_list(:,index.pre_resample_weight) = zeros(nr_particles,1);

prev_time = seconds(0);

for timestep = 1: height(step_orient)
    
    % check whether the path is a valid one
    for i = 1:nr_particles
        particle = particle_list(i,:);
        cur_pos = [particle(index.x_pos), particle(index.y_pos)];
        
        % keep position within range of the map
        cur_pos = min(max(cur_pos,limits(1,:)),limits(2,:));
        prev_pos = [particle(index.prev_x_pos), particle(index.prev_y_pos)];
        prev_pos = min(max(prev_pos,limits(1,:)),limits(2,:));
        
        
        if isequal(cur_pos, prev_pos)
            invalid_points(i,1) = logical(checkOccupancy(walls,cur_pos));
        else
            try [endpoints,midpoints] = raycast(walls,cur_pos,prev_pos);
            catch ME
                disp('      shit is wild')      
            end
            valid_path = logical(checkOccupancy(walls,[endpoints;midpoints],"grid"));
            invalid_points(i,1) = any(valid_path);
        end
    end
    
    if(sum(~invalid_points) == 0 )
        break
    end
    % normalize weighting over particles
    particle_list = MeasUpdate(particle_list, index, ~invalid_points);
    
    output(timestep,:).door_detect = 0;
    
    cur_time  = step_orient(timestep,:).Time;
    range_of_times = timerange(prev_time,cur_time);
    
    [~, which_rows] = withinrange(door_handle_use, range_of_times);
    
    if sum(which_rows)>0
        for i = 1:nr_particles
            particle = particle_list(i,:);
            mean = [particle(index.x_pos), particle(index.y_pos)];
            covariance = [0.2 0; 0 0.2];
            particle_door_pdf = mvnpdf(doors,mean,covariance);
            distance_weighting(i,1) = max(particle_door_pdf);
        end
        particle_list = MeasUpdate(particle_list, index, distance_weighting);
        output(timestep,:).door_detect = 1;
    end
    
    % estimate using highest weighting
    highest_weight = max(particle_list(:,index.weight));
    highest_index = particle_list(:,index.weight) == highest_weight;
    high_weight_particles = particle_list(highest_index,:);
    x_pos_mean = mean(high_weight_particles(:,index.x_pos));
    y_pos_mean = mean(high_weight_particles(:,index.y_pos));
    
    output(timestep,:).estimate  = [x_pos_mean, y_pos_mean];
     
    particle_list = Resample(particle_list, index);
    
    % estimate using mean
    
%     x_pos_mean = mean(particle_list(:,index.x_pos));
%     y_pos_mean = mean(particle_list(:,index.y_pos));
%     
%     output(timestep,:).estimate  = [x_pos_mean, y_pos_mean];
    % time update
    
    sl_noise = random('Normal', 0.20, std_sl, length(particle_list), 1);
    orientation_noise = random('Normal', 0, std_orient, length(particle_list), 1);
    
    sl_realization = step_orient.step_length(timestep) + sl_noise;
    orient_realization = particle_list(:,index.yaw) + delta_angle(timestep) + orientation_noise;
    
    particle_list(:, index.prev_x_pos) = particle_list(:,index.x_pos);
    particle_list(:, index.prev_y_pos) = particle_list(:,index.y_pos);
    
    particle_list(:,index.yaw) = orient_realization;
    particle_list(:,index.x_pos) = particle_list(:,index.x_pos) + cos(orient_realization).* sl_realization;
    
    particle_list(:,index.y_pos) = particle_list(:,index.y_pos) + sin(orient_realization).* sl_realization;
    
    prev_time  = step_orient(timestep,:).Time;
    output(timestep,:).Time = step_orient(timestep,:).Time;
    output(timestep,:).particle_lists = particle_list;
end
output = struct2table(output);
output = table2timetable(output);
utils.final_timestep = timestep;
end

function particle_list = Resample(particle_list, index)
% Systematic sampling sampling  with sort
% Source: Gustafsson - 2010 - Particle Filter Theory and Practice with Positioning

N = length(particle_list);

u=([0:N-1]'+(rand(N,1)))/N;
wc=cumsum(particle_list(:,index.weight)) ;
wc=wc/wc(N) ;
[~,ind1]=sort([u;wc]);
ind2=find(ind1<=N);
resample_index = ind2-(0:N-1)';

dead_particles  = find(particle_list(:,index.weight) == 0);

for i=1:length(dead_particles)
  isequal =(dead_particles(i,1)== resample_index);
  if(sum(isequal)> 0)
      disp('particle resurection')
  end
end

particle_list = particle_list(resample_index,:);
particle_list(:,index.pre_resample_weight) = particle_list(:,index.weight);
particle_list(:,index.weight) = ones(N,1)/N;
end

function particle_list = MeasUpdate(particle_list, index, post_prob)
normalization_weight = sum(particle_list(:,index.weight).*post_prob);
particle_list(:,index.weight) = (1/normalization_weight) * particle_list(:,index.weight).* post_prob;
end

