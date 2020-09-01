function [output, final_timestep] = ParticleFilterSysResample(start_point_meter, ...
    nr_particles, step_orient, std_sl, ...
    std_orient, map, gps_data)

delta_angle = [0; diff(step_orient.yaw)];

% particle initilization
x_pos = start_point_meter(2) * ones(nr_particles,1);
y_pos = start_point_meter(1) * ones(nr_particles,1);
yaw = step_orient(1,:).yaw * ones(nr_particles,1);
weight = 1/nr_particles * ones(nr_particles,1);

particle_list = table(x_pos, y_pos, yaw, weight);

% GPS half normal distribution

dist_prob = makedist('HalfNormal','mu',0,'sigma',2);

output = [];

for timestep = 1: height(step_orient)
    
    particle_positions = [particle_list.x_pos,particle_list.y_pos];
    
    %# measurement update
    % first check whether the particles are outside passable region
    invalid_points = logical(checkOccupancy(map,particle_positions));
        
    if(sum(~invalid_points) == 0 )
        break
    end
    
    particle_list = MeasUpdate(particle_list, ~invalid_points);
    
    gps_index = find(gps_data.Time == step_orient(timestep,:).Time,1);
    
    if ~isempty(gps_index)
         
         gps_position = [gps_data(gps_index,:).x_pos, gps_data(gps_index,:).y_pos];        
         dist_to_gps = vecnorm(particle_positions - gps_position,2,2);
         distance_weighting = pdf(dist_prob,dist_to_gps);
         
         particle_list = MeasUpdate(particle_list, distance_weighting);
         
    end   
       
    particle_list = Resample(particle_list);
    
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

function particle_list = Resample(particle_list) 
% Systematic sampling sampling  with sort 
% Source: Gustafsson - 2010 - Particle Filter Theory and Practice with Positioning

N = height(particle_list);

u=([0:N-1]'+(rand(N,1)))/N;
wc=cumsum(particle_list.weight) ; 
wc=wc/wc(N) ; 
[dum,ind1]=sort([u;wc]); 
ind2=find(ind1<=N); 
resample_index = ind2-(0:N-1)'; 

particle_list = particle_list(resample_index,:);
particle_list.weight = ones(N,1)/N;
end

function particle_list = MeasUpdate(particle_list, post_prob)
    normalization_weight = sum(particle_list.weight.*post_prob);
    particle_list.weight = (1/normalization_weight) * particle_list.weight.* post_prob;
end