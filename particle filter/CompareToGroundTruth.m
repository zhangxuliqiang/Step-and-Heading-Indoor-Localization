function [mean_error, mean_std_error] = CompareToGroundTruth(specific_pf, utils, gt_walkingroute)

pf_door_open_index = find(specific_pf.door_detect);

pf_door_open = specific_pf(pf_door_open_index,:); 

pf_dist_error = [];

for i = 1:height(pf_door_open)
    particle_list = pf_door_open(i,:).particle_lists{1,1};
    pf_x_positions = particle_list(:,utils.index.x_pos);
    pf_y_positions = particle_list(:,utils.index.y_pos);
    gt_point = gt_walkingroute(i,:); 
    
    x_pos_error = pf_x_positions - gt_point.x_pos;
    y_pos_error = pf_y_positions - gt_point.y_pos;
    
    error_vec = [x_pos_error'; y_pos_error'];
    
    distance_error = vecnorm(error_vec);
    
    pf_dist_error(i).Time = specific_pf(i,:).Time;
    pf_dist_error(i).mean = mean(distance_error);
    pf_dist_error(i).std = std(distance_error);
    
end
if ~isempty(pf_dist_error)
    pf_stat.dist_error = struct2table(pf_dist_error);
    mean_error  = mean(pf_stat.dist_error.mean);
    mean_std_error = mean(pf_stat.dist_error.std);
else
mean_error = nan;
mean_std_error = nan;
end

end