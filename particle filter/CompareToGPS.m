function [pf_mean_error,pf_mean_std_error] = CompareToGPS(specific_pf,gps_data)

for i = 1:length(specific_pf)
    pf_x_positions = specific_pf(i).particle_lists.x_pos;
    pf_y_positions = specific_pf(i).particle_lists.y_pos;
    gps_point = gps_data(specific_pf(i).Time,:);
    
    x_pos_error = pf_x_positions - gps_point.x_pos;
    y_pos_error = pf_y_positions - gps_point.y_pos;
    
    error_vec = [x_pos_error'; y_pos_error'];
    
    distance_error = vecnorm(error_vec);
    
    pf_dist_error(i).Time = specific_pf(i).Time;
    pf_dist_error(i).mean = mean(distance_error);
    pf_dist_error(i).std = std(distance_error);
    
end

pf_dist_error = struct2table(pf_dist_error);
pf_mean_error  = mean(pf_dist_error.mean);
pf_mean_std_error = mean(pf_dist_error.std);
end