function [distance_rmse,mean_traj] = CompareToGroundTruth(shs,specific_pf, utils, pos_data)

all_bt_trajectories_x = [];
all_bt_trajectories_y = [];
for ancestor = 1:10:length(specific_pf.particle_lists{1,1})
    backtrack_trajectory = [];
    for ii = height(specific_pf):-1:1
        particle_list = specific_pf(ii,:).particle_lists{1,1};
        backtrack_trajectory = [backtrack_trajectory;particle_list(ancestor,utils.index.x_pos:utils.index.y_pos)];
        % finding the next ancestor
        ancestor  = particle_list(ancestor,utils.index.particle_history);
    end
    all_bt_trajectories_x = [all_bt_trajectories_x; backtrack_trajectory(:,1)'];
    all_bt_trajectories_y = [all_bt_trajectories_y; backtrack_trajectory(:,2)'];
end

%%
mean_bt_traj_x = mean(all_bt_trajectories_x);
mean_bt_traj_y = mean(all_bt_trajectories_y);

mean_traj_y = mean_bt_traj_y(end:-1:1);
mean_traj_x = mean_bt_traj_x(end:-1:1);
mean_traj = [mean_traj_x', mean_traj_y'];
%%
 
 pos_data = retime(pos_data, shs.position.time, 'spline');
 
 x_error = pos_data.x - mean_traj_x';
 y_error = pos_data.y - mean_traj_y';
 
distance_error = sqrt(x_error.^2 + y_error.^2);

distance_rmse = rms(distance_error);

end