
% Occupancy Grid
figure()
hold on
show(walls)
title( '2D probability density function from map information')
grid(gca,'on')
%% Pixel to meter conversion
fig = figure(); 
imshow(image)
title('choose the points needed for meter to pixel conversion')
hold on
scale_points = ginput(2);
hold off
close(fig)

real_world_length = 14.00; %<-- this value is generated through google maps
points_per_meter = norm(scale_points(1,:)'-scale_points(2,:)')/real_world_length;

%% measuring tool for in the occupancy grid

fig = figure()
hold on
show(walls)
plot(doors(:,1),doors(:,2),'x')
measure_points = ginput(2);
hold on 
plot(measure_points(1,1),measure_points(1,2),'x')
plot(measure_points(2,1),measure_points(2,2),'x')


norm(measure_points(1,:)'-measure_points(2,:)')


%% specific particle filter replay

shs_output = [shs.position.x, shs.position.y];

theta = -pi/8;
rot_mat = [cos(theta),-sin(theta);
        sin(theta),cos(theta)];
    
shs_output_final = rot_mat*shs_output';

for ii = 1 :2: height(specific_pf) 
    particle_list = specific_pf(ii,:).particle_lists{1,1};
    figure(1)
    show(walls)
    hold on
    if specific_pf(ii,:).door_detect == 1
    scatter(particle_list(:,utils.index.x_pos), particle_list(:,utils.index.y_pos), '.r')
    else
        scatter(particle_list(:,utils.index.x_pos), particle_list(:,utils.index.y_pos), '.b')
    end
    plot(shs_output_final(1,:)+start_point(1),shs_output_final(2,:)+start_point(2))
    plot(specific_pf(ii,:).estimate(1),specific_pf(ii,:).estimate(2),'og')
    plot([position_data.x],[position_data.y])
    title(['timestep: ' num2str(ii)])
    hold off
end

%%

figure
plot(specific_pf.effective_nr_sample)

%%

all_bt_trajectories_x = [];
all_bt_trajectories_y = [];
for ancestor = 1:length(specific_pf.particle_lists{1,1})
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

all_bt_trajectories_x = NaN(length(specific_pf.particle_lists{1,1}),height(specific_pf));
all_bt_trajectories_y = NaN(length(specific_pf.particle_lists{1,1}),height(specific_pf));

for last_particle = 1:1:length(specific_pf.particle_lists{1,1})
    ancestor = last_particle;
    backtrack_trajectory = NaN(height(specific_pf),2);
    for ii = height(specific_pf):-1:1
        particle_list = specific_pf(ii,:).particle_lists{1,1};
        backtrack_trajectory(ii,:) = particle_list(ancestor,utils.index.x_pos:utils.index.y_pos);
        % finding the next ancestor
        ancestor  = particle_list(ancestor,utils.index.particle_history);
    end
    all_bt_trajectories_x(last_particle,:) =  backtrack_trajectory(:,1)';
    all_bt_trajectories_y(last_particle,:) =  backtrack_trajectory(:,2)';
end

%%
figure
for i = 1:length(all_bt_trajectories_x)
    hold on
    plot(all_bt_trajectories_x(i,:),all_bt_trajectories_y(i,:))
    hold off
end

%%
mean_bt_traj_x = mean(all_bt_trajectories_x);
mean_bt_traj_y = mean(all_bt_trajectories_y);

mean_traj_y = mean_bt_traj_y(end:-1:1);
mean_traj_x = mean_bt_traj_x(end:-1:1);

mean_traj = []

%%
close all
pos_data = struct2table(position_data);
 pos_data.time = seconds(pos_data.time);
 pos_data = table2timetable(pos_data);
 
 pos_data = retime(pos_data, shs.position.time, 'pchip');
 
 absolute_shs = [shs_output_final(1,:)'+start_point(1),shs_output_final(2,:)'+start_point(2)];
 
 target = absolute_shs;
 target = mean_traj;
 
 x_error = pos_data.x - target(:,1);
 y_error = pos_data.y - target(:,2);
 
distance_error = sqrt(x_error.^2 + y_error.^2);

rmse_distance = rms(distance_error);

 figure
 subplot(4,1,1)
 hold on 
 plot(pos_data.time, pos_data.x)
 plot(shs.position.time,  target(:,1))
 ylim([0,30])
 hold off
 xlabel('Time')
 ylabel('X Position (m)')
 title('X Position Comparison')
 legend('shs','ground truth','orientation','horizontal')
 subplot(4,1,2)
 hold on 
 plot(pos_data.time, pos_data.y)
 plot(shs.position.time,  target(:,2))
 ylim([10,40])
 xlabel('Time')
 ylabel('Y Position (m)')
 title('Y Position Comparison')
 legend('shs','ground truth','orientation','horizontal')
 hold off
 subplot(4,1,3)
 hold on 
 plot(pos_data.time, x_error, 'color',[0.9290, 0.6940, 0.1250])
 plot(pos_data.time, y_error, 'color', [0.4940, 0.1840, 0.5560])
 legend('x error','y error','orientation','horizontal')
 xlabel('Time')
 ylabel('Error (m)')
 title('Error in Both Axis')
 hold off
 subplot(4,1,4)
 plot(pos_data.time, distance_error,'black')
 xlabel('Time')
 ylabel('Error (m)')
 title('Total Distance Error')
sgtitle('Trial 2 SHS comparison to ground truth')
 
 set(gcf,'position',[1926         625         622         734])

 % shs plotting on occupancy grid

shs_output = [shs.position.x, shs.position.y];

theta = -pi/8;
rot_mat = [cos(theta),-sin(theta);
        sin(theta),cos(theta)];
    
shs_output_final = rot_mat*shs_output';

figure()
hold on
show(walls)
% scatter(start_point(1),start_point(2),'gx')
plot(target(:,1),target(:,2))
plot([position_data.x],[position_data.y])
legend('shs output','ground truth','Location','SouthEast')
xlabel('X position (m)')
ylabel('Y position (m)')
% plot(backtrack_trajectory(:,1),backtrack_trajectory(:,2),'g')
hold off
set(gcf,'position',[ 1921 632 524 722])

%%
figure
hold on
plot(pos_data.x,pos_data.y)
plot([position_data.x],[position_data.y],'r')
hold off

%%
figure
plot(diff(pos_data.time))
%%
figure
hold on
plot(mean_bt_traj_x,mean_bt_traj_y, 'r')
plot([position_data.x],[position_data.y],'b')
hold off
    
    %%
    figure
    plot(hist_pos(:,1),hist_pos(:,2))

%% video maker

 writerObj = VideoWriter('lopen1.1.avi');
 writerObj.FrameRate = 10;

 % open the video writer
 open(writerObj);

for ii = 1 :1: height(specific_pf) 
    
    figure(1)
    show(walls)
    hold on
    
    particle_list = specific_pf(ii,:).particle_lists{1,1};
    x_pos = particle_list(:,utils.index.x_pos);
    y_pos = particle_list(:,utils.index.y_pos);
    plot([position_data.x],[position_data.y],'g')
    if specific_pf(ii,:).door_detect == 1
        scatter(x_pos, y_pos, '.r')
    else
        scatter(x_pos, y_pos, '.b')
    end
    title(['itteration: ' num2str(ii)])
    hold off
    
    frame = getframe(figure(1));
    writeVideo(writerObj, frame);
end

%  close the writer object
 close(writerObj);
 
 %%
 
