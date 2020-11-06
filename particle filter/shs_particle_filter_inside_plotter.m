
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

%%
plotter_pf = pf.specific_pf;
ground_truth =  pf.gt;

shs_output = [pf.shs.position.x, pf.shs.position.y];

theta = -pi/8;
rot_mat = [cos(theta),-sin(theta);
        sin(theta),cos(theta)];
    
shs_output_final = rot_mat*shs_output';

figure
show(walls)
hold on
plot(shs_output_final(1,:)+start_point(1),shs_output_final(2,:)+start_point(2))
plot(ground_truth.x,ground_truth.y)
hold off
%% specific particle filter replay
close all 
plotter_pf = results(37).specific_pf;
ground_truth =  results(37).gt;

plotter_pf = pf.specific_pf;
ground_truth =  pf.gt;

shs_output = [pf.shs.position.x, pf.shs.position.y];

theta = -pi/8;
rot_mat = [cos(theta),-sin(theta);
        sin(theta),cos(theta)];
    
shs_output_final = rot_mat*shs_output';

for ii = 1 :2: height(plotter_pf) 
    ii
    particle_list = plotter_pf(ii,:).particle_lists{1,1};
    figure(1)
    show(walls)
    hold on
    if plotter_pf(ii,:).door_detect == 1
    scatter(particle_list(:,utils.index.x_pos), particle_list(:,utils.index.y_pos), '.r')
    else
        scatter(particle_list(:,utils.index.x_pos), particle_list(:,utils.index.y_pos), '.b')
    end
    plot(shs_output_final(1,:)+start_point(1),shs_output_final(2,:)+start_point(2))
    plot(plotter_pf(ii,:).estimate(1),plotter_pf(ii,:).estimate(2),'og')
    plot(ground_truth.x,ground_truth.y)
    title(['timestep: ' num2str(ii)])
    hold off
end

%%
clc
close all
% pos_data = struct2table(position_data);
%  pos_data.time = seconds(pos_data.time);
%  pos_data = table2timetable(pos_data);
%  
title_string = 'Trial 3 SHS-PF comparison to ground truth'; 
system_label = 'shs-pf';

pos_data = retime(pf.gt, pf.shs.position.time, 'pchip');
 
 absolute_shs = [shs_output_final(1,:)'+start_point(1),shs_output_final(2,:)'+start_point(2)];
 
 target = absolute_shs;
 target = pf.mean_traj;
 
 x_error = pos_data.x - target(:,1);
 y_error = pos_data.y - target(:,2);
 
distance_error = sqrt(x_error.^2 + y_error.^2);

rmse_distance = rms(distance_error);

 figure
 subplot(4,1,1)
 hold on 
 plot(pos_data.time, pos_data.x)
 plot(pf.shs.position.time,  target(:,1))
 ylim([0,30])
 hold off
 xlabel('Time')
 ylabel('X Position (m)')
 title('X Position Comparison')
 legend(system_label,'ground truth','orientation','horizontal')
 subplot(4,1,2)
 hold on 
 plot(pos_data.time, pos_data.y)
 plot(pf.shs.position.time,  target(:,2))
 ylim([10,40])
 xlabel('Time')
 ylabel('Y Position (m)')
 title('Y Position Comparison')
 legend(system_label,'ground truth','orientation','horizontal')
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
sgtitle(title_string)
 
 set(gcf,'position',[1926         625         622         734])

 % shs plotting on occupancy grid

figure()
hold on
show(walls)
% scatter(start_point(1),start_point(2),'gx')
plot(target(:,1),target(:,2))
plot([pos_data.x],[pos_data.y])
legend([system_label ' output'],'ground truth','Location','SouthEast')
xlabel('X position (m)')
ylabel('Y position (m)')
% plot(pf.mean_traj(:,1),pf.mean_traj(:,2),'g')
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
%     figure
    unique_names = unique({results.sample_name});
    pf_performance = [];
    for i = 1:length(unique_names)
        values = [];
        
        relevant_values = strcmp({results.sample_name}, unique_names{i});
        
        values = [results.distance_rmse].*relevant_values;
        [~,~,relval] = find(values);
        
        performance.name = unique_names{i};
        performance.mean  = mean(relval,'omitnan');
        performance.std = std(relval,'omitnan');
        performance.completed = sum(~isnan(relval));
        pf_performance = [ pf_performance; performance];
    end

    
    figure
    box = boxplot([results.distance_rmse],{results.sample_name});
    title('RMSE per trial')
    ylabel('RMSE (m)')
    xlabel('Trial Name')

for i = 1:length(unique_names)   
    
txt = ['completed: \newline' num2str(pf_performance(i).completed)  ];
text(i,2,txt,'HorizontalAlignment','center')

end
%% video maker

 writerObj = VideoWriter('lopen1.3.avi');
 writerObj.FrameRate = 10;

 % open the video writer
 open(writerObj);

for ii = 1 :1: height(plotter_pf) 
    
    figure(1)
    show(walls)
    hold on
    
    particle_list = plotter_pf(ii,:).particle_lists{1,1};
    x_pos = particle_list(:,utils.index.x_pos);
    y_pos = particle_list(:,utils.index.y_pos);
    plot(pf.gt.x,pf.gt.y,'g')
    if plotter_pf(ii,:).door_detect == 1
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
 
