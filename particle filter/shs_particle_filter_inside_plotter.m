
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

for ii = 1 :3: height(specific_pf) 
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

%% shs plotting on occupancy grid

shs_output = [shs.position.x, shs.position.y];

theta = -pi/8;
rot_mat = [cos(theta),-sin(theta);
        sin(theta),cos(theta)];
    
shs_output_final = rot_mat*shs_output';

figure()
hold on
show(walls)
scatter(start_point(1),start_point(2),'bx')
plot(shs_output_final(1,:)+start_point(1),shs_output_final(2,:)+start_point(2))
hold off

%% video maker

 writerObj = VideoWriter('myVideo1.avi');
 writerObj.FrameRate = 10;

 % open the video writer
 open(writerObj);

for ii = 1 :1: height(specific_pf) 
    
    figure(1)
    show(walls)
    hold on
    if specific_pf(ii).door_detect == 1
    scatter([specific_pf(ii).particle_lists.x_pos]', [specific_pf(ii).particle_lists.y_pos]', '.r')
    else
        scatter([specific_pf(ii).particle_lists.x_pos]', [specific_pf(ii).particle_lists.y_pos]', '.b')
    end
    title(['itteration: ' num2str(ii)])
    hold off
    
    frame = getframe(figure(1));
    writeVideo(writerObj, frame);
end

 close the writer object
 close(writerObj);
 
 %%
 
