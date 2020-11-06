%% plot the different components of step detection
close all
figure
s = stackedplot(sl.raw_data	);
s.LineProperties(7).PlotType = 'scatter';
s.LineProperties(7).Marker = 'x';
s.DisplayLabels{1}= 'acc magnitude (m/s^2)';
s.DisplayLabels{2}= 'acc standard deviation (m/s^2)';
s.DisplayLabels{3}= 'acc over mag std threshold (m/s^2)';
s.DisplayLabels{4}= 'acc after gaussian filter (m/s^2)';
s.DisplayLabels{5}= 'sample score';
s.DisplayLabels{6}= 'outlier detection';
s.DisplayLabels{7}= 'step determination (m/s^2)';
title('All stages of step detection')

%%

close all 
figure
hold on
plot(sd.raw_data.Time, [0; diff(sd.raw_data.algo_step_detect)], 'xr')
plot(sd.raw_data.Time, [0; diff(sd.raw_data.truth_step_detect)], 'xb')

%%
figure()
scatter3(calib_mag(1,:),calib_mag(2,:),calib_mag(3,:))

%%

calib_mag_tester = invD*(raw(:,1:3)'- mag_bias);
% calib_mag_tester = calib_mag_tester';

figure()
scatter3(calib_mag_tester(1,:),calib_mag_tester(2,:),calib_mag_tester(3,:))
xlim([-1,1])
ylim([-1,1])
zlim([-1,1])

title('calibration magnetometer data set calibrated')

%%
figure()
scatter3(mag_raw(:,1),mag_raw(:,2),mag_raw(:,3))
% xlim([-1,1])
% ylim([-1,1])
% zlim([-1,1])
axis equal
xlabel(' magnetic x component (\muT)')
ylabel('magnetic y component (\muT)')
zlabel('magnetic z component (\muT)')
title('uncalibrated magnetometer data')
axesLabelsAlign3D
%%
figure()
scatter3(calib_mag_calib(1,:),calib_mag_calib(2,:),calib_mag_calib(3,:),'r')
xlim([-1,1])
ylim([-1,1])
zlim([-1,1])
axis equal
xlabel(' magnetic x component (\muT)')
ylabel('magnetic y component (\muT)')
zlabel('magnetic z component (\muT)')
title('Calibrated Magnetometer Data')
axesLabelsAlign3D
%%
own_mag_vector = mean(calib_mag_north{:,:});
own_mag_field =  transpose(own_mag_vector./norm(own_mag_vector));

mag_field = magnetic.vector/ norm(magnetic.vector);

starts = zeros(2,3);
ends = [mag_field';own_mag_field'];

figure()
hold on
scatter3(calib_mag_north{:,1},calib_mag_north{:,2},calib_mag_north{:,3})
quiver3(starts(1,1), starts(1,2), starts(1,3), ends(1,1), ends(1,2), ends(1,3), 'g')
quiver3(starts(2,1), starts(2,2), starts(2,3), ends(2,1), ends(2,2), ends(2,3), 'o')
hold off
xlim([-1,1])
ylim([-1,1])
zlim([-1,1])
grid on

title('magnetic north data set calibrated with normalized vector from magnetic world map')

%%

calib_acc_tester = acc_invD*(data(:,1:3)'- acc_bias);
% calib_mag_tester = calib_mag_tester';

figure()
scatter3(calib_acc_tester(1,:),calib_acc_tester(2,:),calib_acc_tester(3,:))
xlim([-1,1])
ylim([-1,1])
zlim([-1,1])

title('calibration magnetometer data set calibrated')

%%

target5 = [magnetic{:,:}];

figure()
scatter3(target5(1,:),target5(2,:),target5(3,:))
% xlim([-1,1])
% ylim([-1,1])
% zlim([-1,1])

title('trajectory accelerometer data set calibrated times gravity')

%%

figure()
estimator = [estimate1.error{:,:}]';
stackedplot(estimator)
title('residuals')


%%
close all
figure()
stackedplot(estimate1.Time, [estimate1.est{:,:}]')
title('my estimator')

figure()
stackedplot(dev_comp_attitude)
title('device estimate')

%%
clear target
target = dev_comp_attitude;
euler_angles = timetable(target.Time);
[euler_angles.yaw, euler_angles.pitch, euler_angles.roll] =  ...
    quat2angle([target{:,:}]);

figure()
stackedplot(euler_angles)
title('euler angles of device attitude')

target1 = estimate1;
euler_angles1 = timetable(target1.Time);
[euler_angles1.yaw, euler_angles1.pitch, euler_angles1.roll] =  ...
    quat2angle([target1.est{:,:}]');

figure()
stackedplot(euler_angles1)
title('euler angles of my attitude estimation')
%%
data = euler_angles1(euler_angles.Time,:);

[target2,ia,ic] = unique(data.Time,'first');


target3 = data(ia,:);

figure()
plot(rad2deg(euler_angles.yaw - target3.yaw))
title("difference between device yaw and my yaw")

%%
figure()
plot(shs.steps.data.step_length)

%%
figure()
subplot(3,1,1)
histogram(diff(mag.Time))
title('mag')
subplot(3,1,2)
histogram(diff(gyr.Time))
title('gyr')
subplot(3,1,3)
histogram(diff(acc.Time))
title('acc')
sgtitle('time difference for sensorlog app')

%%

figure()
plot(diff(combined_raw.Time))
title('time difference between samples')
%%

figure()
histogram(diff(combined_raw.Time))
title('histogram of time difference between samples')
%%
Types = categorical({'GYR','ACC', 'MAG'});
gyr_row_index = ekf_estimate.type == Types(1);
gyr_rows = ekf_estimate(gyr_row_index,:);

acc_row_index = ekf_estimate.type == Types(2);
acc_rows = ekf_estimate(acc_row_index,:);

mag_row_index = ekf_estimate.type == Types(3);
mag_rows = ekf_estimate(mag_row_index,:);



%%
close all
figure()
stackedplot(seconds(diff(mag_rows.Time)))
title('magnetometer residuals')
%%
figure()
stackedplot(step_orient)

%%
close all
figure()
stackedplot(shs_sample.raw_imu.gyroscope)
%%
figure()
stackedplot(dev_com_positions)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure()
hold on
show(map)
plot(gps_data.x_pos,gps_data.y_pos)
hold off 

%%
% close all
trajectory = og_positions;

figure()
hold on
show(map)
x = [trajectory.x] + + start_point_meter(2);
y = [trajectory.y] + start_point_meter(1);
plot(x,y, 'y')
hold off


figure()
stackedplot([streamjWatch200729152346.motionUserAccelerationXG, ...
             streamjWatch200729152346.motionUserAccelerationYG, ...
             streamjWatch200729152346.motionUserAccelerationZG] )
%%
figure()
stackedplot([streamjWatch200729152346.accelerometerAccelerationXG, ...
            streamjWatch200729152346.accelerometerAccelerationYG, ...
            streamjWatch200729152346.accelerometerAccelerationZG])
        
%%

pd2 = makedist('HalfNormal','mu',0,'sigma',2);
x = 0:0.1:10;
pdf1 = pdf(pd2,x);
figure()
plot(x,pdf1)
title('Half normal distribution for GPS comparison')
ylabel('weighting')
xlabel('distance to gps point')

%%

figure()
hold on
show(map)
scatter(midpoints(2),midpoints(1),'bx')

%%
figure()
norm_mag = vecnorm(magSampled{:,:}',1);
plot(norm_mag)
title('calibrated magnetometer reading with outdoor calibration' )

%%
figure
imshow(red_regions_image)

%%
door_pixel_location = bwboundaries(red_regions_image);

doors = grid2local(map, cell2mat(door_pixel_location));

figure
hold on
show(walls)
scatter(particle_list.prev_x_pos,particle_list.prev_y_pos,'x')

%%
for i=1:length(dead_particles)
  isequal =(dead_particles(i,1)== resample_index);
  dead_particles(i,2) = sum(isequal);
end

