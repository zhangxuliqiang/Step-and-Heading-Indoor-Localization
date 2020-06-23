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
scatter3(calib_mag(1,:),calib_mag(2,:),calib_mag(3,:))
% xlim([-1,1])
% ylim([-1,1])
% zlim([-1,1])

title('trajectory magnetometer data set calibrated')

%%

dip_angle = 67.095;
% mag_field = [sind(dip_angle);cosd(dip_angle); 0 ];
mag_field = magnetic.vector/ norm(magnetic.vector);

starts = zeros(1,3);
ends = mag_field';

figure()
hold on
scatter3(calib_mag_north(1,:),calib_mag_north(2,:),calib_mag_north(3,:))
quiver3(starts(:,1), starts(:,2), starts(:,3), ends(:,1), ends(:,2), ends(:,3))
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

figure()
scatter3(calib_acc(1,:),calib_acc(2,:),calib_acc(3,:))
% xlim([-1,1])
% ylim([-1,1])
% zlim([-1,1])

title('trajectory accelerometer data set calibrated times gravity')

%%
% figure()
% plot([estimate.mag_error{:,:}]')

figure()
estimator = [estimate1.error{:,:}]';
stackedplot(estimator)
title('residuals')

%%

figure()
stackedplot(dev_comp_attitude)
%%
figure()
stackedplot(estimate1.Time, [estimate1.final_q{:,:}]')

%%
figure()
subplot(3,1,1)
histogram(diff(mag_rows.Time))
title('mag')
subplot(3,1,2)
histogram(diff(gyr_rows.Time))
title('gyr')
subplot(3,1,3)
histogram(diff(acc_rows.Time))
title('acc')
sgtitle('time difference for linkoping sensor app')
%%
Types = categorical({'GYR','ACC', 'MAG'});
gyr_row_index = raw_imu.combined.Type == Types(1);
gyr_rows = raw_imu.combined(gyr_row_index,:);

acc_row_index = raw_imu.combined.Type == Types(2);
acc_rows = raw_imu.combined(acc_row_index,:);

mag_row_index = raw_imu.combined.Type == Types(3);
mag_rows = raw_imu.combined(mag_row_index,:);

%%
strcmp(raw_imu.combined.Type(1), "ACC" )
