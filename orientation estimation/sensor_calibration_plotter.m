%% -------- PLOTTER: magnetometer calibration
% magnetometer calib data calibration
calib_mag_calib = mag_invD*(mag_raw'- mag_bias);

figure()
hold on
scatter3(calib_mag_calib(1,:),calib_mag_calib(2,:),calib_mag_calib(3,:),'r')
scatter3(calib_mag(1,:),calib_mag(2,:),calib_mag(3,:),'b')
scatter3(calib_mag_north(1,:),calib_mag_north(2,:),calib_mag_north(3,:),'g')
hold off
xlim([-1,1])
ylim([-1,1])
zlim([-1,1])
legend('calibrated calibration data','calibrated SHS data','calibrated magnetic north')
title('Calibrated Magnetometer Data')


%% -------- PLOTTER: accelerometer calibration

% magnetometer calib data calibration
calib_acc_calib = 9.81*acc_invD*(acc_raw'- acc_bias);

figure()
hold on
scatter3(calib_acc_calib(1,:),calib_acc_calib(2,:),calib_acc_calib(3,:),'r')
scatter3(calib_acc(1,:),calib_acc(2,:),calib_acc(3,:),'b')
hold off
% xlim([-1,1])
% ylim([-1,1])
% zlim([-1,1])
legend('calibrated calibration data','calibrated SHS data')
title('Calibrated Accelerometer Data')


%% -------- PLOTTER: noise realizations

if PLOTTER
    figure
    sgtitle('noise realization')
        subplot(3,1,1)    
        stackedplot(noise_acc_data)
        title('raw imu data - accelerometer')
        subplot(3,1,2)    
        stackedplot(noise_gyro_data)
        title('raw imu data - gyroscope')
        subplot(3,1,3)
        stackedplot(noise_mag_data)
        title('raw imu data - magnetometer')
    set(gcf,'position',[ 1434    138    548   489])
end