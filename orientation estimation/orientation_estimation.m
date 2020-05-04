clc
close all
clear variables

debug = false;

file.directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/orientation estimation/';
file.name = 'HIMU-2020-04-28_14-31-45';
file.time_unit = 1E-3;

target.file = file;
target.dataSetProp = DataSetProp("Accelerometer","Time",(file.time_unit), ...
    ["acc_X","acc_Y","acc_Z", ...
    "mag_X","mag_Y","mag_Z", ...
    "gyr_X","gyr_Y","gyr_Z", ...
    "screen_orient_X","screen_orient_Y","screen_orient_Z",...
    "samsung_orient_X","samsung_orient_Y","samsung_orient_Z"]);
orient.name = file.name;
orient.data = CSVFile2Timetable(target);

%% quaternion and convariance matrix

prior_est = [1;0;0;0];
prior_P = eye(4, 4);

calAcc.m = zeros(3, 1);
calAcc.R = 1e-2 * eye(3);

calGyr.m = zeros(3, 1);
calGyr.R = 1e-5 * eye(3);

calMag.m = zeros(3, 1);
calMag.R = 0.5 * eye(3);



% gyroscope time update

dT = [0; seconds(diff(orient.data.Time))];

% transpose measurements to get column vectors for matrix operations
gyro = [orient.data.gyr_X, orient.data.gyr_Y, orient.data.gyr_Z]';
acc = [orient.data.acc_X, orient.data.acc_Y, orient.data.acc_Z]';
mag = [orient.data.mag_X, orient.data.mag_Y, orient.data.mag_Z]';
Time = [seconds(orient.data.Time)];

estimate = repmat(struct('Time', nan, ... 
              'euler_prior_est',nan, ...
              'euler_post_acc_est', nan, ...
              'euler_post_mag_est', nan ), height(orient.data), 1 );

g = [0; 0; 9.82];
dip_angle = 67.095;
mag_field = [cosd(dip_angle); 0 ; sind(dip_angle)];

unit_mag = mag/norm(mag);

for index = 1:1:height(orient.data)
    % -------------  MOTION UPDATE -----------------------
    F = eye(4) + dT(index)/ 2.* Somega(gyro(:,index));
    prior_est = F* prior_est;
    Gu= dT(index)./ 2 *Sq(prior_est);
    prior_est = prior_est / norm(prior_est);
    prior_P = F*prior_P*F' + Gu*calGyr.R*Gu';
    
    % -------------- MEASUREMENT UPDATES ------------------
    
    % Accelerometer measurement update
    dRdq_acc = dRqdq(prior_est);
    H_acc = [dRdq_acc(:,:,1)'*g, dRdq_acc(:,:,2)'*g,...
        dRdq_acc(:,:,3)'*g, dRdq_acc(:,:,4)'*g];
    
    % transpose rotation matrix of body frame to navigation frame to get from
    % navigation to body frame
    
    acc_error = acc(:,index) - quat2rotmat(prior_est)' * g;
    
    [post_acc_P, post_acc_est] = ...
        MeasurementUpdate(acc_error, prior_est, prior_P, calAcc.R, H_acc);
    
    % magnetometer measurement update
    dRdq_mag = dRqdq(post_acc_est);
    H_mag = [dRdq_mag(:,:,1)'*mag_field, ...
             dRdq_mag(:,:,2)'*mag_field, ...
             dRdq_mag(:,:,3)'*mag_field, ...
             dRdq_mag(:,:,4)'*mag_field];
    
    mag_error = unit_mag(:,index) - quat2rotmat(post_acc_est)' * mag_field;
    
    [post_mag_P, post_mag_est] = ...
        MeasurementUpdate(mag_error, post_acc_est, post_acc_P, calMag.R , H_mag);
    
    prior_est = post_mag_est;
    prior_P = post_mag_P; 
    %     --------------- SAVING ESTIMATE COMPONENTS ---------
    x.Time =Time(index);

    x.euler_prior_est = q2euler(prior_est);

    x.euler_post_acc_est = q2euler(post_acc_est);
   
    x.euler_post_mag_est = q2euler(post_mag_est);
    
    if debug
        x.prior_est = prior_est;
        x.prior_P = prior_P;
        
        x.post_acc_est = post_acc_est;
        x.post_acc_P = post_acc_P;
        
        x.post_mag_est = post_mag_est;
        x.post_mag_P = post_mag_P;
    end
    
    estimate(index) = x;
    
end


%% plotting estimate of EKF vs samsung estimation

Data = struct2table(estimate);
Data.Time = seconds(Data.Time);
Data = table2timetable(Data);

prior_euler_angles = cell2mat(Data.euler_prior_est');
post_acc_euler_angles = cell2mat(Data.euler_post_acc_est');
post_mag_euler_angles = cell2mat(Data.euler_post_mag_est');

newYlabels = {'yaw (rad)', 'pitch (rad)' ,'roll (rad)'};

pseudo_gt_orient = [orient.data.samsung_orient_X, orient.data.samsung_orient_Y, orient.data.samsung_orient_Z];

figure()

for i = 1:3
    subplot(4,1,i)
    hold on
    prior_ekf_line(i) = plot(Data.Time, prior_euler_angles(i,:)');
    post_acc_ekf_line(i) = plot(Data.Time, post_acc_euler_angles(i,:)');
    post_mag_ekf_line(i) = plot(Data.Time, post_mag_euler_angles(i,:)');
    samsung_line(i) = plot(Data.Time, deg2rad(pseudo_gt_orient(:,i)));
    hold off
    ylabel(newYlabels(i))
end
xlabel('Time (seconds)')
sgtitle('ekf motion model vs samsung orientation estimation using gyro and accelerometer')
% Create dummy subplot for legend
hLegend = subplot(4,1,4);
posLegend = get(hLegend,'Position');
% Creating the legend
leg = legend(hLegend,[prior_ekf_line(1),post_acc_ekf_line(1),post_mag_ekf_line(1), samsung_line(1)],'ekf prior estimate', 'ekf post acc estimate', 'ekf post mag estimate', 'samsung estimate');
axis(hLegend,'off');
set(leg,'Position',posLegend);

%%

figure()
hold on
plot(orient.data.gyr_X)
plot(orient.data.gyr_Y)
plot(orient.data.gyr_Z)
hold off
