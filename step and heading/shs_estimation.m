function shs  = shs_estimation(sample_name)

%% % load Data %%%
[shs_sample, calib_samples, noise_sample, magnetic_north_sample] = ...
loadIMUData(sample_name);

shs.shs_sample = shs_sample;

%% Calibrating Sensor Data 
[calib_data, ~, variance] = ...
    calibrateSensors(shs_sample, calib_samples, ...
                     noise_sample, magnetic_north_sample);


%% Step Detection and Length Estimation
[shs.steps, ~, shs.sd_components] = stepDetection(calib_data.acc_data, 'data' , false);

% Step length estimation
shs.sl_components = timetable(shs.steps.data.Time);
shs.sl_components.period = [0; seconds(diff(shs.steps.data.Time))];
shs.sl_components.period(shs.sl_components.period == 0) = nan;
shs.sl_components.freq = 1./shs.sl_components.period;

const_time  = shs.sl_components(2,:).Time:seconds(0.1):shs.sl_components(end,:).Time;
step_time = shs.sl_components.Time;

all_times = unique(sort([const_time';step_time]));

shs.sl_components = retime(shs.sl_components,all_times,'linear');

shs.sl_components.standstill = shs.sl_components.period > 3;

male.k1 = 0.4;
male.k = 0.3116;

test_height = 1.78;

shs.steps.data.step_length = test_height.*male.k.*sqrt(shs.sl_components(shs.steps.data.Time,:).freq);
shs.steps.data.step_length(1) = test_height.*male.k1;
shs.est_distance = sum(shs.steps.data.step_length);

%% Extended Kalman Orientation Estimation
%  prior_est = [dev_comp_attitude{4,:}]';
prior_est = [1,0,0,0]';

ekf_estimate = ExtendedKalmanFilter_series(prior_est, calib_data, ...
    variance);

%% SHS Trajectory plotting with door handle flags

target = timetable(ekf_estimate.Time);
target.est = [ekf_estimate.est{:,:}]';
[shs.position, shs.step_and_orient] = plotTrajectory(target,shs,shs_sample.door_handle_use,false);

