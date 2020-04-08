%% step detection and counting - windowed peak detection
%salvi et al approach
clc
close all
clear all

setup_data_sets

Acceleration = CSVFile2Timetable(user1_armband);

% Acceleration = JSONFile2Timetable(person1_test_path1);
disp('done importing')

%% plotting the raw data
% figure(1)
% s1 = stackedplot(Acceleration);
% title('3-axis raw accelerometer data for 8 steps')

%% create timetable for processed data
step_detection = timetable(Acceleration.timestamp);

%% optimal settings according to salvi et al

% filtering
filt_window_size = 13;
filt_type = 'gaussian';
filt_std = 0.35;
filt_cutoff_freq = 3; % Hz
filt_cutoff_gain = -60; %dB

% scoring
score_type = 'mean difference';
score_window_size = 35;

% detection
detect_threshold = 1.2;

% post-processing
pp_window_t = 0.200; %s

%% preprocessing: generate the norm of the acceleration signal
% TODO: interpolation to get constant sampling time
disp('calculating norm')
step_detection.acc0_magnitude = sqrt(Acceleration.X.^2 + Acceleration.Y.^2 + ...
    Acceleration.Z.^2);

%% Thresholding on standard deviation of 
disp('applying threshold on standard deviation')
step_detection.acc0_magnitude_thres = NaN(height(step_detection),1);
step_detection.acc0_magnitude_std = NaN(height(step_detection),1);

step_detection.acc0_magnitude_std = movstd(step_detection.acc0_magnitude,70);

threshold_row_index = step_detection.acc0_magnitude_std > 0.6;
threshold_rows = step_detection(threshold_row_index,:);

step_detection.acc0_magnitude_thres = step_detection.acc0_magnitude .* threshold_row_index;

%% Filtering Convolution process
disp('Applying gaussian filter')
gauss_window = gaussianWindow(filt_window_size, filt_std);
gauss_sum = sum(gauss_window);
kernel = transpose(gauss_window./gauss_sum);
conv_filt_tester = conv(step_detection.acc0_magnitude_thres, kernel, 'same');
step_detection.acc1_conv_gauss = conv_filt_tester;

%% Scoring convolution process
disp('Applying scoring')
score_middle_index = round(score_window_size/2);

kernel1 = ones(score_window_size,1).*(-1/(score_window_size - 1));
kernel1(score_middle_index,1) = 1;
conv_score_tester = conv(step_detection.acc1_conv_gauss, kernel1, 'same');

step_detection.acc2_conv_score = conv_score_tester;

%% Detection stage:
disp('Detecting peaks')
n = 0;
step_detection.acc3_detect = NaN(height(step_detection),1);
mean_detect = NaN(height(step_detection),1);
std_detect = NaN(height(step_detection),1);

for data_index = 1:1:height(step_detection)
%     disp(['detecting peaks: ' int2str(data_index)])
    
    dp_detect = step_detection(data_index,:);
    
    detect_score = dp_detect.acc2_conv_score;
    
    if  not(isnan(detect_score))
        
        n = n + 1;
        
        if n == 1
            detect_mean = detect_score;
            detect_std = 0;
            
        elseif n == 2
            det_o_mean = detect_mean;
            detect_mean = (detect_score + detect_mean)/2;
            detect_std = sqrt( ...
                ((detect_score - detect_mean)^2 + ...
                (det_o_mean - detect_mean)^2) / 2 ...
                );
        else
            det_o_mean = detect_mean;
            detect_mean = (detect_score + (n - 1)*detect_mean) / n;
            detect_std = sqrt(((n - 2) * detect_std^2 / (n - 1)) + ...
                (det_o_mean - detect_mean)^2 + ...
                (detect_score - detect_mean)^2 / n);
        end
        
        if n > 15
            if (detect_score - detect_mean) > detect_std * detect_threshold
                step_detection(dp_detect.Time,:).acc3_detect = ...
                    step_detection(dp_detect.Time,:).acc1_conv_gauss;
            end
        end
    end

end

%% finding local maxima through sliding window
disp('Finding local maxima')
step_detection.acc4_builtin_max = NaN(height(step_detection),1);

builtinmax = islocalmax(step_detection.acc3_detect,'MinSeparation',seconds(pp_window_t), ...
                        'SamplePoints',step_detection.Time);

threshold_row_index = step_detection.acc0_magnitude_std > 0.6;
local_max_rows = step_detection(builtinmax,:);
step_detection(local_max_rows.Time,:).acc4_builtin_max = local_max_rows.acc3_detect;
%%
figure(2)
hold on
plot(step_detection.Time, step_detection.acc1_conv_gauss)
scatter(step_detection.Time,step_detection.acc4_builtin_max)
