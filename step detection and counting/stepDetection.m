function [steps, Acceleration, sd_components] = stepDetection(target, debug)

% if not user specified all steps will de displayed in command terminal
if nargin < 2
    debug_flag = true;
end

% determine which import to use depending on file extension
[~, ~, fExt] = fileparts(target.file.name);
switch lower(fExt)
    case '.csv'
        Acceleration = CSVFile2Timetable(target);
    case '.json'
        Acceleration = JSONFile2Timetable(target);
    otherwise
        error('Unexpected file extension: %s', fExt);
end
disp(['step detection calculation using: ' target.file.name])
disp(['dataset size is:' int2str(height(Acceleration)) ' rows']);
debugDisp('     import data',debug)

%% create timetable for processed data
sd_components = timetable(Acceleration.Time);

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
detect_threshold = 1;

% post-processing
pp_window_t = 0.200; %s

%% preprocessing: generate the norm of the acceleration signal
% TODO: interpolation to get constant sampling time
debugDisp('     calculate norm',debug)
sd_components.acc0_magnitude = sqrt(Acceleration.X.^2 + Acceleration.Y.^2 + ...
    Acceleration.Z.^2);

%% Threshold on standard deviation of acceleration magnitude
debugDisp('     apply threshold on standard deviation',debug_flag)
sd_components.acc0_magnitude_thres = NaN(height(sd_components),1);
sd_components.acc0_magnitude_std = NaN(height(sd_components),1);

sd_components.acc0_magnitude_std = movstd(sd_components.acc0_magnitude,70);

threshold_row_index = sd_components.acc0_magnitude_std > 2;
threshold_rows = sd_components(threshold_row_index,:);

sd_components.acc0_magnitude_thres = sd_components.acc0_magnitude .* threshold_row_index;

%% Filter Convolution process
debugDisp('     Apply gaussian filter',debug_flag)

sd_components.acc1_conv_gauss = NaN(height(sd_components),1);

gauss_window = gaussianWindow(filt_window_size, filt_std);
gauss_sum = sum(gauss_window);
kernel = transpose(gauss_window./gauss_sum);
conv_gauss_filter = conv(sd_components.acc0_magnitude_thres, kernel, 'valid');

half_window = floor(filt_window_size/2);
sd_components(half_window+1:end-half_window,:).acc1_conv_gauss = conv_gauss_filter;

%% Score convolution process
debugDisp('     Apply scoring',debug_flag)
score_middle_index = round(score_window_size/2);

kernel1 = ones(score_window_size,1).*(-1/(score_window_size - 1));
kernel1(score_middle_index,1) = 1;
conv_score = conv(sd_components.acc1_conv_gauss, kernel1, 'same');

sd_components.acc2_conv_score = conv_score;

%% Detection stage:
debugDisp('     Detect peaks',debug_flag)
% Detecting outliers with builtin matlab methods
sd_components.acc3_quick_detect = NaN(height(sd_components),1);

cum_moving_mean = movmean(sd_components.acc2_conv_score, [length(sd_components.acc2_conv_score)-1 0], 'omitnan');
cum_moving_std = movstd(sd_components.acc2_conv_score, [length(sd_components.acc2_conv_score)-1 0], 'omitnan');

cum_detect_score_row_index = (sd_components.acc2_conv_score - cum_moving_mean) > ...
                    cum_moving_std .* detect_threshold;
                
threshold_rows = sd_components(cum_detect_score_row_index,:);
sd_components(threshold_rows.Time,:).acc3_quick_detect = threshold_rows.acc1_conv_gauss;

%% find local maxima through sliding window
debugDisp('     Find local maxima',debug_flag)
sd_components.acc4_builtin_max = NaN(height(sd_components),1);

builtinmax = islocalmax(sd_components.acc3_quick_detect,'MinSeparation',seconds(pp_window_t), ...
                        'SamplePoints',sd_components.Time);

local_max_rows = sd_components(builtinmax,:);
sd_components(local_max_rows.Time,:).acc4_builtin_max = local_max_rows.acc3_quick_detect;

%% Determine rows where a step was detected
sd_index = find(not(isnan(sd_components.acc4_builtin_max)));
steps.data = sd_components(sd_index,:);
[steps.nr_steps, ~] = size(sd_index);

