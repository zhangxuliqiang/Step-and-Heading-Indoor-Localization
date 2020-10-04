function [steps, data, sd_components] = stepDetection(target, data_type, debug_flag)
% STEPDETECTION Calculates steps from three axis accelerometer data
%   INPUT: Either file path (data_type = 'file' ) or raw data timetable (data_type = 'data')
%   OUTPUT: steps - contains number of steps and the raw data associated
%           with it.
%           data - returns raw data in timetable form from import, only
%           really useful when data_type = 'file'.
%           sd_components - returns a timetable with every stage of step
%           detection


% if not user specified all steps will de displayed in command terminal
if nargin < 2
    debug_flag = true;
end

if strcmp(data_type, 'file')
    % determine which import to use depending on file extension
    [~, ~, fExt] = fileparts(target.file.name);
    switch lower(fExt)
        case '.csv'
            data = CSVFile2Timetable(target);
        case '.json'
            data = JSONFile2Timetable(target);
        case '.txt'
            data = SSVFile2Timetable(target);
            timestamp = seconds(0.0810:1/100:291.3200);
            data = retime(data,timestamp,'linear');
        otherwise
            error('Unexpected file extension: %s', fExt);
    end
    disp(['step detection calculation using: ' target.file.name])
    
elseif strcmp(data_type, 'data')
    data = target;
end
debugDisp(['dataset size is:' int2str(height(data)) ' rows'], debug_flag);
debugDisp('     import data',debug_flag)

%% create timetable for processed data
sd_components = timetable(data.Time);

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
detect_threshold = 1.1;

% post-processing
pp_window_t = 0.200; %s

%% preprocessing: generate the norm of the acceleration signal
% TODO: interpolation to get constant sampling time
debugDisp('     calculate norm',debug_flag)
if sum(contains(data.Properties.VariableNames,'acc0_magnitude'))<1
    sd_components.acc0_magnitude = sqrt(data.X.^2 + data.Y.^2 + ...
        data.Z.^2);
else
    sd_components.acc0_magnitude = data.acc0_magnitude;
end

%% Threshold on standard deviation of acceleration magnitude
debugDisp('     apply threshold on standard deviation',debug_flag)
sd_components.acc0_magnitude_std = NaN(height(sd_components),1);
sd_components.acc0_magnitude_thres = NaN(height(sd_components),1);

sd_components.acc0_magnitude_std = ...
    movstd(sd_components.acc0_magnitude,[seconds(0.8),seconds(0)], ...
    'SamplePoints',sd_components.Time);

threshold_row_index = sd_components.acc0_magnitude_std > 0.6;
threshold_rows = sd_components(threshold_row_index,:);

sd_components(threshold_rows.Time,:).acc0_magnitude_thres = ...
    sd_components(threshold_rows.Time,:).acc0_magnitude;

%% Filter Convolution process
debugDisp('     Apply gaussian filter',debug_flag)

sd_components.acc1_conv_gauss = NaN(height(sd_components),1);

gauss_window = gaussianWindow(filt_window_size, filt_std);
gauss_sum = sum(gauss_window);
kernel = transpose(gauss_window./gauss_sum);
conv_gauss_filter = conv(sd_components.acc0_magnitude_thres, kernel, ...
    'valid');

half_window = floor(filt_window_size/2);

sd_components(half_window+1:end-half_window,:).acc1_conv_gauss = ...
    conv_gauss_filter;

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

cum_moving_mean = movmean(sd_components.acc2_conv_score, ...
    [length(sd_components.acc2_conv_score)-1 0], 'omitnan');

cum_moving_std = movstd(sd_components.acc2_conv_score, ...
    [length(sd_components.acc2_conv_score)-1 0], 'omitnan');

cum_detect_score_row_index = ...
    (sd_components.acc2_conv_score - cum_moving_mean) > ...
    cum_moving_std .* detect_threshold;

threshold_rows = sd_components(cum_detect_score_row_index,:);

sd_components(threshold_rows.Time,:).acc3_quick_detect = ...
    threshold_rows.acc1_conv_gauss;

%% find local maxima through sliding window
debugDisp('     Find local maxima',debug_flag)
sd_components.acc4_builtin_max = NaN(height(sd_components),1);

builtinmax = islocalmax(sd_components.acc3_quick_detect, ...
    'MinSeparation',seconds(pp_window_t),'SamplePoints',sd_components.Time);

local_max_rows = sd_components(builtinmax,:);
sd_components(local_max_rows.Time,:).acc4_builtin_max = ...
    local_max_rows.acc3_quick_detect;

%% Determine rows where a step was detected
sd_index = find(not(isnan(sd_components.acc4_builtin_max)));
steps.data = sd_components(sd_index,:);
[steps.nr_steps, ~] = size(sd_index);

