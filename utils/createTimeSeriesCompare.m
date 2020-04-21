function result = createTimeSeriesCompare(Acceleration, matlab_algo_steps) 

sd_comparison = timetable(Acceleration.Time);

% set steps detection points found by ground truth and both alogithms
sd_comparison.sd_matlab_algo_points  = zeros(height(Acceleration),1);
sd_comparison(matlab_algo_steps.Time,:).sd_matlab_algo_points = ones(height(matlab_algo_steps),1);
sd_comparison.sd_ground_truth_points = [0; diff(Acceleration.truth_step_detect)];
sd_comparison.sd_android_algo_points = [0; diff(Acceleration.algo_step_detect)];

% populate cumulative sum for all three techniques
sd_comparison.cumsum_ground_truth = Acceleration.truth_step_detect;
sd_comparison.cumsum_android_algo = Acceleration.algo_step_detect;
sd_comparison.cumsum_matlab_algo = cumsum(sd_comparison.sd_matlab_algo_points);

result = sd_comparison;
end