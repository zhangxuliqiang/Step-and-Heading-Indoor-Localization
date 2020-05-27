clc
close all
% clear variables

file.directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/Online Code/benchmarks-attitude-smartphones-master/datasets/android/around_the_block_samsung/';
file.name = 'accelerometer.txt';
file.time_unit = 1;

target.file = file;
target.dataSetProp = DataSetProp("Accelerometer","Time",(file.time_unit), ...
    ["acc_X","acc_Y","acc_Z"]);

shs.name = file.name;

[shs.steps, shs.data, shs.sd_components] = stepDetection(target, false);

shs.sl_components.period = [0; seconds(diff(shs.steps.data.Time))];
shs.sl_components.period(shs.sl_components.period == 0) = nan;
shs.sl_components.freq = 1./shs.sl_components.period;
shs.mean_freq = mean(shs.sl_components.freq,'omitnan');

male.k1 = 0.415;
male.k = 0.3139;

test_height = 1.78;

shs.step_length = test_height.*male.k.*sqrt(shs.sl_components.freq);
shs.step_length(1) = test_height.*male.k1;
shs.distance_travelled = sum(shs.step_length);

%%
shs.orient_components = ExtendedKalmanFilter(shs.data, true);
%%
time_angles = timetable(shs.data.Time);
time_angles.euler_angles = bart_eulers;



positions = [];
prev_x = 0;
prev_y = 0;


step_orient = time_angles(shs.steps.data.Time, :).euler_angles;
% step_orient = cell2mat(step_orient');

for i = 1: length(step_orient)
   pos.x = prev_x + cos(step_orient(i,1)).* shs.step_length(i); 
   prev_x = pos.x;
   
   pos.y = prev_y + sin(step_orient(i,1)).* shs.step_length(i); 
   prev_y = pos.y;
   
   positions = [positions, pos];
end
%%
vertical_comp_step = cumsum(cos(shs.data(shs.steps.data.Time,:).samsung_orient_X).* shs.step_length);
horizontal_comp_step = cumsum(sin(shs.data(shs.steps.data.Time,:).samsung_orient_X).* shs.step_length);

%%
figure()
plot(horizontal_comp_step, vertical_comp_step)

%%

stackedplot(shs.data.Time, [shs.data.samsung_orient_X, shs.data.samsung_orient_Y, shs.data.samsung_orient_Z])

%%
figure()
plot([positions.x],[positions.y])
xlabel('X position from initial (meters)')
ylabel('Y position from initial (meters)')
title('step and heading system walking around the block')

