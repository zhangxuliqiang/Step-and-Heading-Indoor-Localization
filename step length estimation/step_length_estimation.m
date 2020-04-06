%% step length estimation

dir_path = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/step length/person01';

target = fileread([dir_path '/test_path1/pocket_position_normal_walking_speed.json']);

step_length_data = jsondecode(target);

magnitude = sqrt(step_length_data.linear_acceleration.x.^2 + ...
            step_length_data.linear_acceleration.y.^2 + ...
            step_length_data.linear_acceleration.z.^2);

plot(magnitude)
%%
plot(step_length_data.linear_acceleration.x)