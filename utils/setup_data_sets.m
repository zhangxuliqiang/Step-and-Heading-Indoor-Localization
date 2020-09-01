clear hyperIMU_inhand

file.directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/step counting/original data';
file.name = 'HIMU-2020-03-24_18-58-04.csv';
hyperIMU_inhand.file = file;
hyperIMU_inhand.dataSetProp = DataSetProp("Accelerometer","Time",1/1000, ...
                          ["X","Y","Z"]);

%%
clear user1_armband

file.directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/Online Code/oxford step counter/validation/';
file.name = 'user1_armband_1506423438471.csv';
user1_armband.file = file;
user1_armband.dataSetProp = DataSetProp("Accelerometer","Time",(1E-9), ...
    ["X","Y","Z","algo_step_detect","truth_step_detect"]);

%%
clear person1_test_path1

file.directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/step length/person01';
file.name = '/test_path1/pocket_position_normal_walking_speed.json';
person1_test_path1.file = file;
person1_test_path1.dataSetProp = DataSetProp("Accelerometer","Time",(1E-2), ...
    ["X","Y","Z"]);






