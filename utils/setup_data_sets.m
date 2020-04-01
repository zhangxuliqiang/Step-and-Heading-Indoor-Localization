clear hyperIMU_inhand

file.directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/Datasets/';
file.name = 'HIMU-2020-03-24_18-58-04.csv';
hyperIMU_inhand.file = file;
hyperIMU_inhand.dataSetProp = DataSetProp("Accelerometer","timestamp",1/1000, ...
                          "K6DS3TR_Accelerometerx", ...
                          "K6DS3TR_Accelerometery", ...
                          "K6DS3TR_Accelerometerz");

%%

file.directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/Datasets/';
file.name = 'user1_armband_1506423438471.csv';
user1_armband.file = file;
user1_armband.dataSetProp = DataSetProp("Accelerometer","timestamp",(1E-9), ...
    "Accelerometerx","Accelerometery","Accelerometerz")