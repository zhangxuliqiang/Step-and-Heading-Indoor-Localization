close all
clear variables
clc

path = ['/home/' getenv('USER') ...
    '/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/Online Code/SLERepository-master/SLEDataset/'];
% find all files in directory and sub directories
sl_datasets = dir(strcat(path,'**/*.json'));

pocket_data_index = contains({sl_datasets.name},'hand-reading');

pocket_data = sl_datasets(pocket_data_index);

pocket_data_15m_index = contains({pocket_data.folder},'test_path1');

pocket_data_15m = pocket_data(pocket_data_15m_index);

sl_comparisons = [];

for sl_dataset = pocket_data_15m'
    
    file.directory = [sl_dataset.folder '/'] ;
    file.name = sl_dataset.name;
    
    disp(['step length estimation with :' file.name])
    
    target.file = file;
    target.dataSetProp = DataSetProp("Accelerometer","Time",(1E-2), ...
        ["X","Y","Z","or_X","or_Y","or_Z"]);
    
    [sl.properties, sl.raw_data] = JSONFile2Timetable(target);
    
    find_person_index = strfind(sl_dataset.folder,'person');
    str_split = strsplit(sl_dataset.folder(find_person_index:end),'/');
    sl.properties.user_id = convertCharsToStrings(str_split{1});
    
    disp(sl.properties.user_id)
    
    % vezocnik specific data massage since 
    sl.raw_data.acc0_magnitude = nan(height(sl.raw_data),1);
    sl.raw_data.acc0_magnitude = sqrt(sl.raw_data.X.^2 + (-sl.raw_data.Z-9.81).^2 + ...
                                      sl.raw_data.Y.^2);
    
    [sl.steps, ~, sl.sd_components] = ...
        stepDetection( sl.raw_data,'data', false);
    
    sl.mean_frequency = seconds(1)./mean(diff(sl.steps.data.Time));
        
    sl_comparisons = [sl_comparisons; sl];
    
end
spreadfigures
% 
% creating list of user_id
all_user_id = arrayfun(@(S) S.properties.user_id, sl_comparisons(:));
all_genders = arrayfun(@(S) convertCharsToStrings(S.properties.gender), ...
    sl_comparisons(:));
name_gender = [all_user_id,all_genders];
unique_user_ids_genders = unique(name_gender,'rows');

freq_comp = [];
counter = 0;
for unique_name_gender = unique_user_ids_genders'
    counter = counter + 1;
    
    freq_comp(counter).name = unique_name_gender(1); 
    freq_comp(counter).gender = unique_name_gender(2);
    ii = 0;
    for sl_comparison = sl_comparisons'
        if strcmp(sl_comparison.properties.user_id,unique_name_gender(1))
            ii = ii +1;
            
             freq_comp(counter).properties = sl_comparison.properties;
             freq_comp(counter).step_data(ii) = sl_comparison.steps;
            
            freq_comp(counter).proc_data(ii).height_mult_sqrt_freq = ...
                sl_comparison.properties.height*sqrt(sl_comparison.mean_frequency);
            
            freq_comp(counter).proc_data(ii).path_length_div_nr_steps = ...
                sl_comparison.properties.path_length/sl_comparison.steps.nr_steps;
            
            freq_comp(counter).proc_data(ii).speed  = sl_comparison.properties.walking_speed;
            freq_comp(counter).proc_data(ii).nr_steps  = sl_comparison.steps.nr_steps;
             freq_comp(counter).proc_data(ii).mean_frequency = sl_comparison.mean_frequency;
        end
    end
end
%% Determining Step Length Constant using Tian frequency based method

all_plotting_data = arrayfun(@(S) [[S.proc_data.height_mult_sqrt_freq]',...
    [S.proc_data.path_length_div_nr_steps]'], freq_comp(:), ...
    'UniformOutput', false);

all_plotting_data = cell2mat(all_plotting_data);
corrupt_data = find(isnan(all_plotting_data(:,1))');

% all_plotting_data([corrupt_data,30,42],:) = [];

all_plotting_data(corrupt_data,:) = []; %#ok<*FNDSB> warning supression

% linear least squares calculation
tian_constant = lsqr(all_plotting_data(:,1),all_plotting_data(:,2));
