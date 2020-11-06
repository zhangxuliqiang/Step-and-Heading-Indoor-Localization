function [shs_sample, calib_samples, noise_sample, magnetic_north_sample] = ...
loadIMUData(data_set_name)

switch data_set_name
    
    %% ================== load Data ======================
    case ( 'lopen1.5' )
        folder_name = 'datasets/marie testing/lopen1.5/';
        
        shs_sample = loadAndroidDataset([folder_name 'lopen1_5/']);
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset([folder_name 'calib_mag/']);
        calib_samples.gyr_calib_sample = loadAndroidDataset([folder_name 'noise1_4/']);
        calib_samples.acc_calib_sample = loadAndroidDataset([folder_name 'calib_acc/']);
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset([folder_name 'noise1_4/']);
        
        % load noise sample
        noise_sample = loadAndroidDataset([folder_name 'noise1_4/']); 
    
    case ( 'lopen1.4' )
        folder_name = 'datasets/marie testing/lopen1.4/';
        
        shs_sample = loadAndroidDataset([folder_name 'lopen1_4/']);
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset([folder_name 'calib_mag/']);
        calib_samples.gyr_calib_sample = loadAndroidDataset([folder_name 'noise1_4/']);
        calib_samples.acc_calib_sample = loadAndroidDataset([folder_name 'calib_acc/']);
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset([folder_name 'noise1_4/']);
        
        % load noise sample
        noise_sample = loadAndroidDataset([folder_name 'noise1_4/']); 
    
    case ( 'lopen1.3' )
        folder_name = 'datasets/marie testing/lopen1.3/';
        
        shs_sample = loadAndroidDataset([folder_name 'lopen1_3/']);
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset([folder_name 'calib_mag/']);
        calib_samples.gyr_calib_sample = loadAndroidDataset([folder_name 'noise/']);
        calib_samples.acc_calib_sample = loadAndroidDataset([folder_name 'calib_acc/']);
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset([folder_name 'noise/']);
        
        % load noise sample
        noise_sample = loadAndroidDataset([folder_name 'noise/']); 
    
    case ( 'lopen1.2' )
        folder_name = 'datasets/marie testing/lopen1.2/';
        
        shs_sample = loadAndroidDataset([folder_name 'lopen1_2/']);
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset([folder_name 'calib_mag/']);
        calib_samples.gyr_calib_sample = loadAndroidDataset([folder_name 'noise/']);
        calib_samples.acc_calib_sample = loadAndroidDataset([folder_name 'calib_acc/']);
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset([folder_name 'noise/']);
        
        % load noise sample
        noise_sample = loadAndroidDataset([folder_name 'noise/']);      
        
    case ( 'lopen1.1' )
        shs_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/lopen1_1/');
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/calib_mag/');
        calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/noise/');
        calib_samples.acc_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/calib_acc/');
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/noise/');
        
        % load noise sample
        noise_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/noise/');
        
        
        %% walking inside with recording walking through doors
        
    case ( 'inside_walking_record_ns_marie' )
        shs_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/walking_around_door_record/');
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_mag/');
        calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
        calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_acc/');
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
        
        % load noise sample
        noise_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
        
        
        %% walking inside without recording walking through doors
        
    case ( 'inside_walking_ns_marie' )
        shs_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/binnen_lopen_1_geen_smartwatch/');
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_mag/');
        calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
        calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_acc/');
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
        
        % load noise sample
        noise_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
        
        %%
    case ( 'inside_stationary_marie' )
        shs_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/stationary_test/');
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_mag/');
        calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
        calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_acc/');
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
        
        % load noise sample
        noise_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
        
        
        %% walking outside dataset
        
    case ( 'outside' )
        shs_sample = loadAndroidDataset('datasets/one plus nord/13 October/two_times_around_the_block/');
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/13 October/calib_mag/');
        calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/13 October/calib_gyr/');
        calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/13 October/calib_acc/');
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/13 October/magnetic_north_buiten/');
        
        % load noise sample
        noise_sample = loadAndroidDataset('datasets/one plus nord/13 October/magnetic_north_buiten/');
        
        %%
    case ( 'stationary' )
        shs_sample = loadAndroidDataset('datasets/one plus nord/15 October/stationary_test/');
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October/calib_mag/');
        calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October/calib_gyr/');
        calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October/calib_acc/');
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/15 October/noise/');
        
        % load noise sample
        noise_sample = loadAndroidDataset('datasets/one plus nord/15 October/noise/');
        
        %%
    case ( 'inside' )
        shs_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/walking_inside/');
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/calib_mag/');
        calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/calib_gyr/');
        calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/calib_acc/');
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/noise/');
        
        % load noise sample
        noise_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/noise/');
        
        %%
    case ( 'inside tracked' )
        shs_sample = loadAndroidDataset('datasets/one plus nord/17 October/walking_around_inside/');
        
        % load calibration data
        calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/17 October/calib_mag/');
        calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/17 October/laying_on_the_floor/');
        calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/17 October/calib_acc/');
        
        % load magnetic north data
        magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/17 October/laying_on_the_floor/');
        
        % load noise sample
        noise_sample = loadAndroidDataset('datasets/one plus nord/17 October/laying_on_the_floor/');
        
    otherwise
        error('this data set id cannot be found')
end
end