function [watch_data, activity] = generateGroundTruth(ar_set)

    watch_data = WatchFile2Timetable([ar_set.file_path ar_set.watch_file_name]);

    % load ground truth timeset in which a door handle was used

    load([ar_set.file_path ar_set.gt_file_name])
    activity.time = datetime(data(:,1),'ConvertFrom','datenum');
    activity.time.TimeZone = 'local';
    activity.time.Format = 'yyyy-MM-dd HH:mm:ss.SSS';

    % find sections in which activity occurs
    activity.sections = [];
    start_section = activity.time(1);

    for i  = 2: length(activity.time)
        time_diff  = seconds(activity.time(i)-activity.time(i-1));

        if time_diff> 1

            end_section = activity.time(i-1);
            activity.sections = [activity.sections; start_section, end_section];

            start_section  = activity.time(i);
        end

    end

    % remove sections that have the same start and end time 
    activity.sections(activity.sections(:,1) == activity.sections(:,2),: ) = [];

    for i  = 1: length(activity.sections)
       tr_activity = timerange(activity.sections(i,1), activity.sections(i,2));

       watch_data(tr_activity,:).door_open  = ones(height( watch_data(tr_activity,:)),1);
    end

end