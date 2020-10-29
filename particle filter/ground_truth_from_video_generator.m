close all
directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/marie testing';

video.trial_name = 'lopen1.1';
video.video_name = '/QVR_2020_10_20_16_39_52.mp4';
video.start_time = 35; %in seconds
video.end_time = 170;

%%
video.trial_name = 'lopen1.2';
video.video_name = '/QVR_2020_10_20_16_49_54.mp4';
video.start_time = 33; %in seconds
video.end_time = 163;
%%
video.trial_name = 'lopen1.3';
video.video_name = '/QVR_2020_10_20_17_31_45.mp4';
video.start_time = 53; %in seconds
video.end_time = 272;

%%
video.trial_name = 'lopen1.5';
video.video_name = '/QVR_2020_10_20_17_58_11.mp4';
video.start_time =30; %in seconds
video.end_time = 209;
%%

vidObj = VideoReader([directory video.video_name]);

prev_time = 0;
index = 1;

figure(1)
show(walls)

vidObj.CurrentTime = video.start_time;

 while vidObj.CurrentTime <= video.end_time
     figure(2)
    frame = readFrame(vidObj);
    I = imresize(frame, 0.25);
    J = imrotate(I,-90);
    imshow(J);
    
    if vidObj.CurrentTime - prev_time > 1
        prev_time = vidObj.CurrentTime;
        figure(1)
        click_point = ginput(1);
        position_data(index).time = vidObj.CurrentTime - video.start_time;
        position_data(index).x = click_point(1);
        position_data(index).y = click_point(2);
        hold on
        plot(click_point(1),click_point(2),'xb')
        hold off
        index = index + 1;
    end
 end
 
 %% writing to a csv file for long term saving
 
 mat_position_data =  [[position_data.time]',[position_data.x]',[position_data.y]'];
 
 variable_names = ["time","x","y"];
 
 text = ['this is the ground truth data generated from ' video.video_name ...
     ' with trial name: ' video.trial_name];
 
 writematrix(text,[video.trial_name '_gt_from_video.csv'])
 
 writematrix(variable_names,[video.trial_name '_gt_from_video.csv'],'WriteMode','append')
 writematrix(mat_position_data,[video.trial_name '_gt_from_video.csv'],'WriteMode','append')
 
 file_name = [video.trial_name '_gt_from_video.csv'];

 %%
 figure(4)
 hold on
 plot([position_data.x],[position_data.y],'b')
 %%
 pos_data = struct2table(position_data);
 pos_data.time = seconds(pos_data.time);
 pos_data = table2timetable(pos_data);
 
 pos_data = retime(pos_data, shs.position.time, 'linear');
 
 figure
 subplot(2,1,1)
 hold on 
 plot(pos_data.time, pos_data.x)
 plot(shs.position.time, shs.position.x + 12)
 hold off
 subplot(2,1,2)
 hold on 
 plot(pos_data.time, pos_data.y)
 plot(shs.position.time, shs.position.y + 12)
 hold off