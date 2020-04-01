TruthData = user1armband1506423438471(1:1000,:);

elapsed = (TruthData.timestamp(:)-TruthData.timestamp(1));
milli2sec = elapsed.*1E-9;
TruthData.timestamp = seconds(milli2sec);
TruthData = table2timetable(TruthData);

TruthData.Properties.VariableNames = ["X","Y","Z",'algo_steps','gt_steps','hw_steps'];
%%
figure()
stackedplot(TruthData)
%%
TruthData.step_detect = [0; diff(TruthData.gt_steps)];

step_detect_time = find(TruthData.step_detect);

[height, width] = size(step_detect_time);
 step_time = TruthData(step_detect_time(:,1),:).timestamp;

%%
clear figure(2)
figure(2)
hax=axes;

hold on
plot(step_detection.Time,step_detection.acc1_gauss)
scatter(step_detection.Time,step_detection.acc4_max)

% for i = 1:height
%     step_time = TruthData(step_detect_time(i,1),:).timestamp;
%     line([step_time step_time],get(hax,'YLim'),'Color',[1 0 0])
% end

for i = 1:length(slidding_window)
    line([slidding_window(i) slidding_window(i)],get(hax,'YLim'),'Color',[0 1 0])
end
hold off