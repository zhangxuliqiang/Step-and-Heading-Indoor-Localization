
%% absolute true positives
bar_comp_data = [];
bar_names = {};
n = 0;
figure()
for comparison = gt2algo_comparisons'
    n = n + 1;
    % formatting name for nice plotting
    name = strsplit(strrep(comparison.name,'_',' '),'5');
    bar_names(n) = {name{1}(1:end-2)};
    
    %grouping dataset together
    bar_comp_data = [ bar_comp_data;...
        max([comparison.matlab_pseudo_confusion.true_positive]), ...
        max([comparison.android_pseudo_confusion.true_positive]), ...
        comparison.ground_truth_steps.nr_steps ];
    
end
bar_names = categorical(bar_names);
bar(bar_names,bar_comp_data)
ylabel('number of steps')
title("step true positives compared to ground truth detection")
legend('matlab algorithm true positives','android algorithm true positives','ground truth detection')

%% percentual true positives
bar_comp_data = [];
max_matlab_tp_delta_t_table = [];
max_android_tp_delta_t_table = [];
bar_names = {};
n = 0;
figure()
for comparison = gt2algo_comparisons'
    n = n + 1;
    % formatting name for nice plotting
    name = strsplit(strrep(comparison.name,'_',' '),'5');
    bar_names(n) = {name{1}(1:end-2)};
    
    [max_matlab_tp, max_matlab_tp_index] = max([comparison.matlab_pseudo_confusion.true_positive],[],2,'linear');
    [max_android_tp, max_android_tp_index] = max([comparison.android_pseudo_confusion.true_positive],[],2,'linear');
    
    max_matlab_tp_delta_t = comparison.matlab_pseudo_confusion(max_matlab_tp_index).delta_t;
    max_matlab_tp_delta_t_table = [max_matlab_tp_delta_t_table, max_matlab_tp_delta_t];
    
    max_android_tp_delta_t = comparison.matlab_pseudo_confusion(max_android_tp_index).delta_t;
    max_android_tp_delta_t_table = [max_android_tp_delta_t_table, max_android_tp_delta_t];
    
    %grouping dataset together
    bar_comp_data = [ bar_comp_data; ( max_matlab_tp - comparison.ground_truth_steps.nr_steps)/comparison.ground_truth_steps.nr_steps*100, ...
        (max_android_tp - comparison.ground_truth_steps.nr_steps)/comparison.ground_truth_steps.nr_steps*100, ...
        ];
    
end
bar_names = categorical(bar_names);
b = bar(bar_names,bar_comp_data);

xtips1 = b(1).XEndPoints;
ytips1 = b(1).YEndPoints;
labels1 = string(max_matlab_tp_delta_t_table);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','top')

xtips2 = b(2).XEndPoints;
ytips2 = b(2).YEndPoints;
labels2 = string(max_android_tp_delta_t_table);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','top')

ylabel('percent error from ground truth (%)')
title("step true positives error compared to ground truth detection")
legend('matlab algorithm true positives','android algorithm true positives')

%%
figure()
hold on
plot([pseudo_confusion.delta_t], [pseudo_confusion.true_positive]);
plot([pseudo_confusion.delta_t], [pseudo_confusion.false_negative]);
plot([pseudo_confusion.delta_t], [pseudo_confusion.false_positive]);
hold off
% title('unique matlab step to ground truth step link

%%

figure()
plot(sd.sd_components.Time, sd.sd_components.acc0_magnitude)
hold on
scatter(sd_comparison.matlab_algo_steps.data.Time, ...
    sd_comparison.matlab_algo_steps.data.acc0_magnitude)

scatter(sd_comparison.ground_truth_steps.data.Time, ...
    sd.sd_components(sd_comparison.ground_truth_steps.data.Time,:).acc0_magnitude)

scatter(sd_comparison.salvi_algo_steps.data.Time, ...
    sd.sd_components(sd_comparison.salvi_algo_steps.data.Time,:).acc0_magnitude)
legend('acceleration norm','matlab algo','ground truth','Salvi et al algo')

% legend('acceleration norm','ground truth','Salvi et al algo')
hold off

xlim([seconds(40) seconds(60)])
ylim([0 25])
xlabel('Time')
ylabel('Acceleration (m/s^2)','Interpreter','tex')
title('Accelerometer magnitude with ground truth steps \newline and Salvi et al. step detection', ...
      'Interpreter','tex')

%%

diff_truth = diff(comparison.ground_truth_steps.data.Time);
figure()
% histogram(diff_truth)
plot(diff_truth)
title('truth diff')

diff_matalgo = diff(comparison.matlab_algo_steps.data.Time);
figure()
% histogram(diff_matalgo)
plot(diff_matalgo)
title('matlab diff')

diff_algo = diff(comparison.salvi_algo_steps.data.Time);
figure()
% histogram(diff_algo)
plot(diff_algo)
title('algo diff')

spreadfigures

%%
stackedplot(diff_truth, diff_matalgo, diff_algo)