%% sd_and_c_salvi_comp_plotter_scripts

% SCRIPT USE
% The following script is used to plot signals for illustration and
% different performance measures of the stepDetection function. 
% They are sectioned so that they can be run seperately with ctrl=shift

% NOTE
% the script is not standalone and requires the workspace to contain
% the output of sd_and_c_ground_truth_comparison.m 

%% Illustration: Difference in signal profile depending on carrying mode
close all

t = tiledlayout((length(gt2algo_comparisons)/2),1);
 
for i = 1:length(gt2algo_comparisons)/2
    
    sd_comparison = gt2algo_comparisons(i);
    raw_data = sd_comparison.raw_data;
    raw_data.magnitude = sqrt(raw_data.X.^2 + raw_data.Y.^2 + ...
        raw_data.Z.^2);
    
    ax(i) = nexttile;
    hold on
    plot(ax(i),raw_data.Time,raw_data.magnitude);    

    gt_plot = scatter(ax(i),sd_comparison.ground_truth_steps.data.Time, ...
        raw_data(sd_comparison.ground_truth_steps.data.Time,:).magnitude);

    algo_plot = scatter(ax(i),sd_comparison.salvi_algo_steps.data.Time, ...
        raw_data(sd_comparison.salvi_algo_steps.data.Time,:).magnitude);
    hold off
% legend('acceleration norm','matlab algo','ground truth','Salvi et al algo')

    
    % formatting name for nice plotting by replacing underscore with space
    name_format = strrep(gt2algo_comparisons(i).name,'_',' ');
    
    %cut the string at the char 5 so that date info is not seen in plot
    name = strsplit(name_format,'5');
    
    title([name{1}(1:end-2)])
    legend([gt_plot, algo_plot],'gt step','algo step',...
        'Orientation', 'horizontal');
end
title(t,'Acceleration magnitude profile of different carrying modes')
ylabel(t,'Acceleration (m/s^2)')
xlabel(t,'Time')
t.TileSpacing = 'compact';
t.Padding = 'compact';
linkaxes(ax,'xy')

xlim([seconds(10),seconds(20)])


%% Performance Measure: Absolute and percentual number of steps detected
close all

bar_comp_data = [];
bar_names = {};
n = 0;

figure()

for comparison = gt2algo_comparisons'
   n = n + 1;
   % formatting name for nice plotting by replacing underscore with space
   name_format = strrep(comparison.name,'_',' ');
   
   %cut the string at the char 5 so that date info is not seen in plot
   name = strsplit(name_format,'5');
   
   %only use the name
   bar_names(n) = {name{1}(1:end-2)}; 
   bar_comp_data = [ bar_comp_data; ...                     
                     comparison.matlab_algo_steps.nr_steps, ...
                     comparison.salvi_algo_steps.nr_steps, ...
                     comparison.ground_truth_steps.nr_steps];    
end
bar_names = categorical(bar_names);
bar(bar_names,bar_comp_data)
ylabel('number of steps')
title('step counting comparison') 
legend('matlab algorithm','salvi et al algorithm','ground truth')

% step counting percentual accuracy

bar_comp_data = [];
bar_names = {};
n = 0;
figure(2)
for comparison = gt2algo_comparisons'
   n = n + 1;
   % formatting name for nice plotting
   name = strsplit(strrep(comparison.name,'_',' '),'5');
   bar_names(n) = {name{1}(1:end-2)}; 
   
   gt_nr_steps = comparison.ground_truth_steps.nr_steps;
   
   % calculating percentages and grouping dataset together
   bar_comp_data = [ bar_comp_data;  ... 
   (comparison.matlab_algo_steps.nr_steps - gt_nr_steps)./gt_nr_steps *100, ...
   (comparison.salvi_algo_steps.nr_steps - gt_nr_steps)./gt_nr_steps * 100];    
end
bar_names = categorical(bar_names);
bar(bar_names,bar_comp_data)
ylabel('percentage error (%)')
title("step counting error") 
legend('matlab algorithm','salvi et al algorithm')


%% Absolute and percentual true positives between ground truth and 
%  stepDetection output
close all

bar_comp_data_tp = [];
bar_comp_data_fp = [];
bar_names = {};
n = 0;

for comparison = gt2algo_comparisons'
    n = n + 1;
    % formatting name for nice plotting
    name = strsplit(strrep(comparison.name,'_',' '),'5');
    bar_names(n) = {name{1}(1:end-2)};
    
    % finding largest true postive count 
     % finding value and index of maximum number of 
    [max_matlab_tp, max_matlab_tp_index] = ...
        max([comparison.matlab_pseudo_confusion.true_positive],[],2,'linear');
    
    [max_android_tp, max_android_tp_index] = ...
        max([comparison.android_pseudo_confusion.true_positive],[],2,'linear');
    
    % group dataset together for bar chart plotting 
    bar_comp_data_tp = [ bar_comp_data_tp;
        max_matlab_tp, max_android_tp, ...
        comparison.ground_truth_steps.nr_steps ];
    
    max_matlab_fp = ...
        comparison.matlab_pseudo_confusion(max_matlab_tp_index).false_positive;
    max_android_fp = ...
        comparison.android_pseudo_confusion(max_android_tp_index).false_positive;
    
    bar_comp_data_fp = [ bar_comp_data_fp;
        max_matlab_fp, max_android_fp, 0 ];
    
end

groupLabels = bar_names; 
stackData(:,:,1) = bar_comp_data_tp;
stackData(:,:,2) = bar_comp_data_fp;
h = plotBarStackGroups(stackData, groupLabels);  
ylim([150,500])

h(1,2).FaceColor=[0.1 0.8 0.8];
h(2,1).FaceColor=[0.8500 0.3250 0.0980];
h(2,2).FaceColor=[0.9000 0.5 0.3];
h(3,1).FaceColor=[0.9290 0.6940 0.1250];
% xlabel('carrying mode');
ylabel('true positive number of steps')
title('step detection true positives and false positives \newline compared to ground truth','Interpreter','tex')
legend('matlab true positive','matlab false positive', ...
       'salvi et al true positive','salvi et al false positive', ...
       'ground truth')


%% percentual true positives and the delta_t that facilitates it
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
    
    % finding value and index of maximum number of 
    [max_matlab_tp, max_matlab_tp_index] = ...
        max([comparison.matlab_pseudo_confusion.true_positive],[],2,'linear');
    
    [max_android_tp, max_android_tp_index] = ...
        max([comparison.android_pseudo_confusion.true_positive],[],2,'linear');
    
    max_matlab_tp_delta_t = ...
        comparison.matlab_pseudo_confusion(max_matlab_tp_index).delta_t;
    
    max_matlab_tp_delta_t_table = ...
        [max_matlab_tp_delta_t_table, max_matlab_tp_delta_t];
    
    max_android_tp_delta_t = ...
        comparison.matlab_pseudo_confusion(max_android_tp_index).delta_t;
    
    max_android_tp_delta_t_table = ...
        [max_android_tp_delta_t_table, max_android_tp_delta_t];
    
    gt_nr_steps = comparison.ground_truth_steps.nr_steps;
    
    %grouping dataset together
    bar_comp_data = ...
        [ bar_comp_data; ...
        ( max_matlab_tp - gt_nr_steps)/gt_nr_steps*100, ...
        (max_android_tp -gt_nr_steps)/gt_nr_steps*100, ...
        ];
    
end
bar_names = categorical(bar_names);
b = bar(bar_names,bar_comp_data);

xtips1 = b(1).XEndPoints;
ytips1 = b(1).YEndPoints;
labels1 = string(max_matlab_tp_delta_t_table);
point_label = text(xtips1,ytips1,labels1,'HorizontalAlignment','right',...
    'VerticalAlignment','top','fontsize',8);

set(point_label,'Rotation',45);

xtips2 = b(2).XEndPoints;
ytips2 = b(2).YEndPoints;
labels2 = string(max_android_tp_delta_t_table);
point_label= text(xtips2,ytips2,labels2,'HorizontalAlignment','right',...
    'VerticalAlignment','top','fontsize',8)

set(point_label,'Rotation',45);

ylabel('percent error from ground truth (%)')
title("step true positives error compared to ground truth detection")
legend('matlab algo','salvi et al algo')

%%

pseudo_confusion = comparison.android_pseudo_confusion;
figure()
hold on
plot([pseudo_confusion.delta_t], [pseudo_confusion.true_positive]);
plot([pseudo_confusion.delta_t], [pseudo_confusion.false_negative]);
plot([pseudo_confusion.delta_t], [pseudo_confusion.false_positive]);
plot([pseudo_confusion.delta_t], [pseudo_confusion.tp_double_count]);
hold off
legend('true positive','false negative','false positive','tp double count')
title([comparison.name])
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
title(['Accelerometer magnitude with ground truth steps \newline' 
       'and Salvi et al. step detection'], ...
      'Interpreter','tex')

%%

diff_truth = diff(sd_comparison.ground_truth_steps.data.Time);
figure()
% histogram(diff_truth)
plot(diff_truth)
title('truth diff')

diff_matalgo = diff(sd_comparison.matlab_algo_steps.data.Time);
figure()
% histogram(diff_matalgo)
plot(diff_matalgo)
title('matlab diff')

diff_algo = diff(sd_comparison.salvi_algo_steps.data.Time);
figure()
% histogram(diff_algo)
plot(diff_algo)
title('algo diff')

spreadfigures

%%

