% close all

% generate color scheme struct per user id 
values = rand(length(unique_user_ids_genders),3);
color_scheme = struct('unique__user_ids', ...
    mat2cell(unique_user_ids_genders(:,1),ones(1,length(unique_user_ids_genders))),...
                      'values',mat2cell(values,ones(1,length(values))));

%% plot all user lines and markers depending on walking speed
user_ids = [];
t = [];
figure
i = 1;
for specific_freq_com = freq_comp
        color_index = find([color_scheme.unique__user_ids] == ...
            specific_freq_com.name);        
        
        
        
        x = [specific_freq_com.proc_data(:).height_mult_sqrt_freq];
        z = [specific_freq_com.proc_data(:).path_length_div_nr_steps];
        color =color_scheme(color_index).values;
        
        if color_index == 14 
            x(3) = nan;
            z(3) = nan;
        end
        
        hold on
        t(i) = plot(x,z,'-','color',color);
        scatter(x(1),z(1),'x','MarkerEdgeColor',color)
        scatter(x(2),z(2),'o','MarkerEdgeColor',color)
        scatter(x(3),z(3),'^','MarkerEdgeColor',color)
        hold off
        i  = i+1;    
end
xlabel('user height*sqrt(frequency) (m /sqrt(s))', 'interpreter','tex')
ylabel('path length/ nr steps detected (m)')
title('Frequency based step length estimation')
% plot LSQR tian step length estimation
hold on 
x = 1:0.1:3;
y = tian_constant*x;
t(end+1) = plot(x,y,'--','color',[0.4660, 0.6740, 0.1880],'LineWidth',3);
hold off

xlim([1,2.7])
xlim([1.4,2.7])


% Line legend
leg_entries = [freq_comp.name];
leg = legend(t(1:end-1),leg_entries,'Location','West');
title(leg,'User ID')

% Marker shape legend
ah1=axes('position',get(gca,'position'),'visible','off');
hold on
markers(1) = scatter(nan, nan, 'x','MarkerEdgeColor','black');
markers(2) = scatter(nan, nan, 'o','MarkerEdgeColor','black');
markers(3) = scatter(nan, nan, '^','MarkerEdgeColor','black');
hold off
leg2=legend(ah1,markers,'fast','normal','slow','Location','NorthWest');
title(leg2,'Walking Speed')

% Estimator Legend
ah2=axes('position',get(gca,'position'),'visible','off');
leg3=legend(ah2,t(end),'Tian Step Length Estimator','Location','SouthWest');
title(leg3, 'Least Square Estimation')
%%
gender = strcmp([freq_comp.gender], "male");

gender_data = freq_comp(gender);

male__plotting_data = arrayfun(@(S) [[S.proc_data.height_mult_sqrt_freq]',...
    [S.proc_data.path_length_div_nr_steps]'], gender_data(:), ...
    'UniformOutput', false);

male__plotting_data = cell2mat(male__plotting_data);
corrupt_data = find(isnan(male__plotting_data(:,1))');

male__plotting_data(corrupt_data,:) = [];

x = lsqr(male__plotting_data(:,1),male__plotting_data(:,2))