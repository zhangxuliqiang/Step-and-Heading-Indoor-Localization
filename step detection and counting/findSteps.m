function steps = findSteps(target, target_timetable)
% find the index of step taken and find the raw data that corresponds to it
step_index = find(target);
steps.data = target_timetable(step_index,:);

% find the total amount steps taken
[steps.nr_steps, ~] = size(step_index);

end