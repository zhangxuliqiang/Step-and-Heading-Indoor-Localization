function result = findSteps(target, target_struct)
% find the number of step taken according to the ground truth
step_index = find(target);
steps.data = target_struct(step_index,:);
[steps.nr_steps, ~] = size(step_index);

result = steps;
end