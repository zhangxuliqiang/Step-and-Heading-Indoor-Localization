function result =TpFpFnCalc(algo_steps, ground_truth_steps,delta_t)

    tp_counter = 0;
    tp_double_counter = 0;
    
    for index = 1 : height(ground_truth_steps)
        ground_truth_time = ground_truth_steps(index, :).Time;
        time_range = timerange( ground_truth_time - seconds(delta_t), ground_truth_time + seconds(delta_t));
        found = algo_steps(time_range,:);
        
        if not(isempty(found))
            if height(found)>1
                tp_double_counter = tp_double_counter + 1;
            else
                tp_counter = tp_counter + 1;
            end           
        end        
    end
    
    fp_counter = 0;
    fp_double_counter = 0;
    
    for index = 1 : height(algo_steps)
        algo_time = algo_steps(index, :).Time;
        time_range = timerange( algo_time - seconds(delta_t), algo_time + seconds(delta_t));
        found = ground_truth_steps(time_range,:);
        
        if isempty(found)
            fp_counter = fp_counter + 1;
        elseif height(found)>1
            fp_double_counter = fp_double_counter + 1;
        end
    end
    
    temp_result.delta_t = delta_t;
    temp_result.true_positive = tp_counter;
    temp_result.false_positive = fp_counter;
    temp_result.false_negative = (height(ground_truth_steps) - tp_counter) ;
    temp_result.tp_double_count = tp_double_counter;
    temp_result.fp_double_count = fp_double_counter;
    
    result = temp_result;

end