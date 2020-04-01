classdef Queue < handle
    properties ( Access = public )
        elements 
    end

    methods
        function obj = Queue
            obj.elements = timetable;

        end
        function enqueue( obj, new_element )
            obj.elements = [obj.elements; new_element];
        end
        function first_element = dequeue( obj )
            if isempty(obj.elements)
                error( 'Queue is empty' );
            end
            first_element = obj.elements(1,:);
            obj.elements(1,:) = []; 
        end
        function queue_length = getLength( obj )
            queue_length = height(obj.elements);
        end
    end
end