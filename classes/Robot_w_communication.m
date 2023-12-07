classdef Robot_w_communication < Robot_w_sensors
    % giving the ability to the robots to communicate their data to
    % eachother

    properties
        data_abs
        data_rel
        link_max_dist
        link_failure_prob
    end

    methods
        function obj = Robot_w_communication(x0,name,sensors_var, sensors_range, connection_data)
            % connection_data = [connection_max_dist, connection prob]

            obj@Robot_w_sensors(x0,name,sensors_var, sensors_range)

            obj.data_abs = table('Size', [0 5],'VariableTypes', {'double','double','double','double','double'},'VariableNames', {'Name', 'X', 'Y','Heading','Time'});
            obj.data_rel = table('Size', [0 4],'VariableTypes', {'double','double','double','double'},'VariableNames', {'Name', 'Distance', 'Direction','Time'});

            obj.link_max_dist = connection_data(1);
            obj.link_failure_prob = connection_data(2);
        end

        function obj = get_data(obj,robot,time_stamp)  
            if rand > obj.link_failure_prob
                abs_data = table({robot.name}, robot.z_abs(1), robot.z_abs(2), robot.z_abs(3),time_stamp, 'VariableNames', {'Name', 'X', 'Y','Heading','Time'});

                myRow = ismember(robot.z_r.Name, obj.name);
                my_distance = robot.z_r{myRow, 'Distance'};
                my_direction = robot.z_r{myRow, 'Direction'};

                rel_data = table({robot.name}, my_distance, my_direction, time_stamp, 'VariableNames', {'Name', 'Distance', 'Direction','Time'});
                
                obj.data_abs = [obj.data_abs; abs_data];
                obj.data_rel = [obj.data_rel; rel_data];

            end
        end

        function obj = update_communications(obj,robot_set, time_stamp)
            obj.data_abs = table('Size', [0 5], 'VariableTypes', {'double','double','double','double','double'},'VariableNames', {'Name', 'X', 'Y','Heading','Time'});
            obj.data_rel = table('Size', [0 4], 'VariableTypes', {'double','double','double','double'}, 'VariableNames', {'Name', 'Distance', 'Direction','Time'});
            
            for j=1:length(robot_set)
                hisRow = ismember(obj.z_r.Name, robot_set(j).name);

                if isempty(hisRow)
                    % don't add if he is so far I can't measure him
                else
                    distance = obj.z_r{hisRow, 'Distance'};

                    if distance < obj.link_max_dist
                    % if the robot is reachable then add the measurement 
                    % to be changed to follow the one hop rule
                        obj = obj.get_data(robot_set(j),time_stamp);
                    end
                end
      
            end
        end
    end
end