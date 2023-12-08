classdef Robot_w_sens_and_comm < Robot_w_sensors
    % giving the ability to the robots to communicate their data to
    % eachother

    properties
        data_abs
        data_rel
        link_max_dist
        abs_link_prob
        rel_link_prob
    end

    methods
        function obj = Robot_w_sens_and_comm(x0,name,sensor_data, connection_data)
            % connection_data = [connection_max_dist, connection prob]
            
            obj@Robot_w_sensors(x0,name,sensor_data)
            
            link_prob = connection_data{2};

            obj.link_max_dist = connection_data{1};
            obj.abs_link_prob = link_prob(1);
            obj.rel_link_prob = link_prob(2);

            obj.data_abs = table('Size', [0 5],'VariableTypes', {'double','double','double','double','double'},'VariableNames', {'Name', 'X', 'Y','Heading','Time'});
            obj.data_rel = table('Size', [0 4],'VariableTypes', {'double','double','double','double'},'VariableNames', {'Name', 'Distance', 'Direction','Time'});
        end

        function obj = update_abs_com(obj,robot_set, time_stamp)
            obj.data_abs = table('Size', [0 5], 'VariableTypes', {'double','double','double','double','double'},'VariableNames', {'Name', 'X', 'Y','Heading','Time'});
            
            for j=1:length(robot_set)
                robot = robot_set(j);
                hisRow = ismember(obj.z_r.Name, robot.name);

                if isempty(hisRow)
                    % don't add if he is so far I can't measure him
                else
                    distance = obj.z_r{hisRow, 'Distance'};

                    if distance < obj.link_max_dist
                    % if the robot is reachable then add the measurement 
                    % to be changed to follow the one hop rule
                        if rand < obj.abs_link_prob
                            abs_data = table({robot.name}, robot.z_abs.X, robot.z_abs.Y, robot.z_abs.Heading,time_stamp, 'VariableNames', {'Name', 'X', 'Y','Heading','Time'});
                            obj.data_abs = [obj.data_abs; abs_data];
            
                        end
                    end
                end
      
            end
        end

        function obj = update_rel_com(obj,robot_set, time_stamp)
            obj.data_rel = table('Size', [0 4], 'VariableTypes', {'double','double','double','double'}, 'VariableNames', {'Name', 'Distance', 'Direction','Time'});
            
            for j=1:length(robot_set)
                robot = robot_set(j);
                hisRow = ismember(obj.z_r.Name, robot.name);
                
                if isempty(hisRow)
                    % don't add if he is so far I can't measure him
                else
                    distance = obj.z_r{hisRow, 'Distance'};

                    if distance < obj.link_max_dist
                    % if the robot is reachable then add the measurement 
                    % to be changed to follow the one hop rule
                        if rand < obj.rel_link_prob
            
                            myRow = ismember(robot.z_r.Name, obj.name);
                            my_distance = robot.z_r{myRow, 'Distance'};
                            my_direction = robot.z_r{myRow, 'Direction'};
            
                            rel_data = table({robot.name}, my_distance, my_direction, time_stamp, 'VariableNames', {'Name', 'Distance', 'Direction','Time'});
                            
                            obj.data_rel = [obj.data_rel; rel_data];
            
                        end
                    end
                end
      
            end
        end
        
        function obj = update_com(obj,robot_set,time_stamp)

            obj = obj.update_abs_com(robot_set,time_stamp);
            obj = obj.update_rel_com(robot_set,time_stamp);
            
        end
    end
end