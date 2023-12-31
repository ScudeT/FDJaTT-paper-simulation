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
        function obj = Robot_w_sens_and_comm(x0,name,stat_data,sensor_data, connection_data)
            % connection_data = [connection_max_dist, connection prob]
            
            obj@Robot_w_sensors(x0,name,stat_data,sensor_data)
            
            link_prob = connection_data{2};

            obj.link_max_dist = connection_data{1};
            obj.abs_link_prob = link_prob(1);
            obj.rel_link_prob = link_prob(2);

            obj.data_rel = table('Size', [0 6], 'VariableTypes', {'double','double','double','double','double','double'}, 'VariableNames', {'Name', 'Distance','Direction','Pose_estimate', 'Error_covariance','Time'});
        end

        function obj = update_rel_com(obj,robot_set, time_stamp)
            
            obj.data_rel = table('Size', [0 6], 'VariableTypes', {'double','double','double','double','double','double'}, 'VariableNames', {'Name', 'Distance','Direction','Pose_estimate', 'Error_covariance','Time'});
            
            for j=1:length(robot_set)
                robot = robot_set(j);
                hisRow = ismember(obj.z_r.Name, robot.name);
                distance = obj.z_r{hisRow, 'Distance'};
                myRow = ismember(robot.z_r.Name, obj.name);
                my_distance = robot.z_r{myRow, 'Distance'};
                my_direction = robot.z_r{myRow, 'Direction'};

                if isempty(distance)
                    % don't add if he is not measured
                elseif isempty(my_distance)
                    % don't add if i don't have his measurement
                else

                    if distance < obj.link_max_dist
                    % if the robot is reachable then add the measurement 
                    % to be changed to follow the one hop rule
                        if rand < obj.rel_link_prob
            
                            rel_data = {robot.name, my_distance, my_direction, robot.Pose_est, robot.p_est, time_stamp };
                            
                            obj.data_rel = [obj.data_rel; rel_data];
            
                        end
                    end
                end
      
            end
        end
        
        function obj = simple_rel_com(obj,robot_set, time_stamp)
            
            obj.data_rel = table('Size', [0 6], 'VariableTypes', {'double','double','double','double','double','double'}, 'VariableNames', {'Name', 'dX','dY','Pose_estimate', 'Error_covariance','Time'});
            
            for j=1:length(robot_set)
                robot = robot_set(j);
                hisRow = ismember(obj.z_r.Name, robot.name);
                distance = obj.z_r{hisRow, 'Distance'};
                myRow = ismember(robot.z_r.Name, obj.name);
                my_distance = robot.z_r{myRow, 'Distance'};
                my_dx = robot.z_r{myRow, 'dX'};
                my_dy = robot.z_r{myRow, 'dY'};

                if isempty(distance)
                    % don't add if he is not measured
                elseif isempty(my_distance)
                    % don't add if i don't have his measurement
                else

                    if distance < obj.link_max_dist
                    % if the robot is reachable then add the measurement 
                    % to be changed to follow the one hop rule
                        if rand < obj.rel_link_prob
            
                            rel_data = {robot.name, my_dx, my_dy, robot.Pose_est, robot.p_est, time_stamp };
                            
                            obj.data_rel = [obj.data_rel; rel_data];
            
                        end
                    end
                end
      
            end
        end

        function obj = update_com(obj,robot_set,time_stamp)
            obj = obj.update_rel_com(robot_set,time_stamp);    
        end

    end
end