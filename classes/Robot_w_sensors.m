classdef Robot_w_sensors < Robot

% this gives the ability to the robot to sence its position and the
% position of the robots and targets around him

    properties
        name

        z_abs   % absolute measurement
        v_abs   % absolute measurement variance
        abs_mes_prob

        z_r     % distance and direction of other robots
        v_r     % robot measurement variance
        z_r_max % max measurement distance
        rel_mes_prob

        z_t     % distance and direction of targets
        v_t     % target measurement varaince
        z_t_max % max measurement distance
    end

    methods
        function obj = Robot_w_sensors(x0,name,stat_data,sensor_data)
            % variances = [v_abs; v_r; v_t]
            % max_distances = [max_z_r; max_z_t]
            obj@Robot(x0,stat_data);
            
            variances = sensor_data{1};
            max_distances = sensor_data{2};
            probabilities = sensor_data{3};

            obj.v_abs   = variances(1);
            obj.v_r     = variances(2);
            obj.v_t     = variances(3);

            obj.name = name;

            obj.z_abs = table('Size', [0 5], 'VariableTypes', {'double','double','double','double','double'},'VariableNames', {'Name', 'X', 'Y','Heading','Time'});
            obj.abs_mes_prob = probabilities(1);

            obj.z_r = table('Size', [0 4], 'VariableTypes', {'double','double','double','double'}, 'VariableNames', {'Name', 'Distance','Direction','Time'});
            obj.z_t = table('Size', [0 4], 'VariableTypes', {'double','double','double','double'}, 'VariableNames', {'Name', 'Distance','Direction','Time'});
            obj.z_r_max = max_distances(1);
            obj.rel_mes_prob = probabilities(2);

        end

        function obj = update_abs_meas(obj, time_stamp)
            % abs measurement is the true pose of the robot + some noise
            measured_pose = obj.true_Pose + obj.v_abs*randn(3,1);
            obj.z_abs = table('Size', [0 5], 'VariableTypes', {'double','double','double','double','double'},'VariableNames', {'Name', 'X', 'Y','Heading','Time'});

            if rand < obj.abs_mes_prob
                obj.z_abs = table({obj.name}, measured_pose(1), measured_pose(2), measured_pose(3), time_stamp, 'VariableNames', {'Name', 'X', 'Y','Heading','Time'});
            end
        end

        function measure = rel_sensor_model(obj,robot)
            % dist and direction of the measurement are calculated from the
            % true position of the robots, then some noise is added
            x_i = obj.true_Pose(1);
            y_i = obj.true_Pose(2);
            th_i = obj.true_Pose(3);

            x_l = robot.true_Pose(1);
            y_l = robot.true_Pose(2);
            th_l = robot.true_Pose(3);

            dx = x_l-x_i;
            dy = y_l-y_i;

            dist = sqrt((dx)^2+(dy)^2) + obj.v_r*randn;
            angle = wrapTo2Pi(atan2(dy,dx)) - th_i + obj.v_r*randn;
            
            measure = [dist; wrapTo2Pi(angle)];
        end

        function obj = update_rel_meas(obj, robot_set, time_stamp)
            % the robot measurement table is reset with the new values of
            % the measurements 
            
            obj.z_r = table('Size', [0 4], 'VariableTypes', {'double','double','double','double'}, 'VariableNames', {'Name', 'Distance','Direction','Time'});

            for j=1:length(robot_set)
                current_robot = robot_set(j);
                measure = obj.rel_sensor_model(current_robot);

                if current_robot.name == obj.name 
                    % don't add measurement against your own
                elseif measure(1) < obj.z_r_max 
                    % if the robot is reachable then save it 
                    if rand < obj.rel_mes_prob
                        newRow = table( current_robot.name, measure(1),measure(2), time_stamp, 'VariableNames', {'Name', 'Distance','Direction','Time'});
                        obj.z_r = [obj.z_r; newRow];
                    end
                end
      
            end
        end

        function obj = simple_rel_meas(obj, robot_set, time_stamp)
            % the robot measurement table is reset with the new values of
            % the measurements 
            
            obj.z_r = table('Size', [0 5], 'VariableTypes', {'double','double','double','double','double'}, 'VariableNames', {'Name','Distance', 'dX','dY','Time'});

            for j=1:length(robot_set)
                current_robot = robot_set(j);
                measure = obj.rel_sensor_model(current_robot);
               
                
                if current_robot.name == obj.name 
                    % don't add measurement against your own
                elseif measure(1) < obj.z_r_max 
                    % if the robot is reachable then save it 
                    if rand < obj.rel_mes_prob
                        dX = current_robot.true_Pose - obj.true_Pose;

                        newRow = table( current_robot.name,measure(1), dX(1), dX(2), time_stamp, 'VariableNames', {'Name','Distance', 'dX','dY','Time'});
                        obj.z_r = [obj.z_r; newRow];
                    end
                end
      
            end
        end
        
        function obj = update_meas(obj, robot_set, time_stamp)
            obj = obj.update_abs_meas(time_stamp);
            obj = obj.update_rel_meas(robot_set, time_stamp);
        end

    end
end