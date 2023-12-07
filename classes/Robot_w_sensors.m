classdef Robot_w_sensors < Robot

% this gives the ability to the robot to sence its position and the
% position of the robots and targets around him

    properties
        name

        z_abs   % absolute measurement
        v_abs   % absolute measurement variance

        z_r     % distance and direction of other robots
        v_r     % robot measurement variance
        z_r_max % max measurement distance

        z_t     % distance and direction of targets
        v_t     % target measurement varaince
        z_t_max % max measurement distance
    end

    methods
        function obj = Robot_w_sensors(x0,name,variances, max_distances)
            % variances = [v_abs; v_r; v_t]
            % max_distances = [max_z_r; max_z_t]
            obj@Robot(x0);

            obj.v_abs   = variances(1);
            obj.v_r     = variances(2);
            obj.v_t     = variances(3);

            obj.name = name;
            obj.z_abs = x0 + obj.v_abs*randn(3,1);

            obj.z_r = table('Size', [0 4], 'VariableTypes', {'double','double','double','double'}, 'VariableNames', {'Name', 'Distance','Direction','Time'});
            obj.z_t = table('Size', [0 4], 'VariableTypes', {'double','double','double','double'}, 'VariableNames', {'Name', 'Distance','Direction','Time'});

            obj.z_r_max = max_distances(1);
            obj.z_t_max = max_distances(2);
        end

        function obj = measure_abs(obj)
            % abs measurement is the true pose of the robot + some noise
            obj.z_abs = obj.true_Pose + obj.v_abs*randn(3,1);
        end

        function measure = measure_robot(obj,robot)
            % dist and direction of the measurement are calculated from the
            % true position of the robots, then some noise is added
            dist = sqrt((robot.true_Pose(1)-obj.true_Pose(1))^2+(robot.true_Pose(2)-obj.true_Pose(2))^2) + obj.v_r*randn;
            angle = atan2(robot.true_Pose(2)-obj.true_Pose(2),robot.true_Pose(1)-obj.true_Pose(1)) - obj.true_Pose(3) + obj.v_r*randn;
            
            measure = [dist;angle];
        end

        function obj = update_robot_measurements(obj, robot_set, time_stamp)
            % the robot measurement table is reset with the new values of
            % the measurements 
            
            obj.z_r = table('Size', [0 4], 'VariableTypes', {'double','double','double','double'}, 'VariableNames', {'Name', 'Distance','Direction','Time'});

            for j=1:length(robot_set)
                measure = obj.measure_robot(robot_set(j));

                if robot_set(j).name == obj.name 
                    % don't add measurement against your own
                elseif measure(1) < obj.z_r_max 
                    % if the robot is reachable then save it 
                    newRow = table( robot_set(j).name, measure(1),measure(2), time_stamp, 'VariableNames', {'Name', 'Distance','Direction','Time'});
                    obj.z_r = [obj.z_r; newRow];
                end
      
            end
        end

    end
end