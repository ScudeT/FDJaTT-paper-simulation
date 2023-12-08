classdef Robot

    % the model of a robot that can move randomly in 2d space
    % it saves its current position and the full path it has been through

    properties
        true_Pose;  % [x;y;th]
        true_Path;  % list of poses
        u;
        dt;
    end

    methods
        function obj = Robot(x0)
            obj.true_Pose = double(x0);
            obj.true_Path = obj.true_Pose';
            obj.u = [0;0];
            obj.dt = 0;
        end

        function obj = dynamics(obj,dt,v,w)
            % the dynamics rule the movement of the robot 
            phi = obj.true_Pose(3);

            vel = [ v*cos(phi);
                    v*sin(phi);
                    w           ]; 
            obj.dt = dt;
            obj.true_Pose = obj.true_Pose + vel*dt;
            obj.true_Path = [obj.true_Path; obj.true_Pose'];
        end

        function obj = update(obj,dt)
            % gives some random velocities everytime making the robot move 
            % randomly in 2D space

            v = 0.5; % m/s
            w = -pi/4 + pi*2/4*rand; % random var in [-pi/4, pi/4]
            obj.u = [v;w];
            obj = obj.dynamics(dt,v,w);
        end

        function triangle = draw(obj,i)
            % a draw funciton has been added to allow to draw a triangle 
            % in the position and direction from a moment in time from 
            % the path

            x = obj.true_Path(i,1);
            y = obj.true_Path(i,2);
            th = obj.true_Path(i,3);

            verticesX = [0 1 0]*0.5;
            verticesY = [-0.5 0 0.5]*0.5;

            R = [cos(th) -sin(th); sin(th) cos(th)];
            rotatedVertices = R * [verticesX; verticesY];

            translatedVerticesX = rotatedVertices(1,:) + x;
            translatedVerticesY = rotatedVertices(2,:) + y;

            triangle = [translatedVerticesX; translatedVerticesY];
        end
    end
end