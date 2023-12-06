classdef Robot
    properties
        true_Pose;
        true_Path;
    end

    methods
        function obj = Robot(x0)
            obj.true_Pose = double(x0);
            obj.true_Path = [obj.true_Pose'];
        end

        function obj = dynamics(obj,dt,v,w)
            phi = obj.true_Pose(3);

            vel = [ v*cos(phi);
                    v*sin(phi);
                    w           ]; 
            
            obj.true_Pose = obj.true_Pose + vel*dt;
            obj.true_Path = [obj.true_Path; obj.true_Pose'];
        end

        function obj = update(obj,dt)
            v = 0.5; % m/s
            w = -pi/4 + pi*2/4*rand; % random var in [-pi/4, pi/4]
            
            obj = obj.dynamics(dt,v,w);
        end

        function triangle = get_triangle(obj)
            x = obj.true_Pose(1);
            y = obj.true_Pose(2);
            th = obj.true_Pose(3);

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