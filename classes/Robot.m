classdef Robot

    % the model of a robot that can move randomly in 2d space
    % it saves its current position and the full path it has been through

    properties
        true_Pose;  % [x;y;th]
        true_Path;  % list of poses
        Pose_est;
        p_est;
        process_cov;
    end

    methods
        function obj = Robot(x0,stat_data)
            obj.true_Pose = double(x0);
            obj.true_Pose(3) = wrapTo2Pi(obj.true_Pose(3));
            obj.true_Path = obj.true_Pose';

            e_var0 = stat_data{1};
            w_var = stat_data{2};

            obj.Pose_est = x0;
            obj.Pose_est(3) = wrapTo2Pi(obj.Pose_est(3));

            obj.p_est = eye(3)*e_var0;

            obj.process_cov = eye(2)*w_var;
        end

        function obj = dynamics(obj,dt,v,w)
            % the dynamics rule the movement of the robot 
            phi = obj.true_Pose(3);

            vel = [ v*cos(phi);
                    v*sin(phi);
                    w           ]; 

            obj.true_Pose = obj.true_Pose + vel*dt;
            obj.true_Pose(3) = wrapTo2Pi(obj.true_Pose(3)); % always keep heading form -pi to pi
            obj.true_Path = [obj.true_Path; obj.true_Pose'];
        end
        
        function x_next = dynamics_est(obj,pose_est,u,dt)
                w = [   randn*obj.process_cov(1,1);
                        randn*obj.process_cov(2,2)];

                x_next = pose_est + [   (u(1)-w(1))*cos(pose_est(3)); 
                                        (u(1)-w(1))*sin(pose_est(3)); 
                                            u(2)-w(2)]*dt;
        end

        function obj = propagate(obj,dt,v,w)
            pose_est = obj.Pose_est;
            u = [v;w];

            Phi = [ 1   0  -u(1)*dt*sin(pose_est(3));
                    0   1   u(1)*dt*cos(pose_est(3));
                    0   0   1];
            G = [   cos(pose_est(3))    0;
                    sin(pose_est(3))    0;
                    0                   1];

            Q = G*obj.process_cov*G';

            obj.p_est = Phi*obj.p_est*Phi' + Q;
            obj.Pose_est = obj.dynamics_est(pose_est,u,dt);
            obj.Pose_est(3) = wrapTo2Pi(obj.Pose_est(3));
        end

        function obj = update(obj,dt)
            % gives some random velocities everytime making the robot move 
            % randomly in 2D space

            v = 0.5; % m/s
            w = -pi/4 + pi*2/4*rand; % random var in [-pi/4, pi/4]

            obj = obj.dynamics(dt,v,w);
            obj = obj.propagate(dt,v,w);
        end

        function triangle = draw(obj,i)
            % a draw funciton has been added to allow to draw a triangle 
            % in the position and direction from a moment in time from 
            % the path

            x = obj.true_Path(i,1);
            y = obj.true_Path(i,2);
            th = obj.true_Path(i,3);

            verticesX = [0 1 0]*0.1;
            verticesY = [-0.5 0 0.5]*0.1;

            R = [cos(th) -sin(th); sin(th) cos(th)];
            rotatedVertices = R * [verticesX; verticesY];

            translatedVerticesX = rotatedVertices(1,:) + x;
            translatedVerticesY = rotatedVertices(2,:) + y;

            triangle = [translatedVerticesX; translatedVerticesY];
        end

        function draw_estimate(obj)
            % Given pose and covariance matrix
            pose = obj.Pose_est; % Replace with your actual pose
            P = obj.p_est; % Your 2x2 covariance matrix for x and y
            
            % Plot the position
            plot(pose(1), pose(2), 'bo'); % 'bo' plots a blue circle at the position
            
            % Plot the uncertainty ellipse for x and y
            % Extract the 2x2 covariance matrix for x and y
            P_position = P(1:2, 1:2);
            % Calculate the eigenvalues and eigenvectors of the covariance matrix
            [eigvec, eigval] = eig(P_position);
            
            % Get the largest eigenvalue and its corresponding eigenvector
            [largest_eigenval, largest_eigenvec_ind_c] = max(diag(eigval));
            largest_eigenvec = eigvec(:, largest_eigenvec_ind_c);
            
            % Get the smallest eigenvalue
            if largest_eigenvec_ind_c == 1
                smallest_eigenval = eigval(2,2);
            else
                smallest_eigenval = eigval(1,1);
            end
            
            % Calculate the angle between the x-axis and the largest eigenvector
            angle = atan2(largest_eigenvec(2), largest_eigenvec(1));
            
            % Get the 95% confidence interval error ellipse
            chisquare_val = 5.991; % 95% confidence interval for 2 degrees of freedom
            theta_grid = linspace(0, 2*pi, 100);
            phi = angle;
            X0 = pose(1);
            Y0 = pose(2);
            a = sqrt(chisquare_val*largest_eigenval);
            b = sqrt(chisquare_val*smallest_eigenval);
            
            % the ellipse in x and y coordinates
            ellipse_x_r = a*cos(theta_grid);
            ellipse_y_r = b*sin(theta_grid);
            
            % Define a rotation matrix
            R = [cos(phi) sin(phi); -sin(phi) cos(phi)];
            
            % Rotate the ellipse
            r_ellipse = [ellipse_x_r; ellipse_y_r]' * R;
            
            % Draw the error ellipse
            plot(r_ellipse(:,1) + X0, r_ellipse(:,2) + Y0, 'r');
            
            % Plot the heading
            head_length = 0.5; % Length of the heading line, adjust as needed
            heading_vector = [cos(pose(3)), sin(pose(3))] * head_length;
            quiver(pose(1), pose(2), heading_vector(1), heading_vector(2), 0, 'k', 'LineWidth', 2);
            
            
        end
    
    
    end
end