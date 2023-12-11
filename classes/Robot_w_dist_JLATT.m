classdef Robot_w_dist_JLATT < Robot_w_sens_and_comm
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Pose_est
        p_est
        process_cov
        dhil_Pose_i
        dhil_Pose_l
        R_mes
    end

    methods
        function obj = Robot_w_dist_JLATT(x0,name,sensor_data, connection_data, stat_data)
            obj@Robot_w_sens_and_comm(x0,name,sensor_data, connection_data)
            
            e_var0 = stat_data{1};
            w_var = stat_data{2};

            obj.Pose_est = x0;
            obj.p_est = eye(3)*e_var0;

            obj.process_cov = eye(2)*w_var;
            obj.R_mes = eye(2)*obj.v_r;
            
            syms xi yi thi;
            syms xl yl thl;
            
            hil = [  sqrt((xl-xi)^2+(yl-yi)^2);
                     atan2(yl-yi, xl- xi)-thi];
            
            dhil_xi = [ diff(hil,xi), diff(hil,yi), diff(hil,thi)];
            dhil_xi = vpa(dhil_xi); % numeric approximation to ease calcolus
            dhil_xl = [ diff(hil,xl), diff(hil,yl), diff(hil,thl)];
            dhil_xl = vpa(dhil_xl); % numeric approximation to ease calcolus

            dhil_Pose_i = matlabFunction(dhil_xi);
            dhil_Pose_l = matlabFunction(dhil_xl);

            obj.dhil_Pose_i = @(Pose_i,Pose_l) dhil_Pose_i(Pose_i(1),Pose_l(1),Pose_i(2),Pose_l(2));
            obj.dhil_Pose_l = @(Pose_i,Pose_l) dhil_Pose_l(Pose_i(1),Pose_l(1),Pose_i(2),Pose_l(2));
            
        end

        function [x_ci, p_ci] = track_to_track_fusion(x_set, p_set)
            % let x be the estimate and p the estimated covariance set the
            % track to track algorithm combines different estimates of the
            % same variable toghether
            
            n = length(x_set);

            alpha_weights = zeros(1,n);
            for i = 1:n
                alpha_weights(i) = 1/trace(p_set{i});
            end
            alpha_weights = alpha_weights / sum(alpha_weights);

            p_ci_inv = zeros(size(p_set{1}));
            for i = 1:n
                p_ci_inv = p_ci_inv + alpha_weights(i)/p_set{i};
            end
            p_ci = inv(p_ci_inv);
            
            % Step 4: Calculate combined state estimate
            x_ci = zeros(size(x_set{1}));
            for i = 1:n
                x_ci = x_ci + alpha_weights(i) * (p_set{i} \ x_set{i});
            end
        end
        
        function x_next = dynamics_est(obj,pose_est,u,dt)
                w = [   randn*obj.process_cov(1,1);
                        randn*obj.process_cov(2,2)];

                x_next = pose_est + [   (u(1)-w(1))*cos(pose_est(3)); 
                                        (u(1)-w(1))*sin(pose_est(3)); 
                                            u(2)-w(2)]*dt;
        end

        function obj = propagate(obj)
            pose_est = obj.Pose_est;
            u = obj.u;
            dt = obj.dt;

            Phi = [ 1   0  -u(1)*dt*sin(pose_est(3));
                    0   1   u(1)*dt*cos(pose_est(3));
                    0   0   1];
            G = [   cos(pose_est(3))    0;
                    sin(pose_est(3))    0;
                    0                   1];

            Q = G*obj.process_cov*G';

            obj.p_est = Phi*obj.p_est*Phi' + Q;
            obj.Pose_est = obj.dynamics_est(pose_est,u,dt);
        end
        
        function obj = update_rel_com(obj,robot_set, time_stamp)
            
            obj.data_rel = table('Size', [0 6], 'VariableTypes', {'double','double','double','double','double','double'}, 'VariableNames', {'Name', 'Distance','Direction','Pose_estimate', 'Error_covariance','Time'});
            
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
            
                            rel_data = {robot.name, my_distance, my_direction, robot.Pose_est, robot.p_est, time_stamp };
                            
                            obj.data_rel = [obj.data_rel; rel_data];
            
                        end
                    end
                end
      
            end
        end
        
        function obj = update_com(obj,robot_set,time_stamp)

            obj = obj.update_rel_com(robot_set,time_stamp);
            
        end

        function obj = corrections_from_com(obj)
            n_corrections = height(obj.data_rel);
            s_set = {};
            y_set = {};

            %% Calculate corrections
            for l = 1:height(obj.data_rel)
                %% given robot_l's data
                data_row = obj.data_rel(l,:);
                sender_name = data_row.Name;
                sender_zr_row = ismember(obj.z_r.Name, sender_name);
                zr_row = obj.z_r(sender_zr_row,:);
                
                z_il = [data_row.Distance;  data_row.Direction]; % data received from l
                z_li = [zr_row.Distance;    zr_row.Direction];  % measurement I have of l

                Pose_l = data_row.Pose_estimate{1};
                p_l = data_row.Error_covariance{1};
                Pose_i = obj.Pose_est;
                p_i = obj.p_est;
                
                %% measurement error 
                dz_il = z_il - z_li; 
                dz_il(2) = mod(dz_il(2), 2*pi); % difference in measurements
                
                %% H matrices
                H_i = obj.dhil_Pose_i(Pose_i,Pose_l);
                H_l = obj.dhil_Pose_l(Pose_i,Pose_l);

                R_hat = obj.R_mes + H_l*p_l*H_l';

                %% corrections
                s_l = real(H_i'/R_hat*H_i);
                y_l = real(H_i'/R_hat*(dz_il + H_i*Pose_i));

                %% save
                s_set(end+1,:) = {H_i'/R_hat*H_i};
                y_set(end+1,:) = {H_i'/R_hat*(dz_il + H_i*Pose_i)};

            end
            
            %% Compute update terms
            nu = 1/n_corrections; % I am only updating from relative measurements
            
            inv_p_hat = 0;
            for i=1:height(s_set)
                inv_p_hat = inv_p_hat + nu*s_set{i};
            end
            
            x_hat = 0;
            for i=1:height(y_set)
                x_hat = x_hat + nu*inv(inv_p_hat)*y_set{i};
            end
           

            Omega = inv(obj.p_est);
            q = inv(obj.p_est)*obj.Pose_est;
            
            alpha = 0.3; % value from (0,1) to minimize the resulting Trace(p_est) 
            
            temp = alpha*inv_p_hat + (1-alpha)*Omega;
            Gamma = inv_p_hat*inv(temp)*Omega;
            K = Omega - alpha*Gamma;
            L = inv_p_hat - (1-alpha)*Gamma;
            
            %% Update
            obj.p_est = real(inv(Omega + inv_p_hat - Gamma));
            obj.Pose_est = real(obj.p_est*(K*inv(Omega)*q + L*x_hat));

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