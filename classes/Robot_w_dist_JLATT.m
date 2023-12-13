classdef Robot_w_dist_JLATT < Robot_w_sens_and_comm
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        R_mes
        debug
        save_est
        save_cov
        
    end

    methods
        function obj = Robot_w_dist_JLATT(x0,name, stat_data, sensor_data, connection_data,debug)
            obj@Robot_w_sens_and_comm(x0,name,stat_data,sensor_data, connection_data)
            
            obj.R_mes = eye(2)*obj.v_r;
            obj.debug = debug;

            obj.save_est = [];
            obj.save_cov = [];
            
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

        function obj = corrections_from_com(obj,t)
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
                % distance correction
                distance_diff = z_il(1) - z_li(1);
                % angle corrections
                
                phi_il = wrapTo2Pi(z_il(2)) + wrapTo2Pi(Pose_l(3));
                phi_li = wrapTo2Pi(z_li(2)) + wrapTo2Pi(Pose_i(3));
                
                dphi_il = phi_il- phi_li + pi;
                dphi_il = wrapTo2Pi(dphi_il);
                if dphi_il > 2*pi-0.001
                    dphi_il = 0;
                end

                dz_il = [distance_diff; dphi_il]; 
                dz_il(dz_il<0.001) = 0;
                
                %% H matrices
                dx = Pose_l(1) - Pose_i(1);
                dy = Pose_l(2) - Pose_i(2);

                H_i = [ -dx/sqrt(dx^2 + dy^2)   -dy/sqrt(dx^2 + dy^2)   0;
                        -dy/(dx^2 + dy^2)       dx/(dx^2+dy^2)          -1];
                
                

                H_l = [ dx/sqrt(dx^2 + dy^2)    dy/sqrt(dx^2 + dy^2)    0;
                        dy/(dx^2 + dy^2)        -dx/(dx^2+dy^2)         0];
                
                
                R_hat = obj.R_mes + H_l*p_l*H_l';
                R_hat(R_hat < obj.v_r) = 0;
                R_hat = R_hat + 0.01 * eye(size(R_hat));

                %% corrections
                s_l = H_i'*inv(R_hat)*H_i;
                z_corr = inv(R_hat)*(dz_il + H_i*Pose_i);
                z_corr(2) = wrapToPi(z_corr(2));
                y_l = H_i'*z_corr;

                %% save
                s_set(end+1,:) = {s_l};
                y_set(end+1,:) = {y_l};

            end
            
            if isempty(s_set)
                % dont update if I dont have data
            else
                %% Compute update terms
                nu = 1/n_corrections; % I am only updating from relative measurements
                
                inv_p_hat = 0;
                for i=1:height(s_set)
                    inv_p_hat = inv_p_hat + nu*s_set{i};
                end
                inv_p_hat(inv_p_hat < 0.1) = 0;
                inv_p_hat = inv_p_hat + 1e-3 * eye(size(inv_p_hat));

                p_hat = inv(inv_p_hat);
                
                

                x_hat = 0;
                for i=1:height(y_set)
                    x_hat = x_hat + nu*inv_p_hat*y_set{i};
                end
                x_hat(3) = wrapToPi(x_hat(3));
                
                p_est = obj.p_est;
                p_est(p_est < 0.1) = 0;
                p_est = p_est + 1e-3 * eye(size(p_est));
    
                Omega = inv(p_est);
                q = inv(p_est)*obj.Pose_est;
                
                % update coeff
                hat_contr = trace(p_hat)^-1;
                est_contr = trace(p_est)^-1;
                alpha = hat_contr/(hat_contr + est_contr+1e-5); % value from (0,1) to minimize the resulting Trace(p_est) 
                
                temp = alpha*inv_p_hat + (1-alpha)*Omega;
                temp = temp + 1e-3 * eye(size(temp));

                Gamma = inv_p_hat*inv(temp)*Omega;

                K = Omega - alpha*Gamma;

                L = inv_p_hat - (1-alpha)*Gamma;

                new_p_est = inv(Omega + inv_p_hat - Gamma);
                new_Pose = new_p_est*(K*inv(Omega)*q + L*x_hat);
                new_Pose(3) = wrapTo2Pi(new_Pose(3));
                
                %% Update
                if obj.debug
                    save = [obj.Pose_est, new_Pose, obj.true_Pose];
                    obj.save_est = [obj.save_est; save];
                    save = [obj.p_est, new_p_est];
                    obj.save_cov = [ obj.save_cov; save];

                    e_before = abs(obj.Pose_est - obj.true_Pose);
                    e_after = abs(new_Pose - obj.true_Pose);
    
                    if e_before < e_after
                        fprintf(" - This is making things worse for robot %d at time %f\n", obj.name, t)
                    else 
                        fprintf(" + Update useful for robot %d at time %f\n", obj.name, t)
                    end
                    if e_after(3) > pi/2
                        fprintf(" - Robot %d is definatly not going in the right direction at time %f\n", obj.name, t)
                    elseif e_before(3) < e_after(3)
                        fprintf(" - the update deviating robot %d at time %f\n", obj.name, t)
                    end
                end
                

                if sum(abs(new_Pose-obj.Pose_est)) > 10
                    fprintf(" ? Ignored update of robot %d at time %f\n", obj.name, t)
                else
                    obj.p_est = new_p_est;          
                    obj.Pose_est = new_Pose; 
                end

            end
            

        end

        
        function obj = simple_corrections(obj,t)
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
                
                z_il = [data_row.dX;  data_row.dY]; % data received from l
                z_li = [zr_row.dX;    zr_row.dY];  % measurement I have of l

                Pose_l = data_row.Pose_estimate{1};
                p_l = data_row.Error_covariance{1};
                Pose_i = obj.Pose_est;
                p_i = obj.p_est;
                

                %% measurement error

                dz_il = abs(z_il) - abs(z_li);

                
                %% H matrices

                H_i = [ -1  0   0;
                        0   -1  0];

                H_l = [ 1   0   0;
                        0   1   0];

                R_hat = obj.R_mes + H_l*p_l*H_l';
                R_hat(R_hat < obj.v_r) = 0;
                R_hat = R_hat + 0.01 * eye(size(R_hat));

                %% corrections
                s_l = H_i'*inv(R_hat)*H_i;
                z_corr = inv(R_hat)*(dz_il + H_i*Pose_i);
                z_corr(2) = wrapToPi(z_corr(2));
                y_l = H_i'*z_corr;

                %% save
                s_set(end+1,:) = {s_l};
                y_set(end+1,:) = {y_l};

            end
            
            if isempty(s_set)
                % dont update if I dont have data
            else
                %% Compute update terms
                nu = 1/n_corrections; % I am only updating from relative measurements
                
                inv_p_hat = 0;
                for i=1:height(s_set)
                    inv_p_hat = inv_p_hat + nu*s_set{i};
                end
                inv_p_hat(inv_p_hat < 0.1) = 0;
                inv_p_hat = inv_p_hat + 1e-3 * eye(size(inv_p_hat));

                p_hat = inv(inv_p_hat);
                
                

                x_hat = 0;
                for i=1:height(y_set)
                    x_hat = x_hat + nu*inv_p_hat*y_set{i};
                end
                x_hat(3) = wrapToPi(x_hat(3));
                
                p_est = obj.p_est;
                p_est(p_est < 0.1) = 0;
                p_est = p_est + 1e-3 * eye(size(p_est));
    
                Omega = inv(p_est);
                q = inv(p_est)*obj.Pose_est;
                
                % update coeff
                hat_contr = trace(p_hat)^-1;
                est_contr = trace(p_est)^-1;
                alpha = hat_contr/(hat_contr + est_contr+1e-5); % value from (0,1) to minimize the resulting Trace(p_est) 
                
                temp = alpha*inv_p_hat + (1-alpha)*Omega;
                temp = temp + 1e-3 * eye(size(temp));

                Gamma = inv_p_hat*inv(temp)*Omega;

                K = Omega - alpha*Gamma;

                L = inv_p_hat - (1-alpha)*Gamma;

                new_p_est = inv(Omega + inv_p_hat - Gamma);
                new_Pose = new_p_est*(K*inv(Omega)*q + L*x_hat);
                new_Pose(3) = wrapTo2Pi(new_Pose(3));
                
                %% Update
                if obj.debug
                    save = [obj.Pose_est, new_Pose, obj.true_Pose];
                    obj.save_est = [obj.save_est; save];
                    save = [obj.p_est, new_p_est];
                    obj.save_cov = [ obj.save_cov; save];

                    e_before = abs(obj.Pose_est - obj.true_Pose);
                    e_after = abs(new_Pose - obj.true_Pose);
    
                    if e_before < e_after
                        fprintf(" - This is making things worse for robot %d at time %f\n", obj.name, t)
                    else 
                        fprintf(" + Update useful for robot %d at time %f\n", obj.name, t)
                    end
                    if e_after(3) > pi/2
                        fprintf(" - Robot %d is definatly not going in the right direction at time %f\n", obj.name, t)
                    elseif e_before(3) < e_after(3)
                        fprintf(" - the update deviating robot %d at time %f\n", obj.name, t)
                    end
                end
                

                if sum(abs(new_Pose-obj.Pose_est)) > 10
                    fprintf(" ? Ignored update of robot %d at time %f\n", obj.name, t)
                else
                    obj.p_est = new_p_est;          
                    obj.Pose_est = new_Pose; 
                end

            end
            

        end

    end
end