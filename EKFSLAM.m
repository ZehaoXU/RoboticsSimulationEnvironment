function scan_data = EKFSLAM(robot, sensor, sensor_rf, obstacle, lmks, this_scan_good, pose_this_scan)
% this_scan_good
% pose_this_scan
global World t;
%%%%%%%%%%%%%%%%%%%%%%%% TEMPORAL LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

World.t = t;
        
        %%%%%%%%%%%%%%%%%%%%%%%%% SIMULATE WORLD %%%%%%%%%%%%%%%%%%%%%%%%%%
        R_old = robot.R;      % true pose
        % if ~isempty(World.Wpts)
        %     robot.steer(World.Wpts);
        % end
        % for i = movingObstacles
        %     Obstacles(i).move(AxisDim);
        % end
       
        % robot.R = robot.move(zeros(2,1), 'true');        % Move with noise
        for lid = 1:size(World.W,2)
            v = sensor_rf.noise .* randn(2,1);
            World.y(:,lid) = scanPoint(robot.R, World.W(:,lid)) + v;
        end
        lmks_all = World.y;
        % % Determine landmarks that are within the sensor range
        [World.y, lmks_visible] = sensor_rf.constrainMeasurement(World.y);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%% START EKF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        %%%%%%%%%%%%%%%%%%%%%%% EKF PREDICTION STEP %%%%%%%%%%%%%%%%%%%%%%%
        
        % 1. POSE PREDICTION USING SCAN MATCHING %%%%%%%%%%%%%%%%%%%%%%%%%%
        % TODO: if scan_matching, then do this step. Scan match only if
        % ~isempty(Obstacles)
        enough_correlations = 0;
        
        % Current stored scan is now the last scan
        last_scan_good = this_scan_good;
        pose_last_scan = pose_this_scan;
        this_scan_good = 0; % Initialise this_scan_good for this iteration

        % Put the previous scan's data in scan_ref
        scan_ref = World.scan_data;
        % % Initialise scan data vector at maximum possible size it can take
        % World.scan_data = -ones(3, numSegments * (round(sensor.fov/sensor.angular_res) + 1));
        % scan_ndx = 1;
        % for i = 1:length(Obstacles)
        %     scan_data = Obstacles(i).getMeasured(sensor, robot);
        %     % Build scan_data vector
        %     if ~isempty(scan_data)
        %         World.scan_data(:, scan_ndx:scan_ndx+size(scan_data, 2)-1) = scan_data;
        %         scan_ndx = scan_ndx + size(scan_data, 2);
        %     end
        % end
        % % Reduce scan to only those appended in above for loop
        % World.scan_data = World.scan_data(:, World.scan_data(2, :) ~= -1);
        
        % % Reduce scan so that each laser angle only has one associated
        % % measurement
        % World.scan_data = removeDuplicateLasers(World.scan_data);
        % World.scan_data = World.scan_data(2:3, :); % Remove index row

        % First add Gaussian white noise to scan
        World.scan_data = sensor.scannedObstacle;
        v = repmat(sensor.noise, 1, size(World.scan_data, 2)) .* randn(2,size(World.scan_data, 2));
        if ~isempty(World.scan_data)
            World.scan_data = World.scan_data + v;
        end
        %%%%%%%%%%%%%%%%%閫欒鏄笉鏄鍔犲埌scan鐨勯瑁忛潰鍘伙�?%%%%%%%%%%%%%%%%%%%%%%%

        if ~isempty(World.scan_data)
            obstaclesDetected = 1;

            % Convert to Cartesian co-ordinates for scan matching
            % World.scan_data = getInvMeasurement(World.scan_data);

            % If the number of point scanned exceeds the specified
            % tolerance, it is suitable for scan matching
            if size(World.scan_data, 2) > World.scan_corr_tolerance
                this_scan_good = 1;
            end
            
            % Convert to global co-ordinates for graphical display
            World.scan_global =  World.scan_data;%transToGlobal(robot.R, World.scan_data);
        else
            obstaclesDetected = 0;
            World.scan_global = [];
        end
        % If both last and current scans are good for scan matching,
        % proceed to match scans:
        if last_scan_good && this_scan_good
            n = robot.q .* randn(2,1); % Noise in odometry measurement
            
            % Do ICP scan matching using odometry input as initial guess
            [R, T, correl, icp_var] = doICP(scan_ref, World.scan_data, 1, robot.u + n);
            % If there are enough points correlated between the two
            % scans:
            if length(correl) > World.scan_corr_tolerance
                enough_correlations = 1;
                % Determine estimated pose from scan match
                r_scan = zeros(3, 1);
                da = Normalize(asin(R(2,1)));
                r_scan(1:2) = transToGlobal(pose_last_scan, T);
                r_scan(3) = pose_last_scan(3) + da;
                
                % Transform scan data to global co-ordinates
                scan_data_corr = World.scan_data;
                scan_data_corr = transToGlobal(r_scan, scan_data_corr);
                % Grab data points that were successfully corellated
                scan_data_corr = scan_data_corr(:, correl(:, 2)');

                [r_odo, R_r, R_n] = robot.move(n);

                % Compute covariance matrix of scan match
                C = getICPCovariance(icp_var, scan_data_corr);
                J_u = [R zeros(2, 1); 0 0 1];
                cov_scan = J_u * C * J_u';
                cov_scan_norm = trace(cov_scan);
            end
        end
        
        % 2. POSE PREDICTION USING ODOMETRY MEASUREMENTS %%%%%%%%%%%%%%%%%%
        
        % If there isn't a useful scan, just use odometry data
        if ~enough_correlations
            n = robot.q .* randn(2,1);
            [r_odo, R_r, R_n] = robot.move(n);
            cov_scan = zeros(3, 3);
            dr_scan = zeros(3, 1);
        else
            dr_scan = r_scan - robot.r;
            dr_scan(3) = Normalize(dr_scan(3));
        end            

        % 3. WEIGHTAGE OF ODOMETRY AND SCAN MATCH PREDICTIONS %%%%%%%%%%%%%
        
        cov_odo = R_n * World.Q * R_n';
        cov_odo_norm = trace(cov_odo);
        
        dr_odo = r_odo - robot.r;
        dr_odo(3) = Normalize(dr_odo(3));
        
        dr_true = robot.R - R_old;

        % Check if the robot is making a large turn
        if abs(dr_scan(2)) > pi/6 || abs(dr_odo(2)) > pi/6
            big_turn = 1;
        elseif abs(dr_odo(2)) < pi/18
            big_turn = 0;
        end
        
        % Determine weights:
        % If not enough correlations, or turning is large, use odometry
        if ~enough_correlations % || big_turn
            weight_odo = 1;
            weight_scan = 0;
        else
            weight_odo = cov_scan_norm / (cov_scan_norm + cov_odo_norm);
            weight_scan = 1 - weight_odo;
            weight_scan = 1;
            weight_odo = 0;
        end
                
        % Determine robot pose using weights:
        r = World.r;
        robot.r = weight_odo * dr_odo + weight_scan * dr_scan + robot.r;
        World.x(r) = robot.r;

        % Covariance update
        % Caution: this method of update is suboptimal for CPU processing.
        % Alternative is to have a single equation that updates both
        % the pose and landmark covariances in a single step, but this
        % equation is rather complicated.
        P_rr = World.P(r,r);
        World.P(r,:) = R_r * World.P(r,:);
        World.P(:,r) = World.P(r,:)';
        World.P(r,r) = R_r * P_rr * R_r' + ...
            weight_scan * cov_scan + ...
            weight_odo * cov_odo;
        
        %%%%%%%%%%%%%%%%%%%%%% EKF UPDATE STEP %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 1. CORRECT KNOWN VISIBLE LANDMARKS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Find the visible landmarks. The below code circumvents the data
        % association problem:
        r_old = robot.r;
        
        lmks_visible_known = find(World.l(1,lmks_visible));
        lids = lmks_visible(lmks_visible_known);
        
        % 遍歷�?��看見的lmk
        for lid = lids    
            % Measurement prediction
            [e, E_r, E_l] = scanPoint(rob.r, World.x(World.l(:,lid)));

            E_rl = [E_r E_l];
            rl   = [r World.l(:,lid)'];
            E    = E_rl * World.P(rl,rl) * E_rl';

            % Actual Measurement 
            yi = World.y(:,lid);  % how fuck will I know the actual yi ???

            % Innovation
            z = yi - e;
            z(2) = getPiToPi(z(2));
            Z = World.M + E;

            % Kalman gain
            K = World.P(:, rl) * E_rl' * Z^-1;

            % Update state and covariance
            World.x = World.x + K * z;
            World.P = World.P - K * Z * K'; % The complexity of this line is 
                                            % very high (subtraction of two
                                            % large matrices)
            rob.r = World.x(r);
        end

        % 2. INITIALISE NEW LANDMARKS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Check landmark availabiliy.
        % Again data association is avoided. A new landmark is initialised
        % in the storage space reserved specifically for that landmark:

        % lmks_visible_unknown = find(World.l(1,lmks_visible)==0);
        % lids = lmks_visible(lmks_visible_unknown);
        
        % if ~isempty(lids)
        %     for lid = lids
        %         s = find(World.mapspace, 2);
        %         if ~isempty(s)
        %             World.mapspace(s) = 0;
        %             World.l(:,lid) = s';
        %             % Measurement
        %             yi = World.y(:,lid);

        %             [World.x(World.l(:,lid)), L_r, L_y] = invScanPoint(robot.r, yi);
        %             World.P(s,:) = L_r * World.P(r,:);
        %             World.P(:,s) = World.P(s,:)'; % Cross variance of lmk with all other lmks
        %             World.P(s,s) = L_r * World.P(r,r) * L_r' + L_y * World.M * L_y';
        %         end
        %     end
        % end
        pose_this_scan = robot.r; % The corrected pose at which the scan was taken
            
        %%%%%%%%%%%%%%%%%%%%%%%%%% MAPPING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Determine pose correction during update step
        update = robot.r - r_old;
        scan_data = World.scan_data;

%         %%%%%%%%%%%%%%%%%%%%%%% HISTORICAL DATA COLECTION %%%%%%%%%%%%%%%%%
%         World.R_hist(:, t) = robot.R(1:2);	% True position history
%         World.r_hist(:, t) = robot.r(1:2);   % Estimated position history
%         pose_error2 = (robot.R(1:2) - robot.r(1:2)).^2;
%         World.error_hist(t) = sqrt(pose_error2(1) + pose_error2(2));
%         World.Pr_hist(:, t) = [World.P(r(1),r(1)); World.P(r(2),r(2))];
%         
%         if enough_correlations
%             scan_error = abs(dr_scan-dr_true); scan_error(3) = abs(Normalize(scan_error(3)));
%             scan_error(1) = pdist([0 0; scan_error(1) scan_error(2)]); scan_error(2) = [];
%         else
%             scan_error = [0; 0];
%         end
%         
%         odo_error = abs(dr_odo-dr_true); odo_error(3) = abs(Normalize(odo_error(3)));
%         odo_error(1) = pdist([0 0; odo_error(1) odo_error(2)]); odo_error(2) = [];
%         
%         World.scan_error_hist(:, t) = scan_error;
%         World.odo_error_hist(:, t) = odo_error;
%         World.turning_hist(t) = robot.u(2);
%         World.weight_scan_hist(t) = weight_scan;
%         World.weight_odo_hist(t) = weight_odo;



end