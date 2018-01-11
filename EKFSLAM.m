function scan_data = EKFSLAM(robot, sensor, sensor_rf, obstacle, lmks, this_scan_good, pose_this_scan)
global World t;
%%%%%%%%%%%%%%%%%%%%%%%% SLAM loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

World.t = t;
        
R_old = robot.R;      % true pose
for lid = 1:size(World.W,2)
    v = sensor_rf.noise .* randn(2,1);
    World.y(:,lid) = scanPoint(robot.R, World.W(:,lid)) + v;
end
lmks_all = World.y;
% % Determine landmarks that are within the sensor range
[World.y, lmks_visible] = sensor_rf.constrainMeasurement(World.y);

%%%%%%%%%%%%%%%%%%%%%%%%%%% START EKF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%%%%%%%%%%%%%%% EKF PREDICTION STEP %%%%%%%%%%%%%%%%%%%%%%%

enough_correlations = 0;

% Current stored scan is now the last scan
last_scan_good = this_scan_good;
pose_last_scan = pose_this_scan;
this_scan_good = 0; % Initialise this_scan_good for this iteration

scan_ref = World.scan_data;

% First add Gaussian white noise to scan
World.scan_data = sensor.scannedObstacle;
v = repmat(sensor.noise, 1, size(World.scan_data, 2)) .* randn(2,size(World.scan_data, 2));
if ~isempty(World.scan_data)
    World.scan_data = World.scan_data + v;
end
%%%%%%%%%%%%%%%%%procrss and get the R t of 2 scans%%%%%%%%%%%%%%%%%%%

if ~isempty(World.scan_data)
    obstaclesDetected = 1;
    if size(World.scan_data, 2) > World.scan_corr_tolerance
        this_scan_good = 1;
    end     
    World.scan_global =  World.scan_data; % they have the same type of coordinate.
    % if not, change it.
else
    obstaclesDetected = 0;
    World.scan_global = [];
end
if last_scan_good && this_scan_good
    n = robot.q .* randn(2,1); % Noise in odometry measurement         
    % Do ICP scan matching using odometry input as initial guess
    [R, T, correl, icp_var] = doICP(scan_ref, World.scan_data, 1, robot.u + n);
    % If the matching function well
    if length(correl) > World.scan_corr_tolerance
        enough_correlations = 1;
        r_scan = zeros(3, 1);
        da = Normalize(asin(R(2,1)));
        r_scan(1:2) = transToGlobal(pose_last_scan, T);
        r_scan(3) = pose_last_scan(3) + da;
        
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
P_rr = World.P(r,r);
World.P(r,:) = R_r * World.P(r,:);
World.P(:,r) = World.P(r,:)';
World.P(r,r) = R_r * P_rr * R_r' + ...
    weight_scan * cov_scan + ...
    weight_odo * cov_odo;

%%%%%%%%%%%%%%%%%%%%%% EKF UPDATE STEP %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%1. CORRECT KNOWN VISIBLE LANDMARKS %%%%%%%%%%%%%%%%%%%
r_old = robot.R;

lmks_visible_known = find(World.l(1,lmks_visible));
lids = lmks_visible(lmks_visible_known);
    
for lid = lids    
    % Measurement prediction
    [e, H_rob, H_lmk] = scanPoint(rob.r, World.x(World.l(:,lid)));

    H = [H_rob H_lmk]; 
    rl   = [r World.l(:,lid)']; 
    Hnew    = H * World.P(rl,rl) * H'; % Hnew is H i+1

    yi = World.y(:,lid);  % how fuck will I know the actual yi

    z = yi - e;
    z(2) = Normalize(z(2));
    S =  Hnew + World.M; % S is S. world.M is sen_rf.noise .^ 2 %covariance matrix%  Hnew: obeservation 

    % Kalman gain
    K = World.P(:, rl) * H' * S^-1; % ???? H is H i, S is S

    % Update state and covariance
    World.x = World.x + K * z;  % z is v , = zi - e
    World.P = World.P - K * S * K'; %  P is P,  S is S, K is W 
    rob.r = World.x(r);
end

pose_this_scan = robot.Pos;
%%%%%%%%%%%%%%%%%%%%%%%%%% MAPPING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Determine pose correction during update step
update = robot.R - r_old;
scan_data = World.scan_data;
            
World = doMapping(World, sensor, robot.R, scan_data);
end