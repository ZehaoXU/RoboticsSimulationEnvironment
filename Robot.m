classdef Robot < handle
    properties
        % Robot properties
        Radius          % Length of robot
        Pos               % True pose [x y a]
        estPos               % Estimated pose [x y a]
        dir             % radian direction
        q               % True system noise
        u               % Control vector [dx da]
        min_turn_rad    % Turning radius (m)
        
        wheelbase       % Width of wheel base (dist. between opposite wheels)
        length          % Length of robot
        R               % True pose [x y a]
        r               % Estimated pose [x y a]
        maxspeed        % Maximum speed (m/s)
        curr_wpt        % Current waypoint reached
        
        
        %curr_wpt        % Current waypoint reached
    end
    
    methods        
        function rob = Robot                        % Constructor method
            % Default values
            rob.Radius      =   0.1;
            rob.Pos         =   [0, 0];
            rob.estPos      =   rob.Pos;
            rob.dir         =   0;
            rob.q           =   [0.02 ; pi/180];       % 2cm, 1 deg
            rob.u           =   [0.1; 0.02];
            rob.min_turn_rad=   0.5;
            rob.wheelbase	=   1;
            rob.length      =   1.5;
            rob.R           =   [0, 0, 0]';
            rob.r           =   rob.R;
            rob.q           =   [0.02;pi/180];       % 2cm, 1 deg
            rob.u           =   [0.1; 0.02];
            rob.maxspeed    =   1.4;                % 1.4m/s ~ 5km/h
            rob.min_turn_rad=   0.5;
            rob.curr_wpt    =   1;
%            rob.curr_wpt    =   1;
        end

        function [r_new, RNEW_r, RNEW_u] = move(rob, n, strRobType)
            
            if nargin < 3
                % Robot position (p = [x y]') and orientation (a):
                pose = rob.r; a = rob.r(3);
            elseif strcmp(strRobType, 'true')
                pose = rob.R; a = rob.R(3);
            else
                error('strRobType can only take the value "true"');
            end

            % Control input:
            dx = rob.u(1) + n(1);
            da = rob.u(2) + n(2);
            dp = [dx; 0];

            % Determine new orientation
            a_new = a + da;    
            % Ensure that angle is between -pi and pi
            a_new = Normalize(a_new);

            if nargout == 1
                % Translate robot to new position in global frame
                p_new = transToGlobal(pose, dp);

            else    % Get Jacobians

                % Translate robot to new position in global frame
                [p_new, PNEW_r, PNEW_dp] = transToGlobal(pose, dp);

                % Jacobians of r_a_new:
                ANEW_a = 1;
                ANEW_da = 1;

                % Hence, Jacobians of r_new:
                RNEW_r = [PNEW_r; 0 0 ANEW_a];

                RNEW_u = [PNEW_dp(:, 1) zeros(2, 1); 0 ANEW_da];

            end

            r_new = [p_new; a_new];
            
        end

        function steer(rob, Wpts)
            % Determine if current waypoint reached
            wpt = Wpts(:, rob.curr_wpt);
            % Distance from current waypoint
            dist = sqrt((wpt(1)-rob.R(1))^2 + (wpt(2)-rob.R(2))^2);
            if dist < rob.min_turn_rad
                if rob.curr_wpt < size(Wpts, 2)
                    rob.curr_wpt = rob.curr_wpt + 1;
                else
                    rob.curr_wpt = 1; % Go back to the start
                end
                wpt = Wpts(:, rob.curr_wpt);
            end
            % Change in robot orientation to face current waypoint
            da = Normalize(atan2(wpt(2)-rob.R(2), wpt(1)-rob.R(1))-rob.R(3));
            % Amount by which to perturb current robot control:
            dda = da - rob.u(2);
            du = [0; dda];
            rob.setControl(du);
        end

        % Perturb control vector by an amount du = [accel_x; accel_a].
        function u = setControl(rob, du)
            u = rob.u + du;
            % Make sure that angular control is between -pi and pi
            u(2) = Normalize(u(2));
            if abs(u(1)) > rob.maxspeed
                % Adjust speed so that max speed isn't exceeded
                u(1) = sign(u(1)) * rob.maxspeed;
            end
            turning_radius = u(1)/u(2);     % Using formula v = omega * r
            if abs(turning_radius) < rob.min_turn_rad
                % Adjust angular velocity so that min turning radius isn't
                % exceeded
            	u(2) = sign(u(2)) * abs(u(1)) / rob.min_turn_rad;
            end
            
            rob.u = u;
        end
    end
end