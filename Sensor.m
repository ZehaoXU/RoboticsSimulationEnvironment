classdef Sensor < handle
    properties
        % Sensor properties
        angleSensorMax           % Range of laser sensor (metres)
        dSensorMax               % max distance of sensor
        dValleyMax               % min range of valley in VFH
        b                                   %b is   
        a                               
        C  
        resolutionDeg       % Minimum distance of sensor reading
        fov             % Angle over which sensor operates (field of view)
        noise           % Measurement noise (range and bearing)
        angular_res     % Angular resolution (separation between readings)
        hist
        
        range           % Range of laser sensor (metres)
        range_min       % Minimum distance of sensor reading
        scannedObstacle
    end
    
    methods
        
        function sen = Sensor                       % Constructor method
            % Default values
            sen.angleSensorMax = deg2rad(90);
            sen.dSensorMax = 1.5;
            sen.dValleyMax = 15;
            sen.b = 2;                                  %b is   
            sen.a = sen.b * sen.dSensorMax;                               
            sen.C = 5;  
            sen.resolutionDeg = 5;
            hist = 0;
            scannedObstacle = 0;
            
            sen.fov         =   pi;
            sen.noise       =   [0.03; 1*pi/180];   % 3 cm, 1 deg
            sen.angular_res =   1 * pi/180;         % 1 degree
            sen.range       =   5.6;
            sen.range_min   =   0.2;
            sen.fov         =   pi;
            sen.noise       =   [0.03; 1*pi/180];   % 3 cm, 1 deg
            sen.angular_res =   1 * pi/180;     
            
            
        end
        
        function hist = detectObstacle(sensor, numSector, robot, obstacle)
                i = 1;  
                alpha = deg2rad(sensor.resolutionDeg); 	
                histMag = zeros(numSector,1);
                hist = zeros(numSector,1);           %initial 
                scannedObs = zeros(0, 2);
                is = 1;
                H = zeros(numSector, 1);
                while (i <= length(obstacle))						%traverse obstacles �??璐無bstacle in view!!!
                    d = norm(obstacle(i,:) - robot.Pos);% - robot.Radius;			%鏉╂瑤閲滈梾婊咁暡娑撳孩�?崳銊ゆ眽娑斿妫跨捄婵堫�?
                    theta = caculateangle(robot.Pos, obstacle(i, :));
                    if d < 0 
                        d = 0;
                    end
                    if d < sensor.dSensorMax %& abs( mod(mod(theta, 2*pi) - mod(robot.dir, 2*pi), 2*pi) ) < deg2rad(sensor.angleSensorMax)

                        scannedObs(is, :) = obstacle(i, :);        

                        gamma = asin(robot.Radius / norm(scannedObs(is, :) - robot.Pos));
                        is = is + 1;

                        beta = caculateangle(robot.Pos,obstacle(i,:));

                        k = ceil(beta/alpha);					%the obstacle is on the k th direction

                        m = sensor.C^2*(sensor.a-sensor.b*d); 	

                        %histMag(k)=histMag(k)+m;  						%sector k hk
                        for kk = 1 : numSector
                            if kk * alpha > beta - gamma & kk * alpha < beta + gamma 
                                H(kk) = H(kk) + m;
                            end
                        end

                        i=i+1;
                    else
                        i=i+1;
                    end
                end	
                hist=zeros(numSector,0);
                histMag = H;
                % low-pass filter
                for i = 1 : numSector
                    for j = -4 : 1 : 4
                        if (i - j)<1
                            temp=i-j+numSector;
                        elseif (i-j)>numSector
                            temp=i-j-numSector;
                        else temp=i-j;
                        end
                        if j<0
                            num=5+j;
                        else num=5-j;
                        end
                        hist(i)=histMag(i)+num*histMag(temp);
                    end
                    hist(i)=hist(i)/9;
                end
                sensor.hist = hist;
                sensor.scannedObstacle = scannedObs';
        end

        function [y, lmksVisible] = scanLmks(sen, y, lmks)
            

        end
        
        % Find all measurements that are out of the sensor's range and set
        % them to an impossible value so that they can be removed
        function [y, idx] = constrainMeasurement(sen, y)
            off_range = y(1, :) > sen.range | y(1, :) < sen.range_min;
            off_ang = y(2, :) > sen.fov/2 | ...
                y(2, :) < -sen.fov/2;
            y(:, off_range) = inf;
            y(:, off_ang) = inf;
            idx = find(y(1, :) < inf | y(2, :) < inf);
        end
        
        % Generate a scan where the full sweep gives the max range value
        function y = generateEmptyScan(sen)
            readings_a = -sen.fov/2 : sen.angular_res : sen.fov/2;
            numLasers = length(readings_a);
            y = [repmat(sen.range, 1, numLasers); readings_a];
        end
        
        

    end
end

function angle = caculateangle(startPos ,endPos)
    vector1 = endPos - startPos;
    vector2 = [1, 0];
    angle = abs( acos ( sum(vector1.*vector2) / norm(vector1) / norm(vector2) ) );
end