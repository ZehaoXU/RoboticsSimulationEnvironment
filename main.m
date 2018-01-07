% Author: Zehao Xu, Tianqing Fang  @ Zhejiang University
%  a really simple simulation environment for SLAM and obstacle-
%  avoidance robot system
%% ==================================================
%% ==============1. build map ================
addpath('maps', 'tools')
subplot(221)
newmap;
load obstacle;
load landmarks;
global stepT t;  % time interval
global World;
stepT = 1;
t = 0;
startPoint=[1 1];
endPoint = [9 9];
pause
%% ==========2. initialize robot and sensor ==========
robot = Robot;
robot.Pos = startPoint;  

robot.dir = caculateangle(robot.Pos, endPoint);



iter = 1;

sensor = Sensor;
sensor_rf = Sensor;
sensor_rf.noise = [0.1; 3 * pi/180]; % 
sensor_rf.range = 10;
sensor.range = 30;
sensor.fov = 270 * pi/180;
robot.q = [0.1;2*pi/180];
    
% initialize
[World, robot] = configuration(robot, sensor_rf, landmarks);
numObstacles = length(obstacle);
numSegments = 0;
% Initialise some loop variables
pose_this_scan = [];
this_scan_good = 0;
big_turn = 0;
r = World.r;    % Location of robot pose in mapspace
% % Estimated pose [x y a]
    

%% ===========3. main loop ==========
while norm(robot.Pos - endPoint) > 0.2
    [robot sensor] = moveVFH(endPoint, obstacle, iter, robot, sensor);
    % TODO: change
    robot.R = [robot.Pos, robot.dir];

    scan_data = EKFSLAM(robot, sensor, sensor_rf, obstacle, landmarks, this_scan_good, pose_this_scan);
    
    plotAll(robot.Pos, sensor.hist, robot.dir, ceil(caculateangle(robot.Pos, endPoint)/deg2rad(sensor.resolutionDeg)), scan_data);
    frame=getframe(gcf);
    
    im=frame2im(frame); 
     [I,map]=rgb2ind(im,256);
     if iter==1;
        imwrite(I,map,'F:\test.gif','gif','Loopcount',inf,...
            'DelayTime',0.1); 
    else
        imwrite(I,map,'F:\test.gif','gif','WriteMode','append',...
            'DelayTime',0.1); 
    end
    iter = iter + 1;
end


