function [robot, sensor, hist, kHist]=moveVFH(endPoint, obstacle, iter, robot, sensor)
global stepT;
%% initialization
v = 0.1;								
alpha = deg2rad(sensor.resolutionDeg); 					%rad resolution
                                	%max certainty value                
numSector = 360 / sensor.resolutionDeg;                                %sectors
threshold = 30;                            
                       		
kHist = ceil(caculateangle(robot.Pos, endPoint)/alpha);	%hist Sector

%% 
						
    % we get magnitude of histogram 
    % update hist!!!!!!!!!!!!!

    sensor.hist = sensor.detectObstacle(numSector, robot, obstacle);
    hist = sensor.hist;
    % smooth the sector

    % update hist considering the size of the robot


    %best moving direction dirRobot锛宯ext pos robot.Pos
    j=1;
    

    % c all possible dir 
    % Right means to right
    q=2;
    kHistRightFirst = 0;
    c = [];
    while (q <= numSector + kHistRightFirst + 1)       								% traverse all possible dir
        if hist( mod(q - 1, numSector) + 1 ) <= threshold && hist(mod(q - 1 - 1, numSector) + 1) > threshold
            kHistRight = mod(q - 1, numSector) + 1;                  					% right edge of hist
            if kHistRightFirst == 0
                kHistRightFirst = kHistRight;
            end
            while(q <= numSector + kHistRightFirst && hist( mod(q - 1, numSector) + 1) <= threshold)  		    %left edge
                kHistLeft=mod(q - 1, numSector) + 1;
                q = q + 1;
            end
            if( mod(kHistLeft - kHistRight - 1, numSector) + 1> sensor.dValleyMax) 						%wide valley
                c(j)   = round( mod(kHistLeft - sensor.dValleyMax/2  - 1, numSector) +1 ); 		%towards the left side
                c(j+1) = round( mod(kHistRight + sensor.dValleyMax/2 - 1, numSector) + 1 ); 		%towards the right side
                j=j+2;
                if(kHist >= kHistRight+4 && kHist <= kHistLeft-4)
                    c(j) = kHist; 						% straight at look ahead
                    j=j+1;
                end
            elseif(kHistLeft-kHistRight > sensor.dValleyMax / 5) 					%narrow valley
                c(j) = round((kHistRight+kHistLeft)/2);
                j=j+1;
            end
        else
            q=q+1;            %hist(q) > threshold
        end   
    end                      
    %c have all the proper angle
    if isempty(c)
         c(1) = kHist;
         j = j + 1;
    end
    how = zeros(j-1,1);
    for (i=1:j-1)
        how(i)=howmany(c(i),kHist);		%
    end                         		
    ft = find(how==min(how));			%
    fk = ft(1);
    robot.dir = c(fk);  							
   
    
    %------------new position-------------
    robot.Pos=robot.Pos+[v * stepT *cos(robot.dir*alpha),v * stepT * sin(robot.dir*alpha)];
    %plotAll(robot.Pos, hist, kHist, robot.dir);
    

    kHist=ceil(caculateangle(robot.Pos,endPoint)/alpha);


end

function obstacle = getObstacle
    obstacle=[];
    newmap;
    h=findobj(gcf, 'Color', 'k');
    x=get (h, 'xdata');
    y=get (h, 'ydata');
    numSector=size(x);
    x1=cell2mat(x);
    y1=cell2mat(y);
    obstacle=[x1, y1];
end

%
function angle = caculateangle(startPos ,endPos)

    vector1 = endPos - startPos;
    vector2 = [1, 0];
    angle = abs( acos ( sum(vector1.*vector2) / norm(vector1) / norm(vector2) ) );
end

function dif=howmany(c1,c2)
    n = 72; % number of sektors 
    dif = min([abs(c1-c2), abs(c1-c2-n), abs(c1-c2+n)]);
end



