%% doRANSAC.m : registration of two data sets
% Input    : Data1 - 3 x n matrix of the x, y and z coordinates of data set 1
%            Data2 - 3 x m matrix of the x, y and z coordinates of data set 2
%            Rslt   - the tolerance distance for establishing closest point
%                     correspondences. Normally set equal to the resolution
%                     of data1
%            u     - 2 x 1 matrix in local polar co-ordinates with initial
%                    guess of rototranslation
%
% Outpur : R    - 2-by-2 Rotation matrix used to register data2
%          t    - 2-by-1 accumulative translation vector used to register data2
%          Corr - N_corr-by-3 matrix of the index no.s of the corresponding points of
%                 data1 and data2 and their corresponding Euclidean distance
%          err - the mean error between the corresponding points of data1
%                  and data2 (normalized with res)
%          Data2_R - 3 x m matrix of the registered data2 

function [R, t, Corr, Var, Data2_R] = doRANSAC(Data1, Data2, Rslt, u)
% -------------------------------------------------------------------------
% ---------------------------- 1. Initialization --------------------------
% -------------------------------------------------------------------------

maxIter = 20;
R = eye(2);
t = zeros(2,1);     %output

R1 = [cos(u(2)) -sin(u(2)); sin(u(2)) cos(u(2))]; % Determine R1 and t1 from odometry input u:
t1 = [u(1); 0];
R_init = R1;
t_init = t1;

tri = delaunayn(Data1');

n = 0;
Data2_R = Data2;
% ------------------------------------------------------------------------
% --------- 2. using ICP to determine the correspongding points ----------
% ------------------------------------------------------------------------
flag1 = 0; flag2 = 1;
while flag2 ~= flag1
    flag1 = flag2;
    [Corr, D] = dsearchn(Data1', tri, Data2_R'); % N-D nearest point search % 看看这个是什么！
    Corr(:,2:3) = [[1 : length(Corr)]' D];    
    Corr(find(D > 2*Rslt),:) = []; % remove corr points that without 2 times resolution
    
    Corr = -sortrows(-Corr,3);
    Corr = sortrows(Corr,1);
    [B, Bi, Bj] = unique(Corr(:,1));
    Corr = Corr(Bi,:);

    Data2_R = R1*Data2_R;
    Data2_R = [Data2_R(1,:)+t1(1); Data2_R(2,:)+t1(2)];
    R = R1 * R;
    t = R1 * t + t1;    
    flag2 = length(Corr);        
    n = n + 1;
    if n > maxIter
        break;
    end
    % If there is no correlation revert to odometry data
    if isempty(Corr)
        R = R_init;
        t = t_init;
        break;
    end
    Set1 = Data1(:, Corr(:,1));
    Set2 = Data2_R(:, Corr(:,2));
    [R1, t1] = getRTmatrix(Set1, Set2);
end
e1 = 1000001;
e2 = 1000000;
n = 0;
noChangeCount = 0;
while noChangeCount < 10
    e1 = e2;
    [Corr, D] = dsearchn(Data1', tri, Data2_R');
    Corr(:,2:3) = [[1 : length(Corr)]' D];    
    Corr(D>2*Rslt,:) = [];

    Corr = -sortrows(-Corr,3);
    Corr = sortrows(Corr,1); 
    [B, Bi, Bj] = unique(Corr(:,1));
    Corr = Corr(Bi,:);

    % If there is no correlation revert to odometry data
    if isempty(Corr)
        R = R_init;
        t = t_init;
        break;
    end
    Set1 = Data1(:, Corr(:,1));
    Set2 = Data2_R(:, Corr(:,2)); % corresponding data
    [R1, t1] = getRTmatrix(Set1, Set2); 
    Data2_R = R1*Data2_R;
    Data2_R = [Data2_R(1,:)+t1(1); Data2_R(2,:)+t1(2)];    
    R = R1*R;
    t = R1*t + t1;    
    e2 = sum(Corr(:,3))/(length(Corr)*Rslt);
   
    n = n + 1;
    if n > maxIter        
        break;
    end
    if abs(e2-e1)<Rslt/1000
        noChangeCount = noChangeCount + 1;
    end
end
% now we get the estimated corresponding matrix Corr
% R
% -------------------------------------------------------------------------
% --------- 3. using RANSAC to get the real correspongding matrix ---------
% -------------------------------------------------------------------------
sigma2 = sum(Corr(:,3).^2 ) / (2*length(Corr)-3);
sigma2 = 1;
% disp(sigma2);
% test only
model = [0, 0, 0]; % [theta, t'] 
resModel = model;

tt = 2.2 * sqrt(sigma2);

N = length(Corr); % Total num of samples
n = 3;    % sample number. there are 6 params in model, so at least 3 points are to be selected
epsilon = 0.95; % inlier proportion(estimated)
P = 1 - (1 - epsilon)^n;
K = log(1 - P)/log(1 - epsilon^n); 
K = N;
iter = 0;
maxInterNum = 0;
Set1 = Data1(:, Corr(:,1));
Set2 = Data2(:, Corr(:,2)); % corresponding data

while iter < K
   % 1. randomly selected n points within data set 
   sampleIndex = randperm(N, n);
   [R0, t0] = getRTmatrix(Set1(:, sampleIndex), Set2(:, sampleIndex));
   model = [acos(R0(1)), t0'];
   [interPointNum, tempIsInterPoint] = getInterPointNum(Set1, Set2, model, tt);
   if interPointNum > maxInterNum
      maxInterNum = interPointNum;
      resModel = model;
      isInterPoint = tempIsInterPoint;    
   end
   iter = iter + 1;
end
% model = resModel;
% model = [ [cos(model(1)); -sin(model(1))], [sin(model(1)); cos(model(1))], [model(2); model(3)]]

% determing real R and t
% resModel
% tic

options = optimoptions(@fmincon,'Display', 'off', 'OptimalityTolerance', 1e-4);
% robustCostModel = fmincon(@(resModel)robustCostFun(Set1, Set2, tt, resModel), resModel,[], [], [], [], [], [], [], options);
% resModel = robustCostModel;
model = resModel;
model = [ [cos(model(1)); -sin(model(1))], [sin(model(1)); cos(model(1))], [model(2); model(3)]];
resModel = model;
% [interPointNum, tempIsInterPoint] = getInterPointNum(Set1, Set2, robustCostModel, t);
% toc

% -------------------------------------------------------------------------
% ----------------------------- 4. results --------------------------------
% -------------------------------------------------------------------------
R = resModel(:,1:2);
t = resModel(:,3); 
min_error2 = sum(Corr(:,3))^2;  % Least squares minimum error
Data2_R = Data2_R;
Var = min_error2/(2*length(Corr)-3);    % Estimated variance
end

%% using SVD to get rotation and transmission matrix
function [R1, t1] = getRTmatrix(Set1, Set2)
% Set1 and Set2 are corresponding data sets
m1 = mean(Set1,2);
m2 = mean(Set2,2); 
Set1 = [Set1(1,:)-m1(1); Set1(2,:)-m1(2)];
Set2 = [Set2(1,:)-m2(1); Set2(2,:)-m2(2)];
K = Set2*Set1';% svd K and we can get rotation matrix
K = K/length(Set1); 
[U A V] = svd(K); % http://blog.csdn.net/dfdfdsfdfdfdf/article/details/53213240?locationNum=2&fps=1
R1 = V*U';
if det(R1)<0
    B = eye(2);
    B(2,2) = det(V*U');
    R1 = V*B*U';
end
t1 = m1 - R1*m2;
end

function [interPointNum, tempIsInterPoint] = getInterPointNum(Set1, Set2, model, t)
    num = 0;
    tempIsInterPoint = zeros(1, length(Set1));
    model = [ [cos(model(1)); sin(model(1))], [-sin(model(1)); cos(model(1))], [model(2); model(3)]];
    for i = 1 : length(Set1)
       if (norm( model(:, 1:2) * Set2(:, i) + model(:, 3) - Set1(:, i)))^2 <= t^2 
            num = num + 1;
            tempIsInterPoint(i) = 1;
       else
            tempIsInterPoint(i) = 0;
       end
    end
    interPointNum = num;
end

function cost = robustCostFun(Set1, Set2, t, resModel)
    model = resModel;
    resModel = [ [cos(model(1)); sin(model(1))], [-sin(model(1)); cos(model(1))], [model(2); model(3)]];
    cost = 0;
    for i = 1 : length(Set1)
        temp = norm( resModel(:, 1:2) * Set2(:, i) + resModel(:, 3) - Set1(:, i) )^2;
        if temp > t ^2
            cost = cost + t^2;
        else
            cost = cost + temp^2;
        end
    end
end
