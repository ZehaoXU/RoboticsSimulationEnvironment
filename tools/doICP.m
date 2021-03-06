% Arguments: data1 - 3 x n matrix of the x, y and z coordinates of data set 1
%            data2 - 3 x m matrix of the x, y and z coordinates of data set 2
%            res   - the tolerance distance for establishing closest point
%                     correspondences. Normally set equal to the resolution
%                     of data1
%           分辨率 这玩意牛逼
%            u     - 2 x 1 matrix in local polar co-ordinates with initial
%                    guess of rototranslation
%            tri   - optional argument. obtained by tri = delaunayn(data1');
%
% Returns: R - 3 x 3 accumulative rotation matrix used to register data2
%          t - 3 x 1 accumulative translation vector used to register data2
%          corr - p x 3 matrix of the index no.s of the corresponding points of
%                 data1 and data2 and their corresponding Euclidean distance
%          error - the mean error between the corresponding points of data1
%                  and data2 (normalized with res)
%          data2 - 3 x m matrix of the registered data2 
%
%

maxIter = 50;
c1 = 0;
c2 = 1;
R = eye(2);
t = zeros(2,1);

% Determine R1 and t1 from odometry input u:
% 转过了多少角度

R1 = [cos(u(2)) -sin(u(2)); sin(u(2)) cos(u(2))];
t1 = [u(1); 0];

R_init = R1;
t_init = t1;

if nargin < 5
    tri = delaunayn(data1');
end
% 三角剖分
n = 0;
while c2 ~= c1
    c1 = c2;
    [corr, D] = dsearchn(data1', tri, data2'); % N-D nearest point search
    corr(:,2:3) = [[1 : length(corr)]' D];    
    corr(find(D>2*res),:) = [];
    
    corr = -sortrows(-corr,3);
    corr = sortrows(corr,1);
    [B, Bi, Bj] = unique(corr(:,1));
    corr = corr(Bi,:);

    data2 = R1*data2;
    data2 = [data2(1,:)+t1(1); data2(2,:)+t1(2)];
    R = R1*R;
    t = R1*t + t1;    
    c2 = length(corr);        
    n = n + 1;
    if n > maxIter
        break;
    end
    
    if isempty(corr)
        R = R_init;
        t = t_init;
        break;
    end
    [R1, t1] = reg(data1, data2, corr);
end

e1 = 1000001;
e2 = 1000000;
n = 0;
noChangeCount = 0;
while noChangeCount < 10
    e1 = e2;
    [corr, D] = dsearchn(data1', tri, data2');
    corr(:,2:3) = [[1 : length(corr)]' D];    
    corr(D>2*res,:) = [];

    corr = -sortrows(-corr,3);
    corr = sortrows(corr,1); 
    [B, Bi, Bj] = unique(corr(:,1));
    corr = corr(Bi,:);

    % If there is no correlation revert to odometry data
    if isempty(corr)
        R = R_init;
        t = t_init;
        break;
    end
    [R1 t1] = reg(data1, data2, corr); 
    data2 = R1*data2;
    data2 = [data2(1,:)+t1(1); data2(2,:)+t1(2)];    
    R = R1*R;
    t = R1*t + t1;    
    e2 = sum(corr(:,3))/(length(corr)*res);
   
    n = n + 1;
    if n > maxIter        
        break;
    end
    if abs(e2-e1)<res/10000
        noChangeCount = noChangeCount + 1;
    end
end
error = min(e1,e2);     % Minimum error
min_error2 = sum(corr(:,3))^2;  % Least squares minimum error
icp_var = min_error2/(2*length(corr)-3);    % Estimated variance

%-----------------------------------------------------------------
function [R1, t1] = reg(data1, data2, corr)
n = length(corr); 
M = data1(:,corr(:,1)); 
mm = mean(M,2);
S = data2(:,corr(:,2));
ms = mean(S,2); 
% 对应的两个
Sshifted = [S(1,:)-ms(1); S(2,:)-ms(2)];
Mshifted = [M(1,:)-mm(1); M(2,:)-mm(2)];
K = Sshifted*Mshifted';% using svd to get rotation matrix
K = K/n;
[U A V] = svd(K);
R1 = V*U';
if det(R1)<0
    B = eye(2);
    B(2,2) = det(V*U');
    R1 = V*B*U';
end
t1 = mm - R1*ms;