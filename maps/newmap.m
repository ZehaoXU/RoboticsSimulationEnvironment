% build map
function [landmarks obstacle] = newmap
global obstacle landmarks
subplot(221)
clear
axis([0 10 0 10]);
hold on
grid on;
 

load obstacle;
load landmarks;

plot(obstacle(:, 1) , obstacle(:, 2), '.k'); 
scatter(landmarks(:, 1), landmarks(:, 2), 'xb'); 
end
