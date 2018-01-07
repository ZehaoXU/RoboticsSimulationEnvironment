%[2] ’œ∞≠µ„
function drawobstacle
global obstacle;
axis([0 10 0 10]);
grid on;

for i=1:length(obstacle)
    plot(obstacle(i,1),obstacle(i,2),'.k');
    hold on;
end
