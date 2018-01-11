function plotAll(robot, H, kt, kb, scan_data)
    subplot(2, 2, 1); 
    Rob = line(...
        'linestyle','none',...
        'marker','.',...
        'color','b',...
        'xdata',[],...
        'ydata',[]);
    set(Rob, 'xdata', robot.Pos(1), 'ydata', robot.Pos(2))
   
    if ~isempty(scan_data)
    % scatter(scan_data(1, :), scan_data(2, :), 'xr');
     Scan = line(...
        'linestyle','none',...
        'marker','x',...
        'color','r',...
        'xdata',[],...
        'ydata',[]);
     set(Scan, 'xdata', scan_data(1, :), 'ydata', scan_data(2, :))
    end
    subplot(2, 2, 2); 
    %-----------------
    n=length(H);
    x1=[1:n];
    k1=kt;   
    k2=kb;
    y=[0:max(H)];
    if(max(H) <=1)
        y=[0:0.01:1]; 	%to get a smoother line
    end
    hold off		
    bar(x1,	H, 'b');            %plot the histogram
    hold on;
    ylabel('H(k)');
    xlabel('sector k');

    line([k1,k1],[0,max(H)],'LineStyle','-', 'color','r');
    line([k2,k2],[0,max(H)],'LineStyle','--', 'color','g');
    %set(gca, 'Units', 'normalized', 'Position', [0 .7 0.8 0.2])
    subplot(223)
    hold off
    if ~isempty(scan_data)
    scatter(scan_data(1, :), scan_data(2, :), 'xr');
    end

    drawnow;