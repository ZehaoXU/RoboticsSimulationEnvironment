function World = doMapping(World, sen, r_new, scan_data)

    %%%%%%%%%%%%%%%%%%%%%%%%% MAPPING SETTINGS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Amount by which to increment/reduce cell value when obstacle is detected:
    map_augment = 20;
    map_reduce = 20; 
    map_max = 255;          % Maximum value for a grid cell
    map_min = 0;            % Minimum value for a grid cell
    map_isOccupied = 150;   % Value at which cell is in occupied state
    map_isEmpty = 50;       % Value at which cell is in empty state

    scan_polar = getMeasurement(scan_data);
    empty_region_pol = ones(2, size(scan_polar, 2) * (round((sen.range - sen.range_min)/World.map_res) + 1)) * -1;
    position = 1;
    for i = 1:size(scan_polar, 2)
        empty_region_r = sen.range_min : World.map_res : scan_polar(1, i) - World.map_res;
        empty_region_a = repmat(scan_polar(2, i), 1, length(empty_region_r));
        empty_region_pol(:, position : position + length(empty_region_r) - 1) ...
            = [empty_region_r; empty_region_a];
        position = position + length(empty_region_r);
    end
    empty_region_pol = empty_region_pol(:, empty_region_pol(1, :) ~= -1);
    empty_region = invScanPoint(r_new, empty_region_pol);

    scan_global = scan_data;
    gridmap_occupied = zeros(size(World.gridmap));
    gridmap_empty = zeros(size(World.gridmap));
    scan_global = interp1(World.map_vals, World.map_vals, scan_global, 'nearest');
    empty_region = interp1(World.map_vals, World.map_vals, empty_region, 'nearest');

    for i = 1:length(World.map_vals)
        for j = 1:length(World.map_vals)
            if ~isempty(scan_global)
                gridmap_occupied(i, j) = sum(scan_global(1,:) == World.map_vals(j) & ...
                    scan_global(2,:) == World.map_vals(i));
            end
        end
    end
    
    World.gridmap_counter = World.gridmap_counter ...
        + map_augment * gridmap_occupied ...
        - map_reduce * gridmap_empty;
        
    World.gridmap_counter(World.gridmap_counter > map_max) = map_max;
    World.gridmap_counter(World.gridmap_counter < map_min) = map_min;
    
    gridmap_occupied = World.gridmap_counter > map_isOccupied;
    gridmap_empty = World.gridmap_counter < map_isEmpty;
    World.gridmap = ones(size(World.gridmap_counter)) * 0.5;
    World.gridmap(gridmap_occupied) = 1;
    World.gridmap(gridmap_empty) = 0;
    
    World.gridmap_greyscale = mat2gray(World.gridmap_counter, [map_min map_max]);

    
    subplot(224); imagesc(flipud(World.gridmap_greyscale(1:20, 1:20)))     
end