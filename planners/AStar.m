classdef AStar < handle
    properties
        grid_resolution = 1.0;
        env
        grid_size
        obstacles_grid
    end
    
    methods
        function obj = AStar(environment, resolution)
            obj.env = environment;
            obj.grid_resolution = resolution;
            obj.build_grid();
        end
        
        function build_grid(obj)
            x_range = obj.env.bounds(1,:);
            y_range = obj.env.bounds(2,:);
            
            nx = ceil((x_range(2) - x_range(1)) / obj.grid_resolution);
            ny = ceil((y_range(2) - y_range(1)) / obj.grid_resolution);
            obj.grid_size = [nx, ny];
            
            obj.obstacles_grid = false(nx, ny);
            
            for i = 1:nx
                for j = 1:ny
                    x = x_range(1) + (i-0.5)*obj.grid_resolution;
                    y = y_range(1) + (j-0.5)*obj.grid_resolution;
                    
                    if obj.env.is_collision([x; y], 0.5)
                        obj.obstacles_grid(i,j) = true;
                    end
                end
            end
        end
        
        function path = plan(obj, start_pos, goal_pos)
            start_idx = obj.pos_to_grid(start_pos);
            goal_idx  = obj.pos_to_grid(goal_pos);
            
            if obj.obstacles_grid(start_idx(1), start_idx(2)) || ...
               obj.obstacles_grid(goal_idx(1), goal_idx(2))
                path = [];
                warning('Start or goal in obstacle');
                return;
            end
            
            open_set = [start_idx, 0, obj.heuristic(start_idx, goal_idx)];
            closed_set = false(obj.grid_size);
            came_from = zeros([obj.grid_size, 2]);
            g_score = inf(obj.grid_size);
            g_score(start_idx(1), start_idx(2)) = 0;
            
            while size(open_set, 1) > 0
                [~, min_idx] = min(open_set(:,4));
                current = open_set(min_idx, 1:2);
                
                if all(current == goal_idx)
                    path = obj.reconstruct_path(came_from, current);
                    return;
                end
                
                open_set(min_idx, :) = [];
                closed_set(current(1), current(2)) = true;
                
                neighbors = obj.get_neighbors(current);
                for k = 1:size(neighbors, 1)
                    neighbor = neighbors(k, :);
                    
                    if closed_set(neighbor(1), neighbor(2))
                        continue;
                    end
                    
                    tentative_g = g_score(current(1), current(2)) + ...
                                  norm(neighbor - current);
                    
                    if tentative_g < g_score(neighbor(1), neighbor(2))
                        came_from(neighbor(1), neighbor(2), :) = current;
                        g_score(neighbor(1), neighbor(2)) = tentative_g;
                        f_score = tentative_g + obj.heuristic(neighbor, goal_idx);
                        
                        in_open = false;
                        for m = 1:size(open_set, 1)
                            if all(open_set(m, 1:2) == neighbor)
                                open_set(m, 3:4) = [tentative_g, f_score];
                                in_open = true;
                                break;
                            end
                        end
                        
                        if ~in_open
                            open_set = [open_set; neighbor, tentative_g, f_score];
                        end
                    end
                end
            end
            
            path = [];
            warning('No path found');
        end
        
        function idx = pos_to_grid(obj, pos)
            x_range = obj.env.bounds(1,:);
            y_range = obj.env.bounds(2,:);
            
            i = round((pos(1) - x_range(1)) / obj.grid_resolution) + 1;
            j = round((pos(2) - y_range(1)) / obj.grid_resolution) + 1;
            
            i = max(1, min(i, obj.grid_size(1)));
            j = max(1, min(j, obj.grid_size(2)));
            idx = [i, j];
        end
        
        function pos = grid_to_pos(obj, idx)
            x_range = obj.env.bounds(1,:);
            y_range = obj.env.bounds(2,:);
            
            x = x_range(1) + (idx(1)-0.5)*obj.grid_resolution;
            y = y_range(1) + (idx(2)-0.5)*obj.grid_resolution;
            pos = [x; y];
        end
        
        function h = heuristic(obj, idx1, idx2)
            h = norm(idx1 - idx2);
        end
        
        function neighbors = get_neighbors(obj, idx)
            directions = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
            neighbors = [];
            
            for k = 1:size(directions, 1)
                n = idx + directions(k, :);
                
                if n(1) >= 1 && n(1) <= obj.grid_size(1) && ...
                   n(2) >= 1 && n(2) <= obj.grid_size(2) && ...
                   ~obj.obstacles_grid(n(1), n(2))
                    neighbors = [neighbors; n];
                end
            end
        end
        
        function path = reconstruct_path(obj, came_from, current)
            path_idx = current;
            
            while any(came_from(current(1), current(2), :))
                current = squeeze(came_from(current(1), current(2), :))';
                path_idx = [current; path_idx];
            end
            
            path = zeros(2, size(path_idx, 1));
            for k = 1:size(path_idx, 1)
                path(:, k) = obj.grid_to_pos(path_idx(k, :));
            end
        end
    end
end
