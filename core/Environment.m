classdef Environment < handle
    % Environment - 2D world with boundaries and obstacles
    
    properties
        x_lim       % [xmin, xmax]
        y_lim       % [ymin, ymax]
        obstacles   % cell array of obstacle structs
        goal        % [x; y] goal position (optional)
    end
    
    methods
        function obj = Environment(x_lim, y_lim)
            obj.x_lim     = x_lim;
            obj.y_lim     = y_lim;
            obj.obstacles = {};
            obj.goal      = [];
        end
        
        function add_circular_obstacle(obj, center, radius)
            obs.type   = 'circle';
            obs.center = center;
            obs.radius = radius;
            obj.obstacles{end+1} = obs;
        end
        
        function add_rectangular_obstacle(obj, x_range, y_range)
            obs.type    = 'rect';
            obs.x_range = x_range;
            obs.y_range = y_range;
            obj.obstacles{end+1} = obs;
        end
        
        function result = in_collision(obj, position)
            % returns true if position collides with any obstacle
            result = false;
            for k = 1:length(obj.obstacles)
                obs = obj.obstacles{k};
                if strcmp(obs.type, 'circle')
                    if norm(position - obs.center) <= obs.radius
                        result = true; return;
                    end
                elseif strcmp(obs.type, 'rect')
                    if position(1) >= obs.x_range(1) && position(1) <= obs.x_range(2) && ...
                       position(2) >= obs.y_range(1) && position(2) <= obs.y_range(2)
                        result = true; return;
                    end
                end
            end
        end
        
        function result = in_bounds(obj, position)
            result = position(1) >= obj.x_lim(1) && position(1) <= obj.x_lim(2) && ...
                     position(2) >= obj.y_lim(1) && position(2) <= obj.y_lim(2);
        end
    end
end
