classdef RRT < handle
    properties
        env
        max_iter = 5000;
        step_size = 2.0;
        goal_sample_rate = 0.1;
    end
    
    methods
        function obj = RRT(environment, max_iter, step_size)
            obj.env = environment;
            if nargin > 1
                obj.max_iter = max_iter;
            end
            if nargin > 2
                obj.step_size = step_size;
            end
        end
        
        function [path, tree] = plan(obj, start_pos, goal_pos, goal_radius)
            if nargin < 4
                goal_radius = 2.0;
            end
            
            tree.vertices = start_pos(:);
            tree.parents = 0;
            
            for iter = 1:obj.max_iter
                if rand < obj.goal_sample_rate
                    rand_point = goal_pos(:);
                else
                    rand_point = obj.sample_free();
                end
                
                nearest_idx = obj.nearest(tree, rand_point);
                nearest_point = tree.vertices(:, nearest_idx);
                
                new_point = obj.steer(nearest_point, rand_point);
                
                if ~obj.env.in_collision(new_point) && ...
                   ~obj.collision_free_path(nearest_point, new_point)
                    tree.vertices = [tree.vertices, new_point];
                    tree.parents  = [tree.parents, nearest_idx];
                    
                    if norm(new_point - goal_pos(:)) < goal_radius
                        path = obj.extract_path(tree, length(tree.parents));
                        return;
                    end
                end
            end
            
            path = [];
            warning('RRT: Max iterations reached without finding path');
        end
        
        function point = sample_free(obj)
            % Use x_lim and y_lim from Environment
            x_range = obj.env.x_lim;
            y_range = obj.env.y_lim;
            
            point = [x_range(1) + rand*(x_range(2) - x_range(1));
                     y_range(1) + rand*(y_range(2) - y_range(1))];
        end
        
        function idx = nearest(obj, tree, point)
            dists = vecnorm(tree.vertices - point);
            [~, idx] = min(dists);
        end
        
        function new_point = steer(obj, from_point, to_point)
            direction = to_point - from_point;
            dist = norm(direction);
            if dist < obj.step_size
                new_point = to_point;
            else
                new_point = from_point + (direction/dist) * obj.step_size;
            end
        end
        
        function collision = collision_free_path(obj, p1, p2)
            n_checks = ceil(norm(p2 - p1) / 0.5);
            collision = false;
            for k = 1:n_checks
                alpha = k / n_checks;
                point = p1 + alpha * (p2 - p1);
                if obj.env.in_collision(point)
                    collision = true;
                    return;
                end
            end
        end
        
        function path = extract_path(obj, tree, goal_idx)
            path = tree.vertices(:, goal_idx);
            current_idx = goal_idx;
            while tree.parents(current_idx) ~= 0
                current_idx = tree.parents(current_idx);
                path = [tree.vertices(:, current_idx), path];
            end
        end
    end
end
