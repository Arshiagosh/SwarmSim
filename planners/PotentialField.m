classdef PotentialField < handle
    % PotentialField - Artificial potential field path planner
    % Fixed version with correct obstacle schema and collision handling
    %
    % Source: AI_Codebase.txt:1333-1412 (original, buggy)
    
    properties
        k_att = 1.0;        % attractive gain
        k_rep = 100.0;      % repulsive gain (increased for stronger avoidance)
        d0 = 5.0;           % obstacle influence distance
        env                 % environment reference
        max_vel = 5.0;      % maximum velocity magnitude
        inside_rep = 500.0; % strong repulsion when inside obstacle
    end
    
    methods
        function obj = PotentialField(environment, k_att, k_rep, d0)
            obj.env = environment;
            if nargin > 1 && ~isempty(k_att), obj.k_att = k_att; end
            if nargin > 2 && ~isempty(k_rep), obj.k_rep = k_rep; end
            if nargin > 3 && ~isempty(d0), obj.d0 = d0; end
        end
        
        function force = compute_force(obj, pos, goal)
            % Compute total force as sum of attractive and repulsive
            f_att = obj.attractive_force(pos, goal);
            f_rep = obj.repulsive_force(pos);
            force = f_att + f_rep;
        end
        
        function f = attractive_force(obj, pos, goal)
            % Standard attractive force toward goal
            diff = goal(:) - pos(:);
            dist = norm(diff);
            if dist > 0
                f = obj.k_att * diff;
            else
                f = zeros(2, 1);
            end
        end
        
        function f = repulsive_force(obj, pos)
            % Repulsive force from all obstacles
            % FIX: Handles both 'circle' and 'rect' types correctly
            % FIX: Provides strong repulsion when INSIDE obstacles
            
            f = zeros(2, 1);
            pos = pos(:);
            
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                
                if strcmp(obs.type, 'circle')
                    % Circular obstacle
                    center = obs.center(:);
                    diff = pos - center;
                    dist_to_center = norm(diff);
                    d = dist_to_center - obs.radius;  % signed distance to surface
                    
                    if d <= 0
                        % INSIDE obstacle: apply strong outward force
                        if dist_to_center > 1e-6
                            direction = diff / dist_to_center;
                        else
                            % At center: push in random direction
                            direction = [1; 0];
                        end
                        f = f + obj.inside_rep * direction;
                        
                    elseif d < obj.d0
                        % Within influence distance: standard repulsion
                        direction = diff / dist_to_center;
                        f = f + obj.k_rep * (1/d - 1/obj.d0) * (1/d^2) * direction;
                    end
                    
                elseif strcmp(obs.type, 'rect')
                    % FIX: Environment uses 'rect', not 'rectangle'
                    % FIX: Environment uses x_range/y_range, not bounds
                    
                    x_min = obs.x_range(1);
                    x_max = obs.x_range(2);
                    y_min = obs.y_range(1);
                    y_max = obs.y_range(2);
                    
                    % Find closest point on rectangle boundary
                    closest_x = max(x_min, min(pos(1), x_max));
                    closest_y = max(y_min, min(pos(2), y_max));
                    closest = [closest_x; closest_y];
                    
                    diff = pos - closest;
                    d = norm(diff);
                    
                    % Check if inside rectangle
                    inside = (pos(1) >= x_min && pos(1) <= x_max && ...
                              pos(2) >= y_min && pos(2) <= y_max);
                    
                    if inside
                        % INSIDE obstacle: push toward nearest edge
                        dist_to_edges = [pos(1) - x_min, x_max - pos(1), ...
                                         pos(2) - y_min, y_max - pos(2)];
                        [~, nearest_edge] = min(dist_to_edges);
                        
                        switch nearest_edge
                            case 1, direction = [-1; 0];  % push left
                            case 2, direction = [1; 0];   % push right
                            case 3, direction = [0; -1];  % push down
                            case 4, direction = [0; 1];   % push up
                        end
                        f = f + obj.inside_rep * direction;
                        
                    elseif d < obj.d0 && d > 1e-6
                        % Within influence distance: standard repulsion
                        direction = diff / d;
                        f = f + obj.k_rep * (1/d - 1/obj.d0) * (1/d^2) * direction;
                    end
                end
            end
        end
        
        function path = plan(obj, start_pos, goal_pos, max_steps, dt)
            % Plan path using gradient descent with collision recovery
            if nargin < 4 || isempty(max_steps), max_steps = 2000; end
            if nargin < 5 || isempty(dt), dt = 0.1; end
            
            path = start_pos(:);
            pos = start_pos(:);
            goal = goal_pos(:);
            
            stuck_count = 0;
            prev_pos = pos;
            
            for step = 1:max_steps
                force = obj.compute_force(pos, goal);
                vel = force;
                
                % Clamp velocity
                if norm(vel) > obj.max_vel
                    vel = vel / norm(vel) * obj.max_vel;
                end
                
                % Proposed new position
                new_pos = pos + vel * dt;
                
                % FIX: Check collision and don't allow stepping into obstacles
                if obj.env.in_collision(new_pos)
                    % Try smaller steps or only repulsive motion
                    for scale = [0.5, 0.25, 0.1]
                        test_pos = pos + vel * dt * scale;
                        if ~obj.env.in_collision(test_pos)
                            new_pos = test_pos;
                            break;
                        end
                    end
                    % If still colliding, only use repulsive force
                    if obj.env.in_collision(new_pos)
                        f_rep = obj.repulsive_force(pos);
                        if norm(f_rep) > 1e-6
                            new_pos = pos + (f_rep / norm(f_rep)) * obj.max_vel * dt * 0.5;
                        end
                    end
                end
                
                % Clamp to environment bounds
                new_pos(1) = max(obj.env.x_lim(1), min(new_pos(1), obj.env.x_lim(2)));
                new_pos(2) = max(obj.env.y_lim(1), min(new_pos(2), obj.env.y_lim(2)));
                
                pos = new_pos;
                path = [path, pos];
                
                % Check if reached goal
                if norm(pos - goal) < 1.0
                    break;
                end
                
                % FIX: Detect local minima (stuck)
                if norm(pos - prev_pos) < 1e-4
                    stuck_count = stuck_count + 1;
                    if stuck_count > 50
                        % Attempt random perturbation to escape
                        perturb = randn(2, 1) * 2.0;
                        test_pos = pos + perturb;
                        if ~obj.env.in_collision(test_pos)
                            pos = test_pos;
                            path = [path, pos];
                        end
                        stuck_count = 0;
                    end
                else
                    stuck_count = 0;
                end
                prev_pos = pos;
            end
            
            % Warning if goal not reached
            if norm(pos - goal) >= 1.0
                warning('PotentialField:GoalNotReached', ...
                    'Path planning stopped without reaching goal (dist=%.2f)', ...
                    norm(pos - goal));
            end
        end
    end
end
