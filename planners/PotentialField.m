classdef PotentialField < handle
    properties
        k_att = 1.0;    % attractive gain
        k_rep = 50.0;   % repulsive gain
        d0 = 5.0;       % obstacle influence distance
        env
    end
    
    methods
        function obj = PotentialField(environment, k_att, k_rep, d0)
            obj.env = environment;
            if nargin > 1, obj.k_att = k_att; end
            if nargin > 2, obj.k_rep = k_rep; end
            if nargin > 3, obj.d0 = d0; end
        end
        
        function force = compute_force(obj, pos, goal)
            f_att = obj.attractive_force(pos, goal);
            f_rep = obj.repulsive_force(pos);
            force = f_att + f_rep;
        end
        
        function f = attractive_force(obj, pos, goal)
            f = obj.k_att * (goal - pos);
        end
        
        function f = repulsive_force(obj, pos)
            f = zeros(2, 1);
            
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                
                if strcmp(obs.type, 'circle')
                    diff = pos - obs.center;
                    d = norm(diff) - obs.radius;
                    
                    if d < obj.d0 && d > 0
                        f = f + obj.k_rep * (1/d - 1/obj.d0) * (1/d^2) * (diff/norm(diff));
                    end
                    
                elseif strcmp(obs.type, 'rectangle')
                    closest = [max(obs.bounds(1,1), min(pos(1), obs.bounds(1,2)));
                               max(obs.bounds(2,1), min(pos(2), obs.bounds(2,2)))];
                    diff = pos - closest;
                    d = norm(diff);
                    
                    if d < obj.d0 && d > 0
                        f = f + obj.k_rep * (1/d - 1/obj.d0) * (1/d^2) * (diff/d);
                    end
                end
            end
        end
        
        function path = plan(obj, start_pos, goal_pos, max_steps, dt)
            if nargin < 4, max_steps = 1000; end
            if nargin < 5, dt = 0.1; end
            
            path = start_pos;
            pos = start_pos;
            
            for step = 1:max_steps
                force = obj.compute_force(pos, goal_pos);
                vel = force;
                
                if norm(vel) > 5.0
                    vel = vel / norm(vel) * 5.0;
                end
                
                pos = pos + vel * dt;
                path = [path, pos];
                
                if norm(pos - goal_pos) < 1.0
                    break;
                end
            end
        end
    end
end
