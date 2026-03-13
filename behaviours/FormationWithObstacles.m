classdef FormationWithObstacles < handle
    properties
        formation_controller
        k_obstacle = 50.0;
        d_safe = 3.0;
    end
    
    methods
        function obj = FormationWithObstacles(formation_ctrl, k_obs, d_safe)
            obj.formation_controller = formation_ctrl;
            if nargin > 1, obj.k_obstacle = k_obs; end
            if nargin > 2, obj.d_safe = d_safe; end
        end
        
        function u = compute_control(obj, swarm, env)
            u_formation = obj.formation_controller.compute_control(swarm, env);
            
            for i = 1:swarm.N
                agent = swarm.agents{i};
                pos = agent.state(1:2);
                
                f_obs = obj.obstacle_avoidance_force(pos, env);
                u_formation(:, i) = u_formation(:, i) + f_obs;
                
                max_accel = 3.0;
                if norm(u_formation(:,i)) > max_accel
                    u_formation(:,i) = u_formation(:,i) / norm(u_formation(:,i)) * max_accel;
                end
            end
            
            u = u_formation;
        end
        
        function f = obstacle_avoidance_force(obj, pos, env)
            f = zeros(2, 1);
            
            for i = 1:length(env.obstacles)
                obs = env.obstacles{i};
                
                if strcmp(obs.type, 'circle')
                    diff = pos - obs.center;
                    d = norm(diff) - obs.radius;
                    
                    if d < obj.d_safe && d > 0
                        f = f + obj.k_obstacle * (1/d - 1/obj.d_safe) * (diff/norm(diff));
                    end
                    
                elseif strcmp(obs.type, 'rectangle')
                    closest = [max(obs.bounds(1,1), min(pos(1), obs.bounds(1,2)));
                               max(obs.bounds(2,1), min(pos(2), obs.bounds(2,2)))];
                    diff = pos - closest;
                    d = norm(diff);
                    
                    if d < obj.d_safe && d > 0.1
                        f = f + obj.k_obstacle * (1/d - 1/obj.d_safe) * (diff/d);
                    end
                end
            end
        end
    end
end
